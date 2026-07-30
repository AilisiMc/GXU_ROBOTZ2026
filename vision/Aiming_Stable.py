#!/usr/bin/env python3
"""A4 靶纸识别 + 稳定性增强版视觉子状态机。

在 aiming_state_machine.py 基础上增加：
- 空间一致性闸门：拒绝位置或面积突变的虚假检测
- 连续确认机制：首次捕获和跳变后的重捕获均需连续多帧验证
- 指数平滑：抑制轮廓小幅抖动
- 检测置信度过滤：利用评分函数排除低分候选
- 安全 HOLD：旧框只用于显示，不向 STM32 发送有效误差
- CRLF 行尾修复（兼容 STM32 CDC）
- 激光落点残差补偿：在 32 mm 物理偏置基础上叠加像素级安装偏差

运行：
  python3 vision/aiming_state_machine_stable.py --camera 4 --port /dev/ttyACM0
  python3 vision/aiming_state_machine_stable.py --camera 4 --dry-run --auto-start A

新增参数：
  --spatial-gate-px    相邻可靠位置允许的最大跳变，默认 60 像素
  --min-score          最低检测置信度，默认 30
  --confirm-frames     首次捕获/重捕获所需连续帧数，默认 3
  --max-area-ratio     相邻可靠框允许的最大面积倍数，默认 1.8
  --alpha              当前检测参与平滑的比例，默认 0.40
  --calibration-log    可选的人工标注CSV记录；t=靶纸，f=异物，n=暂停
  --aim-offset-x/y     激光落点残差补偿，当前标定默认 (-7, +33) 像素
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from collections import deque
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "serial_comm"))

from aiming_protocol_fixed import decode_control, encode_aim_status
from duplex_serial import DuplexSerial


VIEW = "A4 aiming (stable)"
MASK_VIEW = "Black mask"
PAPER_MASK_VIEW = "White paper mask"
CONTROLS = "Controls"
A4_SHORT_LONG_RATIO = 210.0 / 297.0
A4_INNER_SHORT_LONG_RATIO = 174.0 / 261.0
DEFAULTS = {
    "Black max": 85,
    "White S max": 110,
    "White V min": 105,
    "Min area": 700,
    "Close size": 5,
    "Hold frames": 5,
}
# 2026-07-27，B 点实拍初标：
# 实际激光点相对黄色准星约左 7 px、下 33 px。瞄准点按相同方向修正，
# 让闭环稳定后云台把实际激光点移回靶心。后续可用命令行参数继续微调。
DEFAULT_AIM_OFFSET_X_PX = -7
DEFAULT_AIM_OFFSET_Y_PX = 33


def quadrilateral_metrics(corners: np.ndarray) -> tuple[float, float]:
    """返回四边形面积和投影短边/长边比。"""
    ordered = corners.reshape(4, 2).astype(np.float32)
    area = abs(float(cv2.contourArea(ordered)))
    sides = np.array([
        np.linalg.norm(ordered[1] - ordered[0]),
        np.linalg.norm(ordered[2] - ordered[1]),
        np.linalg.norm(ordered[3] - ordered[2]),
        np.linalg.norm(ordered[0] - ordered[3]),
    ])
    horizontal = float((sides[0] + sides[2]) / 2.0)
    vertical = float((sides[1] + sides[3]) / 2.0)
    longest = max(horizontal, vertical)
    ratio = min(horizontal, vertical) / longest if longest > 0 else 0.0
    return area, ratio


class CalibrationRecorder:
    """可选的人工标注采样器；默认不启用，也不影响视觉控制。"""

    FIELDNAMES = (
        "timestamp",
        "elapsed_s",
        "point",
        "label",
        "cx",
        "cy",
        "area_px2",
        "short_long_ratio",
        "score",
        "filter_status",
    )

    def __init__(self, path: Path, image_width: int, image_height: int) -> None:
        self.path = path
        self.image_width = image_width
        self.image_height = image_height
        self.label: Optional[str] = None
        self.sample_count = 0
        self._samples: list[dict[str, float | str]] = []
        self._started = time.perf_counter()

        path.parent.mkdir(parents=True, exist_ok=True)
        self._file = path.open("x", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()

    def set_label(self, label: Optional[str]) -> None:
        if label not in (None, "target", "foreign"):
            raise ValueError(f"未知标定标签：{label}")
        self.label = label

    def record(
        self,
        point: Optional[str],
        detection: Optional[tuple[np.ndarray, np.ndarray, float]],
        filter_status: str,
    ) -> None:
        # 只有操作者明确选择标签且当前点位有效时才采样，防止把异物自动
        # 当成真实靶纸并据此生成危险的 ROI。
        if self.label is None or point is None or detection is None:
            return

        corners, center, score = detection
        area, ratio = quadrilateral_metrics(corners)
        row: dict[str, float | str] = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "elapsed_s": round(time.perf_counter() - self._started, 3),
            "point": point,
            "label": self.label,
            "cx": round(float(center[0]), 2),
            "cy": round(float(center[1]), 2),
            "area_px2": round(area, 2),
            "short_long_ratio": round(ratio, 4),
            "score": round(float(score), 2),
            "filter_status": filter_status,
        }
        self._writer.writerow(row)
        self._samples.append(row)
        self.sample_count += 1
        if self.sample_count % 30 == 0:
            self._file.flush()

    def _build_summary(self) -> str:
        lines = [
            f"标定文件：{self.path}",
            "建议值仅使用人工标为 target 的样本；启用前仍需逐点实测验证。",
            "计算规则：中心P5～P95外扩50px；面积P5×0.80～P95×1.25。",
            "",
        ]
        target_rows = [row for row in self._samples if row["label"] == "target"]
        for point in ("A", "C", "D", "B"):
            rows = [row for row in target_rows if row["point"] == point]
            if not rows:
                lines.append(f"{point}点：没有 target 样本")
                continue

            xs = np.array([float(row["cx"]) for row in rows], dtype=np.float64)
            ys = np.array([float(row["cy"]) for row in rows], dtype=np.float64)
            areas = np.array([float(row["area_px2"]) for row in rows], dtype=np.float64)
            ratios = np.array(
                [float(row["short_long_ratio"]) for row in rows], dtype=np.float64
            )

            x05, x95 = np.percentile(xs, [5, 95])
            y05, y95 = np.percentile(ys, [5, 95])
            area05, area95 = np.percentile(areas, [5, 95])
            ratio05, ratio95 = np.percentile(ratios, [5, 95])
            roi_x1 = max(0, int(math.floor(x05 - 50)))
            roi_y1 = max(0, int(math.floor(y05 - 50)))
            roi_x2 = min(self.image_width - 1, int(math.ceil(x95 + 50)))
            roi_y2 = min(self.image_height - 1, int(math.ceil(y95 + 50)))
            area_min = max(1, int(math.floor(area05 * 0.80)))
            area_max = max(area_min, int(math.ceil(area95 * 1.25)))

            lines.extend([
                f"{point}点：target样本 {len(rows)} 帧",
                f"  建议ROI：x={roi_x1}～{roi_x2}, y={roi_y1}～{roi_y2}",
                f"  建议面积：{area_min}～{area_max} px²",
                f"  实测比例P5～P95：{ratio05:.3f}～{ratio95:.3f}",
            ])
        return "\n".join(lines) + "\n"

    def close(self) -> Optional[Path]:
        if self._file.closed:
            return None
        self._file.flush()
        self._file.close()
        summary_path = self.path.with_suffix(".summary.txt")
        summary_path.write_text(self._build_summary(), encoding="utf-8")
        return summary_path


class VisionState(Enum):
    WAIT = "WAIT"
    SEARCH = "SEARCH"
    TRACK = "TRACK"
    LOCKED = "LOCKED"


class VisionStateMachine:
    """视觉端只判断目标和图像稳定性，不控制云台或激光。"""

    def __init__(self, lock_frames: int, lock_threshold: int, jitter_limit: int) -> None:
        self.lock_frames = lock_frames
        self.lock_threshold = lock_threshold
        self.jitter_limit = jitter_limit
        self.state = VisionState.WAIT
        self.point: Optional[str] = None
        self._errors: deque[tuple[int, int]] = deque(maxlen=lock_frames)

    def start(self, point: str) -> None:
        self.point = point
        self.state = VisionState.SEARCH
        self._errors.clear()

    def stop(self) -> None:
        self.point = None
        self.state = VisionState.WAIT
        self._errors.clear()

    @property
    def lock_count(self) -> int:
        return len(self._errors)

    def update(self, found: bool, error_x: int, error_y: int) -> tuple[bool, bool]:
        if self.state == VisionState.WAIT:
            self._errors.clear()
            return False, False

        if not found:
            self._errors.clear()
            self.state = VisionState.SEARCH
            return False, False

        self.state = VisionState.TRACK
        inside = abs(error_x) <= self.lock_threshold and abs(error_y) <= self.lock_threshold
        if not inside:
            self._errors.clear()
            return True, False

        self._errors.append((error_x, error_y))
        if len(self._errors) < self.lock_frames:
            return True, False

        xs = [item[0] for item in self._errors]
        ys = [item[1] for item in self._errors]
        stable = (max(xs) - min(xs) <= self.jitter_limit) and (
            max(ys) - min(ys) <= self.jitter_limit
        )
        if stable:
            self.state = VisionState.LOCKED
        return True, stable


# ---------------------------------------------------------------------------
# 检测稳定性过滤器
# ---------------------------------------------------------------------------

class DetectionStabilizer:
    """防止云台运动时检测框跳到背景异物。

    首次候选不会立即控制云台；只有连续 ``confirm_frames`` 帧位置一致才会
    变为 TRUSTED。已经跟踪后，位置或面积突变的候选进入重新确认，在确认
    期间旧框可用于画面观察，但状态为 HOLD，绝不能作为有效控制量发送。
    """

    def __init__(
        self,
        spatial_gate_px: int,
        min_score: float,
        alpha: float,
        confirm_frames: int,
        max_area_ratio: float,
    ) -> None:
        self.spatial_gate = spatial_gate_px
        self.min_score = min_score
        self.alpha = alpha
        self.confirm_frames = confirm_frames
        self.max_area_ratio = max_area_ratio

        # 平滑后的位置（对外输出的唯一坐标来源）
        self.smoothed_center: Optional[np.ndarray] = None
        self.smoothed_corners: Optional[np.ndarray] = None

        # 尚未取得信任、或发生跳变后等待重新确认的候选。
        self.pending_center: Optional[np.ndarray] = None
        self.pending_corners: Optional[np.ndarray] = None
        self._pending_count = 0

        # 已信任轨迹的状态
        self._miss_count = 0
        self._acquisition_count = 0
        self._jump_count = 0

        # 对外报告
        self.status = "INIT"
        self.last_score = 0.0

    def reset(self) -> None:
        """完全重置，用于 STOP 或目标丢失超时。"""
        self.smoothed_center = None
        self.smoothed_corners = None
        self._clear_pending()
        self._miss_count = 0
        self._acquisition_count = 0
        self._jump_count = 0
        self.status = "INIT"

    @property
    def pending_count(self) -> int:
        return self._pending_count

    @staticmethod
    def _quad_area(corners: np.ndarray) -> float:
        return abs(float(cv2.contourArea(corners.astype(np.float32))))

    def _clear_pending(self) -> None:
        self.pending_center = None
        self.pending_corners = None
        self._pending_count = 0

    def _pending_is_consistent(
        self, center: np.ndarray, corners: np.ndarray
    ) -> bool:
        if self.pending_center is None or self.pending_corners is None:
            return False
        center_jump = float(np.linalg.norm(center - self.pending_center))
        old_area = self._quad_area(self.pending_corners)
        new_area = self._quad_area(corners)
        if old_area <= 1.0 or new_area <= 1.0:
            return False
        area_ratio = max(old_area, new_area) / min(old_area, new_area)
        # 候选确认使用比正常跟踪稍紧的空间限制，防止两个背景框轮流出现
        # 时也被累计成“连续确认”。
        pending_gate = max(20.0, self.spatial_gate * 0.65)
        return center_jump <= pending_gate and area_ratio <= self.max_area_ratio

    def _observe_pending(
        self, corners: np.ndarray, center: np.ndarray
    ) -> bool:
        """累计待确认候选；达到确认帧数时返回 True。"""
        if not self._pending_is_consistent(center, corners):
            self.pending_center = center.copy()
            self.pending_corners = corners.copy()
            self._pending_count = 1
        else:
            # 对待确认候选也做轻度平均，避免确认瞬间坐标突然跳动。
            self.pending_center = 0.5 * center + 0.5 * self.pending_center
            self.pending_corners = 0.5 * corners + 0.5 * self.pending_corners
            self._pending_count += 1
        return self._pending_count >= self.confirm_frames

    def _accept_pending(
        self,
    ) -> tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
        if self.pending_center is None or self.pending_corners is None:
            self.status = "INIT"
            return None, None, "INIT"
        self.smoothed_center = self.pending_center.copy()
        self.smoothed_corners = self.pending_corners.copy()
        self._acquisition_count = self._pending_count
        self._clear_pending()
        self._miss_count = 0
        self._jump_count = 0
        self.status = "TRUSTED"
        return self.smoothed_corners, self.smoothed_center, "TRUSTED"

    def _tracking_candidate_is_consistent(
        self, center: np.ndarray, corners: np.ndarray
    ) -> bool:
        if self.smoothed_center is None or self.smoothed_corners is None:
            return False
        center_jump = float(np.linalg.norm(center - self.smoothed_center))
        old_area = self._quad_area(self.smoothed_corners)
        new_area = self._quad_area(corners)
        if old_area <= 1.0 or new_area <= 1.0:
            return False
        area_ratio = max(old_area, new_area) / min(old_area, new_area)
        return center_jump <= self.spatial_gate and area_ratio <= self.max_area_ratio

    def update(
        self,
        detection: Optional[tuple[np.ndarray, np.ndarray, float]],
        hold_limit: int,
    ) -> tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
        """输入原始检测，输出过滤后的 (corners, center, status)。

        status 值：
          TRUSTED    — 通过所有检查，位置可靠，正常更新平滑
          HOLD       — 短暂漏检/低分/跳变，使用旧位置，不更新平滑
          ACQUIRING  — 正在连续确认新目标，尚无可用控制位置
          LOST       — 彻底丢失，无可用位置
          INIT       — 尚未捕获过目标
        """
        if detection is None:
            return self._handle_miss(hold_limit)

        corners, center, score = detection
        self.last_score = score

        # 置信度过低 — 视为漏检，保持旧位置
        if score < self.min_score:
            return self._handle_miss(hold_limit)

        # 首次捕获必须连续确认，不能让第一帧误识别直接驱动云台。
        if self.smoothed_center is None:
            if self._observe_pending(corners, center):
                return self._accept_pending()
            self.status = "ACQUIRING"
            return None, None, "ACQUIRING"

        if not self._tracking_candidate_is_consistent(center, corners):
            # 跳变不能清零丢失计数。旧实现会在每次跳变前把 miss_count 置零，
            # 导致错误旧框能够被 HOLD 无限期保留。
            self._miss_count += 1
            self._jump_count += 1
            if self._observe_pending(corners, center):
                return self._accept_pending()
            if self._miss_count > hold_limit:
                self.smoothed_center = None
                self.smoothed_corners = None
                self._acquisition_count = 0
                self.status = "ACQUIRING"
                return None, None, "ACQUIRING"
            self.status = "HOLD"
            return self.smoothed_corners, self.smoothed_center, "HOLD"

        # 正常跟踪：只有被接受的新检测才清零 miss_count。
        self._miss_count = 0
        self._jump_count = 0
        self._clear_pending()
        self._acquisition_count += 1

        self.smoothed_center = (
            self.alpha * center + (1.0 - self.alpha) * self.smoothed_center
        )
        self.smoothed_corners = (
            self.alpha * corners + (1.0 - self.alpha) * self.smoothed_corners
        )
        self.status = "TRUSTED"
        return self.smoothed_corners, self.smoothed_center, "TRUSTED"

    def _handle_miss(
        self, hold_limit: int
    ) -> tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
        self._miss_count += 1
        # 连续确认必须由连续有效帧组成，漏检会打断候选确认。
        self._clear_pending()

        if (
            self.smoothed_center is not None
            and self.smoothed_corners is not None
            and self._miss_count <= hold_limit
        ):
            self.status = "HOLD"
            return self.smoothed_corners, self.smoothed_center, "HOLD"

        self.smoothed_center = None
        self.smoothed_corners = None
        self._acquisition_count = 0
        self._jump_count = 0
        self.status = "LOST" if self._miss_count > hold_limit else "INIT"
        return None, None, self.status


# ---------------------------------------------------------------------------
# 检测函数（从 rectangle_target.py 复制，增加 score 返回值）
# ---------------------------------------------------------------------------

def _nothing(_: int) -> None:
    pass


def create_controls() -> None:
    cv2.namedWindow(CONTROLS, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROLS, 430, 245)
    cv2.createTrackbar("Black max", CONTROLS, DEFAULTS["Black max"], 255, _nothing)
    cv2.createTrackbar("White S max", CONTROLS, DEFAULTS["White S max"], 255, _nothing)
    cv2.createTrackbar("White V min", CONTROLS, DEFAULTS["White V min"], 255, _nothing)
    cv2.createTrackbar("Min area", CONTROLS, DEFAULTS["Min area"], 20000, _nothing)
    cv2.createTrackbar("Close size", CONTROLS, DEFAULTS["Close size"], 15, _nothing)
    cv2.createTrackbar("Hold frames", CONTROLS, DEFAULTS["Hold frames"], 30, _nothing)


def reset_controls() -> None:
    for name, value in DEFAULTS.items():
        cv2.setTrackbarPos(name, CONTROLS, value)


def open_camera(index: int, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开 /dev/video{index}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ok, _ = cap.read()
    if not ok:
        cap.release()
        raise RuntimeError("摄像头已打开但无法读取画面")
    return cap


def order_corners(points: np.ndarray) -> np.ndarray:
    points = points.reshape(4, 2).astype(np.float32)
    centroid = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - centroid[1], points[:, 0] - centroid[0])
    ordered = points[np.argsort(angles)]
    ordered = np.roll(ordered, -int(np.argmin(np.sum(ordered, axis=1))), axis=0)
    edge_1 = ordered[1] - ordered[0]
    edge_2 = ordered[2] - ordered[1]
    cross_z = edge_1[0] * edge_2[1] - edge_1[1] * edge_2[0]
    if cross_z < 0:
        ordered = ordered[[0, 3, 2, 1]]
    return ordered


def line_intersection(
    a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray
) -> Optional[np.ndarray]:
    x1, y1 = a
    x2, y2 = b
    x3, y3 = c
    x4, y4 = d
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 1e-6:
        return None
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator
    return np.array([px, py], dtype=np.float32)


def _target_candidate(
    contour: np.ndarray,
    min_area: float,
    white_mask: np.ndarray,
    black_mask: np.ndarray,
) -> Optional[tuple[np.ndarray, np.ndarray, float]]:
    area = cv2.contourArea(contour)
    if area < min_area:
        return None
    frame_h, frame_w = white_mask.shape[:2]
    x, y, width, height = cv2.boundingRect(contour)
    margin = 3
    if x <= margin or y <= margin or x + width >= frame_w - margin or y + height >= frame_h - margin:
        return None
    perimeter = cv2.arcLength(contour, True)
    polygon = cv2.approxPolyDP(contour, 0.030 * perimeter, True)
    if len(polygon) == 4 and cv2.isContourConvex(polygon):
        corners = order_corners(polygon)
    else:
        rotated_rect = cv2.minAreaRect(contour)
        rect_width, rect_height = rotated_rect[1]
        rect_area = rect_width * rect_height
        if min(rect_width, rect_height) < 15 or rect_area <= 0:
            return None
        if area / rect_area < 0.78 or len(polygon) > 8:
            return None
        corners = order_corners(cv2.boxPoints(rotated_rect))
    sides = np.array([
        np.linalg.norm(corners[1] - corners[0]),
        np.linalg.norm(corners[2] - corners[1]),
        np.linalg.norm(corners[3] - corners[2]),
        np.linalg.norm(corners[0] - corners[3]),
    ])
    if np.min(sides) < 15:
        return None
    horizontal = (sides[0] + sides[2]) / 2.0
    vertical = (sides[1] + sides[3]) / 2.0
    observed_ratio = min(horizontal, vertical) / max(horizontal, vertical)
    if not 0.32 <= observed_ratio <= 1.0:
        return None
    center = line_intersection(corners[0], corners[2], corners[1], corners[3])
    if center is None:
        return None
    warp_width, warp_height = 297, 210
    destination = np.float32([
        [0, 0],
        [warp_width - 1, 0],
        [warp_width - 1, warp_height - 1],
        [0, warp_height - 1],
    ])
    paper_transform = cv2.getPerspectiveTransform(corners, destination)
    warped_white = cv2.warpPerspective(
        white_mask, paper_transform, (warp_width, warp_height), flags=cv2.INTER_NEAREST
    )
    inner_white_ratio = float(np.mean(warped_white[15:-15, 20:-20]) / 255.0)
    if inner_white_ratio < 0.72:
        return None
    outer_corners = center + 1.18 * (corners - center)
    border_transform = cv2.getPerspectiveTransform(outer_corners.astype(np.float32), destination)
    warped_black = cv2.warpPerspective(
        black_mask, border_transform, (warp_width, warp_height), flags=cv2.INTER_NEAREST
    )
    band_y, band_x = 20, 30
    side_black = np.array([
        np.mean(warped_black[:band_y, band_x:-band_x]) / 255.0,
        np.mean(warped_black[-band_y:, band_x:-band_x]) / 255.0,
        np.mean(warped_black[band_y:-band_y, :band_x]) / 255.0,
        np.mean(warped_black[band_y:-band_y, -band_x:]) / 255.0,
    ])
    if np.count_nonzero(side_black >= 0.35) < 3:
        return None
    if np.min(side_black) < 0.18 or np.mean(side_black) < 0.43:
        return None
    ratio_error = abs(math.log(observed_ratio / A4_INNER_SHORT_LONG_RATIO))
    border_score = float(np.mean(side_black))
    score = math.sqrt(area) * border_score**2 * inner_white_ratio * math.exp(-2.0 * ratio_error)
    return outer_corners, center, score


def _black_ring_candidate(
    contours: list[np.ndarray],
    hierarchy: np.ndarray,
    index: int,
    min_area: float,
    gray: np.ndarray,
    black_max: int,
) -> Optional[tuple[np.ndarray, np.ndarray, float]]:
    parent_index = int(hierarchy[index][3])
    if parent_index < 0:
        return None
    inner = contours[index]
    inner_area = cv2.contourArea(inner)
    if inner_area < min_area:
        return None
    frame_h, frame_w = gray.shape[:2]
    x, y, width, height = cv2.boundingRect(inner)
    margin = 3
    if x <= margin or y <= margin or x + width >= frame_w - margin or y + height >= frame_h - margin:
        return None
    # 红色靶环在灰度较低时也会进入 black_mask；当最外圈恰好接触黑框，
    # 内孔轮廓会出现数个很小的向内凹口，原始 approxPolyDP 因而在 4、7、
    # 10 个顶点之间跳变。黑框内沿本身应接近凸四边形，先取凸包可消除
    # 这些内部凹口，同时保留透视形成的梯形，避免用 minAreaRect 丢失透视。
    hull = cv2.convexHull(inner)
    hull_area = cv2.contourArea(hull)
    if hull_area <= 0 or inner_area / hull_area < 0.90:
        return None
    perimeter = cv2.arcLength(hull, True)
    polygon = cv2.approxPolyDP(hull, 0.025 * perimeter, True)
    if len(polygon) != 4 or not cv2.isContourConvex(polygon):
        return None
    corners = order_corners(polygon)
    sides = np.array([
        np.linalg.norm(corners[1] - corners[0]),
        np.linalg.norm(corners[2] - corners[1]),
        np.linalg.norm(corners[3] - corners[2]),
        np.linalg.norm(corners[0] - corners[3]),
    ])
    if np.min(sides) < 15:
        return None
    horizontal = (sides[0] + sides[2]) / 2.0
    vertical = (sides[1] + sides[3]) / 2.0
    observed_ratio = min(horizontal, vertical) / max(horizontal, vertical)
    if not 0.30 <= observed_ratio <= 1.0:
        return None
    parent_area = cv2.contourArea(contours[parent_index])
    ring_ratio = parent_area / inner_area if inner_area > 0 else 0.0
    if not 1.08 <= ring_ratio <= 4.0:
        return None
    center = line_intersection(corners[0], corners[2], corners[1], corners[3])
    if center is None:
        return None
    interior_mask = np.zeros_like(gray)
    cv2.fillConvexPoly(interior_mask, corners.astype(np.int32), 255)
    erosion = max(3, int(round(np.min(sides) * 0.04)))
    if erosion % 2 == 0:
        erosion += 1
    interior_mask = cv2.erode(
        interior_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (erosion, erosion))
    )
    interior_pixels = gray[interior_mask > 0]
    if interior_pixels.size == 0 or float(np.mean(interior_pixels)) < max(95, black_max + 20):
        return None
    outer_corners = center + 1.16 * (corners - center)
    ratio_error = abs(math.log(observed_ratio / A4_INNER_SHORT_LONG_RATIO))
    score = math.sqrt(inner_area) * math.exp(-0.8 * ratio_error)
    return outer_corners, center, score


def detect_a4_rectangle_with_score(
    frame: np.ndarray, minimum_score: float = 0.0
) -> Optional[tuple[np.ndarray, np.ndarray, float]]:
    """与 detect_a4_rectangle 相同，但额外返回置信度评分。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    black_max = cv2.getTrackbarPos("Black max", CONTROLS)
    _, black_mask = cv2.threshold(gray, black_max, 255, cv2.THRESH_BINARY_INV)
    white_s_max = cv2.getTrackbarPos("White S max", CONTROLS)
    white_v_min = cv2.getTrackbarPos("White V min", CONTROLS)
    close_size = max(1, cv2.getTrackbarPos("Close size", CONTROLS))
    if close_size % 2 == 0:
        close_size += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (close_size, close_size))
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

    black_contours, black_hierarchy = cv2.findContours(
        black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    black_best: Optional[tuple[np.ndarray, np.ndarray, float]] = None
    if black_hierarchy is not None:
        hierarchy_rows = black_hierarchy[0]
        for i in range(len(black_contours)):
            candidate = _black_ring_candidate(
                black_contours, hierarchy_rows, i,
                max(1, cv2.getTrackbarPos("Min area", CONTROLS)),
                gray, black_max,
            )
            if candidate is not None and (black_best is None or candidate[2] > black_best[2]):
                black_best = candidate

    white_close_size = max(5, close_size)
    white_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (white_close_size, white_close_size))
    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    saturation_limits = sorted({min(70, white_s_max), white_s_max})
    white_masks = []
    for sat_max in saturation_limits:
        wm = cv2.inRange(hsv, np.array([0, 0, white_v_min], dtype=np.uint8),
                         np.array([179, sat_max, 255], dtype=np.uint8))
        wm = cv2.morphologyEx(wm, cv2.MORPH_OPEN, open_kernel)
        wm = cv2.morphologyEx(wm, cv2.MORPH_CLOSE, white_kernel)
        white_masks.append(wm)

    min_area = max(1, cv2.getTrackbarPos("Min area", CONTROLS))
    best: Optional[tuple[np.ndarray, np.ndarray, float]] = None
    for wm in white_masks:
        contours, _ = cv2.findContours(wm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            candidate = _target_candidate(contour, min_area, wm, black_mask)
            if candidate is not None and (best is None or candidate[2] > best[2]):
                best = candidate

    cv2.imshow(MASK_VIEW, black_mask)
    cv2.imshow(PAPER_MASK_VIEW, white_masks[-1])
    # 白纸候选经过“内部为白色 + 至少三边为黑色”的双重验证，比仅依赖
    # 黑色内孔层级的候选更不容易把门框、显示器或纸板边缘当成靶纸。
    # 分数不足的白纸候选本来就会被稳定器拒绝，此时才使用黑框路径兜底。
    if best is not None and best[2] >= minimum_score:
        return best
    if black_best is not None:
        return black_best
    if best is not None:
        return best
    return None


def cross(
    image: np.ndarray, point: tuple[int, int],
    color: tuple[int, int, int], size: int = 14
) -> None:
    x, y = point
    cv2.line(image, (x - size, y), (x + size, y), color, 2)
    cv2.line(image, (x, y - size), (x, y + size), color, 2)


def compensated_aim_point(
    image_width: int, image_height: int,
    corners: Optional[np.ndarray],
    laser_offset_x_mm: float, laser_offset_y_mm: float,
    target_width_mm: float, target_height_mm: float,
    residual_offset_x_px: int, residual_offset_y_px: int,
) -> tuple[tuple[int, int], np.ndarray]:
    base = np.array([image_width / 2.0, image_height / 2.0], dtype=np.float32)
    physical_offset_px = np.zeros(2, dtype=np.float32)
    if corners is not None:
        ordered = corners.reshape(4, 2).astype(np.float32)
        target_plane = np.float32([
            [0.0, 0.0], [target_width_mm, 0.0],
            [target_width_mm, target_height_mm], [0.0, target_height_mm],
        ])
        plane_to_image = cv2.getPerspectiveTransform(target_plane, ordered)
        target_center_mm = np.array(
            [[[target_width_mm * 0.5, target_height_mm * 0.5]]], dtype=np.float32
        )
        camera_sight_mm = target_center_mm.copy()
        camera_sight_mm[0, 0, 0] -= laser_offset_x_mm
        camera_sight_mm[0, 0, 1] -= laser_offset_y_mm
        projected = cv2.perspectiveTransform(
            np.concatenate((target_center_mm, camera_sight_mm), axis=1), plane_to_image
        )[0]
        physical_offset_px = projected[0] - projected[1]
    residual = np.array([residual_offset_x_px, residual_offset_y_px], dtype=np.float32)
    total_offset = physical_offset_px + residual
    aim = np.rint(base + total_offset).astype(np.int32)
    return (int(aim[0]), int(aim[1])), physical_offset_px


# ---------------------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="A4 靶纸视觉子状态机（稳定性增强版）")
    parser.add_argument("--camera", type=int, default=4)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--port", help="真实串口，例如 /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true", help="不打开串口，只打印状态机输出")
    parser.add_argument("--send-rate", type=float, default=20.0)
    parser.add_argument("--auto-start", choices=["A", "B", "C", "D"])
    parser.add_argument("--lock-frames", type=int, default=5)
    parser.add_argument("--lock-threshold", type=int, default=10)
    parser.add_argument("--jitter-limit", type=int, default=8)
    parser.add_argument("--laser-offset-x-mm", type=float, default=0.0)
    parser.add_argument("--laser-offset-y-mm", type=float, default=32.0)
    parser.add_argument("--target-width-mm", type=float, default=297.0)
    parser.add_argument("--target-height-mm", type=float, default=210.0)
    parser.add_argument(
        "--aim-offset-x",
        type=int,
        default=DEFAULT_AIM_OFFSET_X_PX,
        help=(
            "黄色准星的水平残差补偿（像素）；负值向左。"
            f"默认 {DEFAULT_AIM_OFFSET_X_PX}"
        ),
    )
    parser.add_argument(
        "--aim-offset-y",
        type=int,
        default=DEFAULT_AIM_OFFSET_Y_PX,
        help=(
            "黄色准星的垂直残差补偿（像素）；正值向下。"
            f"默认 {DEFAULT_AIM_OFFSET_Y_PX}"
        ),
    )
    # ---- 稳定性新增参数 ----
    parser.add_argument("--spatial-gate-px", type=int, default=60,
                        help="空间闸门（像素）：超过此距离的检测视为假检测，拒绝")
    parser.add_argument("--min-score", type=float, default=30,
                        help="最低检测置信度，低于此值视为漏检")
    parser.add_argument("--confirm-frames", type=int, default=3,
                        help="首次捕获或跳变后重捕获所需的连续一致帧数")
    parser.add_argument("--max-area-ratio", type=float, default=1.8,
                        help="相邻可靠矩形允许的最大面积倍数")
    parser.add_argument("--alpha", type=float, default=0.40,
                        help="指数平滑中当前检测所占比例（0-1）")
    parser.add_argument(
        "--calibration-log",
        nargs="?",
        const="AUTO",
        metavar="CSV",
        help=(
            "启用人工标注的CSV标定记录；省略路径时自动保存到 "
            "vision/calibration_logs/"
        ),
    )
    args = parser.parse_args()

    if args.send_rate <= 0:
        parser.error("--send-rate 必须大于 0")
    if args.lock_frames <= 0 or args.confirm_frames <= 0:
        parser.error("--lock-frames 和 --confirm-frames 必须大于 0")
    if args.spatial_gate_px <= 0:
        parser.error("--spatial-gate-px 必须大于 0")
    if args.min_score < 0:
        parser.error("--min-score 不能为负数")
    if args.max_area_ratio < 1.0:
        parser.error("--max-area-ratio 不能小于 1")
    if not 0.0 < args.alpha <= 1.0:
        parser.error("--alpha 必须在 (0, 1] 范围内")
    if args.lock_threshold < 0 or args.jitter_limit < 0:
        parser.error("锁定阈值不能为负数")
    if args.target_width_mm <= 0 or args.target_height_mm <= 0:
        parser.error("靶框实际宽度和高度必须大于 0")
    serial_enabled = args.dry_run or args.port is not None

    calibration_path: Optional[Path] = None
    if args.calibration_log is not None:
        if args.calibration_log == "AUTO":
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            calibration_path = (
                Path(__file__).parent
                / "calibration_logs"
                / f"target_calibration_{stamp}.csv"
            )
        else:
            calibration_path = Path(args.calibration_log).expanduser()
        if calibration_path.exists():
            parser.error(f"标定文件已存在，不会覆盖：{calibration_path}")

    cap = open_camera(args.camera, args.width, args.height)
    actual = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    codec = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))
    print(
        f"/dev/video{args.camera}: {actual[0]}x{actual[1]}, codec={codec}; "
        "a/b/c/d START，x STOP，q 退出，s 保存，r 重置。"
    )
    print(
        f"[STABLE] spatial_gate={args.spatial_gate_px}px  "
        f"min_score={args.min_score:.0f}  confirm={args.confirm_frames}  "
        f"area_ratio={args.max_area_ratio:.2f}  alpha={args.alpha:.2f}"
    )

    create_controls()
    cv2.namedWindow(VIEW, cv2.WINDOW_NORMAL)
    cv2.namedWindow(MASK_VIEW, cv2.WINDOW_NORMAL)
    capture_dir = Path(__file__).parent / "fsm_captures"
    recorder: Optional[CalibrationRecorder] = None
    if calibration_path is not None:
        try:
            recorder = CalibrationRecorder(calibration_path, actual[0], actual[1])
        except Exception:
            cap.release()
            cv2.destroyAllWindows()
            raise
        print(f"[CAL] CSV：{calibration_path}")
        print("[CAL] 先按 a/c/d/b 选择点位；t=真实靶纸，f=异物，n=暂停记录。")
    serial_link = DuplexSerial(args.port, args.baudrate, args.dry_run) if serial_enabled else None
    vision_fsm = VisionStateMachine(args.lock_frames, args.lock_threshold, args.jitter_limit)
    stabilizer = DetectionStabilizer(
        spatial_gate_px=args.spatial_gate_px,
        min_score=args.min_score,
        alpha=args.alpha,
        confirm_frames=args.confirm_frames,
        max_area_ratio=args.max_area_ratio,
    )
    if args.auto_start:
        vision_fsm.start(args.auto_start)
        print(f"[FSM] AUTO START point={args.auto_start}")

    fps = 0.0
    last_time = time.perf_counter()
    next_send_time = last_time

    try:
        while True:
            if serial_link is not None:
                try:
                    incoming_lines = serial_link.read_lines()
                except (OSError, UnicodeError, ValueError) as error:
                    incoming_lines = []
                    vision_fsm.stop()
                    stabilizer.reset()
                    print(f"[SERIAL] 接收错误，已回到 WAIT：{error}")
                for line in incoming_lines:
                    try:
                        command = decode_control(line)
                    except ValueError as error:
                        print(f"[SERIAL] 忽略非法命令：{error}")
                        continue
                    if command.action == "START" and command.point is not None:
                        vision_fsm.start(command.point)
                        stabilizer.reset()
                        if recorder is not None:
                            recorder.set_label(None)
                        print(f"[FSM] RX START point={command.point}")
                    elif command.action == "STOP":
                        vision_fsm.stop()
                        stabilizer.reset()
                        if recorder is not None:
                            recorder.set_label(None)
                        print("[FSM] RX STOP -> WAIT")

            ok, frame = cap.read()
            now = time.perf_counter()
            dt = now - last_time
            last_time = now
            if dt > 0:
                fps = (0.9 * fps + 0.1 / dt) if fps else 1.0 / dt
            if not ok:
                print("读取画面失败")
                break

            result = frame.copy()
            h, w = result.shape[:2]

            # ---- 检测 + 稳定性过滤 ----
            raw_detection = detect_a4_rectangle_with_score(frame, args.min_score)
            hold_limit = cv2.getTrackbarPos("Hold frames", CONTROLS)
            stable_corners, stable_center, stable_status = stabilizer.update(
                raw_detection, hold_limit
            )
            # -------------------------------------------------

            # 原始检测中心（调试用）
            raw_center = None
            raw_score = 0.0
            raw_area = 0.0
            raw_ratio = 0.0
            if raw_detection is not None:
                raw_corners = raw_detection[0]
                raw_center = raw_detection[1]
                raw_score = raw_detection[2]
                raw_area, raw_ratio = quadrilateral_metrics(raw_corners)
            if recorder is not None:
                recorder.record(vision_fsm.point, raw_detection, stable_status)

            # 确定瞄准点
            compensation_corners = stable_corners if stable_status not in ("LOST",) else None
            aim_point, physical_offset_px = compensated_aim_point(
                w, h, compensation_corners,
                args.laser_offset_x_mm, args.laser_offset_y_mm,
                args.target_width_mm, args.target_height_mm,
                args.aim_offset_x, args.aim_offset_y,
            )
            cross(result, aim_point, (0, 255, 255))  # 黄色瞄准点

            # ---- 可视化 ----
            ex, ey = 0, 0

            # 状态颜色（简化：TRUSTED/HOLD/LOST/INIT）
            status_colors = {
                "TRUSTED": (0, 255, 0),
                "HOLD":    (0, 200, 200),
                "ACQUIRING": (255, 180, 0),
                "LOST":    (0, 0, 255),
                "INIT":    (180, 180, 180),
            }
            status_color = status_colors.get(stable_status, (128, 128, 128))

            if stable_status not in ("LOST", "INIT") and stable_center is not None:
                sc = (int(round(stable_center[0])), int(round(stable_center[1])))

                # 空间闸门虚线圆（灰）
                gate_radius = args.spatial_gate_px
                cv2.circle(result, sc, gate_radius, (80, 80, 80), 1, cv2.LINE_AA)

                # 检测框 + 稳定中心十字
                if stable_corners is not None:
                    cv2.polylines(result, [stable_corners.astype(np.int32)], True, status_color, 2)
                cross(result, sc, status_color, 14)
                cv2.line(result, aim_point, sc, status_color, 1)

                # 原始检测中心（灰色小十字）
                if raw_center is not None:
                    raw_ci = (int(round(raw_center[0])), int(round(raw_center[1])))
                    cross(result, raw_ci, (140, 140, 140), 6)
                    cv2.line(result, sc, raw_ci, (140, 140, 140), 1)

                ex = sc[0] - aim_point[0]
                ey = sc[1] - aim_point[1]

            # 信息叠加
            cv2.putText(result, f"A4: {stable_status}", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)
            cv2.putText(result, f"error_x: {ex:+d} px", (15, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(result, f"error_y: {ey:+d} px", (15, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(result,
                        f"score: {raw_score:.0f}  gate: {args.spatial_gate_px}px  "
                        f"confirm: {stabilizer.pending_count}/{args.confirm_frames}  "
                        f"jump: {stabilizer._jump_count}",
                        (15, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # ---- 串口发送 ----
            # 只有当前帧通过所有检查时才发送有效误差。HOLD 只是显示旧框，
            # ACQUIRING 仍在确认新框，两者都不能驱动云台或累计 LOCKED。
            current_frame_trusted = stable_status == "TRUSTED" and stable_center is not None
            valid, stable = vision_fsm.update(current_frame_trusted, ex, ey)
            send_ex, send_ey = (ex, ey) if valid else (0, 0)
            if serial_link is not None and now >= next_send_time:
                serial_link.write(encode_aim_status(valid, stable, send_ex, send_ey))
                next_send_time = now + 1.0 / args.send_rate

            # FSM 状态显示
            point_text = vision_fsm.point or "-"
            fsm_colors = {
                VisionState.WAIT: (180, 180, 180),
                VisionState.SEARCH: (0, 0, 255),
                VisionState.TRACK: (0, 165, 255),
                VisionState.LOCKED: (0, 255, 0),
            }
            cv2.putText(
                result,
                f"VISION: {vision_fsm.state.value} point={point_text} "
                f"lock={vision_fsm.lock_count}/{args.lock_frames}",
                (15, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                fsm_colors[vision_fsm.state], 2,
            )
            cv2.putText(
                result,
                f"aim comp: total=({physical_offset_px[0] + args.aim_offset_x:+.1f},"
                f"{physical_offset_px[1] + args.aim_offset_y:+.1f}) px  "
                f"res=({args.aim_offset_x:+d},{args.aim_offset_y:+d})",
                (15, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
            )
            cv2.putText(result, f"FPS: {fps:.1f}", (w - 145, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
            cv2.putText(result,
                        "yellow: aim | green/orange: stable | grey: raw | circle: gate",
                        (15, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            if recorder is not None:
                if raw_detection is not None:
                    cv2.polylines(
                        result,
                        [raw_detection[0].astype(np.int32)],
                        True,
                        (180, 180, 180),
                        1,
                    )
                cal_label = recorder.label.upper() if recorder.label else "PAUSED"
                cal_color = {
                    "TARGET": (0, 255, 0),
                    "FOREIGN": (0, 0, 255),
                    "PAUSED": (160, 160, 160),
                }[cal_label]
                cv2.putText(
                    result,
                    f"CAL: {cal_label}  samples={recorder.sample_count}  [t/f/n]",
                    (15, 215),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    cal_color,
                    2,
                )
                cv2.putText(
                    result,
                    f"raw area={raw_area:.0f}px2  ratio={raw_ratio:.3f}",
                    (15, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (180, 180, 180),
                    1,
                )
            cv2.imshow(VIEW, result)

            # 键盘
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                reset_controls()
                stabilizer.reset()
            if key in (ord("a"), ord("b"), ord("c"), ord("d")):
                point = chr(key).upper()
                vision_fsm.start(point)
                stabilizer.reset()
                if recorder is not None:
                    recorder.set_label(None)
                print(f"[FSM] KEY START point={point}")
            if key == ord("x"):
                vision_fsm.stop()
                stabilizer.reset()
                if recorder is not None:
                    recorder.set_label(None)
                print("[FSM] KEY STOP -> WAIT")
            if recorder is not None and key in (ord("t"), ord("f"), ord("n")):
                if key in (ord("t"), ord("f")) and vision_fsm.point is None:
                    print("[CAL] 请先按 a/c/d/b 选择当前点位。")
                else:
                    label = {
                        ord("t"): "target",
                        ord("f"): "foreign",
                        ord("n"): None,
                    }[key]
                    recorder.set_label(label)
                    print(f"[CAL] 当前记录状态：{label or 'paused'}")
            if key == ord("s"):
                capture_dir.mkdir(exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(str(capture_dir / f"fsm_stable_raw_{stamp}.jpg"), frame)
                cv2.imwrite(str(capture_dir / f"fsm_stable_result_{stamp}.jpg"), result)
                print(f"已保存到 {capture_dir}")
    finally:
        if recorder is not None:
            summary_path = recorder.close()
            print(f"[CAL] CSV已保存：{recorder.path}")
            if summary_path is not None:
                print(f"[CAL] 建议范围已保存：{summary_path}")
        if serial_link is not None:
            try:
                serial_link.write(encode_aim_status(False, False, 0, 0))
            except OSError:
                pass
            serial_link.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
