#!/usr/bin/env python3


from __future__ import annotations

import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "serial_comm"))

from protocol import encode_aim
from serial_port import SerialOutput


VIEW = "A4 rectangle detection"
MASK_VIEW = "Black mask"
PAPER_MASK_VIEW = "White paper mask"
CONTROLS = "Controls"
A4_SHORT_LONG_RATIO = 210.0 / 297.0
# 黑胶带围住以后，内部白色区域的理论比例约为
# (21 - 2*1.8) : (29.7 - 2*1.8) = 17.4 : 26.1。
A4_INNER_SHORT_LONG_RATIO = 174.0 / 261.0
DEFAULTS = {
    "Black max": 85,
    # USB 摄像头自动白平衡后，白纸在部分角度会明显偏蓝，S 可达到约 100。
    "White S max": 110,
    "White V min": 105,
    "Min area": 700,
    "Close size": 5,
    "Hold frames": 5,
}


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
    # 单帧漏检时暂时使用上一次可靠结果；设上限，防止靶纸离开后仍输出陈旧坐标。
    cv2.createTrackbar("Hold frames", CONTROLS, DEFAULTS["Hold frames"], 30, _nothing)


def reset_controls() -> None:
    for name, value in DEFAULTS.items():
        cv2.setTrackbarPos(name, CONTROLS, value)


def open_camera(index: int, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开 /dev/video{index}")

    # MJPEG 在 720p 下通常比未压缩 YUYV 占用更少 USB 带宽；若设备不支持，
    # V4L2 会自动协商为摄像头可用格式。
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 降低画面延迟，不堆积旧帧
    ok, _ = cap.read()
    if not ok:
        cap.release()
        raise RuntimeError("摄像头已打开但无法读取画面")
    return cap


def order_corners(points: np.ndarray) -> np.ndarray:
    """把四角按轮廓顺序排列，并从最靠左上的角开始。

    旧的坐标和/差方法在强透视梯形下可能把同一个点同时分给两个角。按质心
    极角排序保证四个点唯一且沿边界连续，对旋转矩形和斜视梯形都适用。
    """
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


def line_intersection(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> Optional[np.ndarray]:
    """两条直线 ab 与 cd 的交点；即透视投影下矩形对角线的交点。"""
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


def target_candidate(
    contour: np.ndarray,
    min_area: float,
    white_mask: np.ndarray,
    black_mask: np.ndarray,
) -> Optional[tuple[np.ndarray, np.ndarray, float]]:
    """验证“白色纸面 + 四周黑胶带”，并返回外框角点、中心和评分。

    旧逻辑会给任意大轮廓套 ``minAreaRect``，纸板边缘、阴影或三条线也可能
    被当作靶框。这里先寻找白色 A4 内部，再检查它外面四条边是否真的有黑带。
    """
    area = cv2.contourArea(contour)
    if area < min_area:
        return None

    frame_h, frame_w = white_mask.shape[:2]
    # 首次捕获要求白色纸面完整可见；短暂缺失由主循环的 HOLD 处理。
    x, y, width, height = cv2.boundingRect(contour)
    margin = 3
    if x <= margin or y <= margin or x + width >= frame_w - margin or y + height >= frame_h - margin:
        return None

    perimeter = cv2.arcLength(contour, True)
    # 纸张褶皱和胶带遮挡可能多产生一个很小的折点；3% 周长近似仍保留
    # A4 的四个主角点，同时不会放宽后面的白心、黑边验证。
    polygon = cv2.approxPolyDP(contour, 0.030 * perimeter, True)
    if len(polygon) == 4 and cv2.isContourConvex(polygon):
        corners = order_corners(polygon)
    else:
        # 只允许“本身已经很像填满矩形”的白纸轮廓进入兜底分支。
        # 这与旧版对所有黑色轮廓无条件套外接矩形不同。
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

    # 内部白纸区域理论比例约 0.667；保留透视余量，但排除细长条和正方形。
    # 强斜视会把 A4 长边压缩到看起来接近正方形，投影边长比不能设 0.95
    # 的硬上限；黑边和白色内部验证负责排除普通背景矩形。
    if not 0.32 <= observed_ratio <= 1.0:
        return None

    center = line_intersection(corners[0], corners[2], corners[1], corners[3])
    if center is None:
        return None

    # 把白纸内部透视展开，确认它不是由几个零散亮块拼成的矩形。
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

    # 由内部白纸向外扩约 18%，覆盖实际 1.8 cm 黑胶带区域。
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

    # 反光可能让一条黑胶带变浅，因此允许一条边较弱；但不能让真正缺边的
    # U 形轮廓或任意背景矩形仅凭外接矩形通过首次确认。
    if np.count_nonzero(side_black >= 0.35) < 3:
        return None
    if np.min(side_black) < 0.18 or np.mean(side_black) < 0.43:
        return None

    ratio_error = abs(math.log(observed_ratio / A4_INNER_SHORT_LONG_RATIO))
    border_score = float(np.mean(side_black))
    # 面积只做温和加权，黑边完整度和白纸纯度才是主要依据。
    score = math.sqrt(area) * border_score**2 * inner_white_ratio * math.exp(-2.0 * ratio_error)
    return outer_corners, center, score


def black_ring_candidate(
    contours: list[np.ndarray],
    hierarchy: np.ndarray,
    index: int,
    min_area: float,
    gray: np.ndarray,
    black_max: int,
) -> Optional[tuple[np.ndarray, np.ndarray, float]]:
    """从闭合黑胶带环内部的四边形孔洞直接恢复靶心。

    在二值图里，完整黑胶带是白色环，其内部纸面会成为带 parent 的孔洞轮廓。
    这条路径不依赖纸面颜色，也不需要用等比例放大去猜透视后的胶带位置。
    """
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

    perimeter = cv2.arcLength(inner, True)
    polygon = cv2.approxPolyDP(inner, 0.025 * perimeter, True)
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
    # 斜视时投影边长比会显著改变，A4 甚至可能投影成接近正方形；这里只
    # 排除近似细线。循迹框由黑环相对厚度、亮内部和打靶阶段状态共同排除。
    if not 0.30 <= observed_ratio <= 1.0:
        return None

    parent_area = cv2.contourArea(contours[parent_index])
    ring_ratio = parent_area / inner_area if inner_area > 0 else 0.0
    # 理论黑框外/内面积比约 1.38；胶带反光或与少量背景粘连时允许更大余量。
    if not 1.08 <= ring_ratio <= 4.0:
        return None

    center = line_intersection(corners[0], corners[2], corners[1], corners[3])
    if center is None:
        return None

    # 孔洞内部应当明显亮于黑色阈值，排除黑色物体内部的暗孔。
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

    # 外扩仅用于画绿色框，控制误差只使用透视保持正确的内框对角线交点。
    outer_corners = center + 1.16 * (corners - center)
    ratio_error = abs(math.log(observed_ratio / A4_INNER_SHORT_LONG_RATIO))
    score = math.sqrt(inner_area) * math.exp(-0.8 * ratio_error)
    return outer_corners, center, score


def detect_a4_rectangle(frame: np.ndarray) -> Optional[tuple[np.ndarray, np.ndarray]]:
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

    # 主路径：完整黑框在轮廓树中表现为“外环 + 内部四边形孔洞”。
    black_contours, black_hierarchy = cv2.findContours(
        black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    black_best: Optional[tuple[np.ndarray, np.ndarray, float]] = None
    if black_hierarchy is not None:
        hierarchy_rows = black_hierarchy[0]
        for index in range(len(black_contours)):
            candidate = black_ring_candidate(
                black_contours,
                hierarchy_rows,
                index,
                max(1, cv2.getTrackbarPos("Min area", CONTROLS)),
                gray,
                black_max,
            )
            if candidate is not None and (black_best is None or candidate[2] > black_best[2]):
                black_best = candidate

    white_close_size = max(5, close_size)
    white_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (white_close_size, white_close_size))
    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    # 同时使用严格白纸阈值和允许自动白平衡偏色的宽松阈值：近距离反光时
    # 严格阈值更干净，A/B 远视角下纸面偏蓝时宽松阈值仍能找到纸张。
    saturation_limits = sorted({min(70, white_s_max), white_s_max})
    white_masks = []
    for saturation_max in saturation_limits:
        current_white = cv2.inRange(
            hsv,
            np.array([0, 0, white_v_min], dtype=np.uint8),
            np.array([179, saturation_max, 255], dtype=np.uint8),
        )
        current_white = cv2.morphologyEx(current_white, cv2.MORPH_OPEN, open_kernel)
        current_white = cv2.morphologyEx(current_white, cv2.MORPH_CLOSE, white_kernel)
        white_masks.append(current_white)

    min_area = max(1, cv2.getTrackbarPos("Min area", CONTROLS))
    best: Optional[tuple[np.ndarray, np.ndarray, float]] = None
    for white_mask in white_masks:
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            candidate = target_candidate(contour, min_area, white_mask, black_mask)
            if candidate is not None and (best is None or candidate[2] > best[2]):
                best = candidate

    cv2.imshow(MASK_VIEW, black_mask)
    cv2.imshow(PAPER_MASK_VIEW, white_masks[-1])
    if black_best is not None:
        return black_best[0], black_best[1]
    if best is None:
        return None
    return best[0], best[1]


def cross(image: np.ndarray, point: tuple[int, int], color: tuple[int, int, int], size: int = 14) -> None:
    x, y = point
    cv2.line(image, (x - size, y), (x + size, y), color, 2)
    cv2.line(image, (x, y - size), (x, y + size), color, 2)


def main() -> None:
    parser = argparse.ArgumentParser(description="A4 黑色矩形靶框定位")
    parser.add_argument("--camera", type=int, default=4, help="例如 4 对应 /dev/video4")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--port", help="真实串口，例如 /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true", help="不打开串口，只打印 AIM 数据")
    parser.add_argument("--send-rate", type=float, default=20.0, help="串口发送频率，默认 20 Hz")
    args = parser.parse_args()

    if args.send_rate <= 0:
        parser.error("--send-rate 必须大于 0")
    serial_enabled = args.dry_run or args.port is not None

    cap = open_camera(args.camera, args.width, args.height)
    actual = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    codec = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))
    print(f"/dev/video{args.camera}: {actual[0]}x{actual[1]}, codec={codec}; q 退出，s 保存，r 重置。")

    create_controls()
    cv2.namedWindow(VIEW, cv2.WINDOW_NORMAL)
    cv2.namedWindow(MASK_VIEW, cv2.WINDOW_NORMAL)
    capture_dir = Path(__file__).parent / "captures"
    serial_output = SerialOutput(args.port, args.baudrate, args.dry_run) if serial_enabled else None
    fps = 0.0
    last_time = time.perf_counter()
    next_send_time = last_time
    last_corners: Optional[np.ndarray] = None
    last_center: Optional[np.ndarray] = None
    missed_frames = 0

    try:
        while True:
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
            image_center = (w // 2, h // 2)
            cross(result, image_center, (0, 255, 255))  # 黄色：相机主点附近

            detected = detect_a4_rectangle(frame)
            ex, ey = 0, 0
            if detected is not None:
                corners, center = detected
                # 轻度指数平滑，降低轮廓在相邻帧间跳动造成的舵机抖动。
                if last_corners is not None and last_center is not None:
                    corners = 0.65 * corners + 0.35 * last_corners
                    center = 0.65 * center + 0.35 * last_center
                last_corners, last_center = corners.copy(), center.copy()
                missed_frames = 0
                state = "FOUND"
                label_color = (0, 255, 0)
            else:
                missed_frames += 1
                hold_limit = cv2.getTrackbarPos("Hold frames", CONTROLS)
                if last_corners is not None and last_center is not None and missed_frames <= hold_limit:
                    # 借鉴 recdetect2.py 的上一帧结果机制，但只保留有限帧数，避免陈旧坐标。
                    corners, center = last_corners, last_center
                    state = f"HOLD {missed_frames}/{hold_limit}"
                    label_color = (0, 165, 255)
                else:
                    state = "NOT FOUND"
                    label_color = (0, 0, 255)
                    last_corners, last_center = None, None

            if state != "NOT FOUND":
                center_int = (int(round(center[0])), int(round(center[1])))
                shape_color = (0, 255, 0) if state == "FOUND" else (0, 165, 255)
                cv2.polylines(result, [corners.astype(np.int32)], True, shape_color, 2)
                cross(result, center_int, shape_color)
                cv2.line(result, image_center, center_int, shape_color, 1)
                ex, ey = center_int[0] - image_center[0], center_int[1] - image_center[1]
                cv2.putText(result, f"error_x: {ex:+d} px", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(result, f"error_y: {ey:+d} px", (15, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # 只增加输出，不改变原有识别逻辑：当前帧 FOUND 才发送真实误差；
            # HOLD 或 NOT FOUND 均发送无效目标，让 STM32 停止使用旧误差。
            if serial_output is not None and now >= next_send_time:
                serial_output.write(encode_aim(state == "FOUND", ex, ey))
                next_send_time = now + 1.0 / args.send_rate

            cv2.putText(result, f"A4 frame: {state}", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, label_color, 2)
            cv2.putText(result, f"FPS: {fps:.1f}", (w - 145, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
            cv2.putText(result, "yellow: image center | green: A4 geometric center", (15, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.imshow(VIEW, result)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                reset_controls()
            if key == ord("s"):
                capture_dir.mkdir(exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(str(capture_dir / f"a4_raw_{stamp}.jpg"), frame)
                cv2.imwrite(str(capture_dir / f"a4_result_{stamp}.jpg"), result)
                print(f"已保存到 {capture_dir}")
    finally:
        if serial_output is not None:
            try:
                serial_output.write(encode_aim(False, 0, 0))
            except OSError:
                pass
            serial_output.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
