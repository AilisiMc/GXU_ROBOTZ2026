#!/usr/bin/env python3
"""红色靶心检测：实时画面、画面中心十字、靶心误差。

按键：
  q / ESC  退出
  s        保存当前原图和检测结果到当前目录的 captures/ 文件夹
  r        重置 HSV 滑条为默认值

用法：
  python3 target_test.py
  python3 target_test.py --camera 4
  python3 target_test.py --camera 4 --width 1280 --height 720
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np


WINDOW = "Target detection"
MASK_WINDOW = "Red mask"
CONTROLS = "Controls"
DEFAULTS = {
    "S min": 80,
    "V min": 70,
    "Min area": 120,
}


def open_camera(requested_index: Optional[int], width: int, height: int) -> Tuple[cv2.VideoCapture, int]:
    """打开指定摄像头；未指定时依次尝试 0 到 9。"""
    candidates = [requested_index] if requested_index is not None else list(range(10))
    for index in candidates:
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if cap.isOpened():
            # 比赛最远处约 1.6 m，必须使用摄像头支持的 720p 才能看清 2 cm 靶心。
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, 30)
            ok, _ = cap.read()
            if ok:
                return cap, index
        cap.release()
    raise RuntimeError("没有找到可用摄像头。可尝试：python3 target_test.py --camera 4")


def nothing(_: int) -> None:
    pass


def create_controls() -> None:
    cv2.namedWindow(CONTROLS, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROLS, 420, 140)
    cv2.createTrackbar("S min", CONTROLS, DEFAULTS["S min"], 255, nothing)
    cv2.createTrackbar("V min", CONTROLS, DEFAULTS["V min"], 255, nothing)
    cv2.createTrackbar("Min area", CONTROLS, DEFAULTS["Min area"], 5000, nothing)


def reset_controls() -> None:
    for name, value in DEFAULTS.items():
        cv2.setTrackbarPos(name, CONTROLS, value)


def get_red_mask(frame: np.ndarray) -> np.ndarray:
    """取 HSV 中红色的两个色相区间，排除低饱和度/低亮度的灰暗区域。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    s_min = cv2.getTrackbarPos("S min", CONTROLS)
    v_min = cv2.getTrackbarPos("V min", CONTROLS)

    # OpenCV 的 H 范围是 0~179；红色跨越 0，因此要分为两段。
    lower_red_1 = np.array([0, s_min, v_min])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([170, s_min, v_min])
    upper_red_2 = np.array([179, 255, 255])
    mask = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask |= cv2.inRange(hsv, lower_red_2, upper_red_2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_target(mask: np.ndarray) -> Optional[Tuple[int, int, float]]:
    """寻找实心、近圆形的红色靶心，忽略大而细的红色圆环。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = max(cv2.getTrackbarPos("Min area", CONTROLS), 1)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if width == 0 or height == 0:
            continue

        # 红色圆环虽然包围面积大，但它在外接矩形中只占很小的一部分；
        # 实心靶心的填充率通常较高，且宽高接近。
        fill_ratio = area / (width * height)
        aspect_ratio = width / height
        if fill_ratio >= 0.25 and 0.5 <= aspect_ratio <= 2.0:
            candidates.append(contour)
    if not candidates:
        return None

    # 若有多个实心红点，优先取面积最大的一个。
    contour = max(candidates, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None
    return int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]), area


def draw_cross(image: np.ndarray, x: int, y: int, color: Tuple[int, int, int], size: int = 15) -> None:
    cv2.line(image, (x - size, y), (x + size, y), color, 2)
    cv2.line(image, (x, y - size), (x, y + size), color, 2)


def main() -> None:
    parser = argparse.ArgumentParser(description="红色靶心实时检测")
    parser.add_argument("--camera", type=int, help="摄像头编号，例如 4 对应 /dev/video4")
    parser.add_argument("--width", type=int, default=1280, help="请求的画面宽度，默认 1280")
    parser.add_argument("--height", type=int, default=720, help="请求的画面高度，默认 720")
    args = parser.parse_args()

    cap, index = open_camera(args.camera, args.width, args.height)
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"已打开 /dev/video{index}，实际分辨率 {actual_width}x{actual_height}。按 q 退出，s 保存画面，r 重置参数。")
    create_controls()
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.namedWindow(MASK_WINDOW, cv2.WINDOW_NORMAL)
    captures_dir = Path(__file__).parent / "captures"

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读取摄像头画面失败。")
                break

            mask = get_red_mask(frame)
            result = frame.copy()
            height, width = result.shape[:2]
            image_cx, image_cy = width // 2, height // 2
            draw_cross(result, image_cx, image_cy, (255, 255, 0))  # 黄色：画面中心

            target = find_target(mask)
            if target is None:
                cv2.putText(result, "Target: NOT FOUND", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                tx, ty, area = target
                error_x = tx - image_cx
                error_y = ty - image_cy
                draw_cross(result, tx, ty, (0, 255, 0))  # 绿色：识别到的靶心
                cv2.line(result, (image_cx, image_cy), (tx, ty), (0, 255, 0), 1)
                locked = abs(error_x) <= 12 and abs(error_y) <= 12
                status = "LOCKED" if locked else "TRACKING"
                color = (0, 255, 0) if locked else (0, 165, 255)
                cv2.putText(result, f"Target: {status}", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.putText(result, f"error_x: {error_x:+d} px", (15, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
                cv2.putText(result, f"error_y: {error_y:+d} px", (15, 88),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
                cv2.putText(result, f"area: {area:.0f}", (15, 116),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            cv2.putText(result, "yellow: image center | green: target center", (15, height - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.imshow(WINDOW, result)
            cv2.imshow(MASK_WINDOW, mask)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                reset_controls()
            if key == ord("s"):
                captures_dir.mkdir(exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(str(captures_dir / f"raw_{stamp}.jpg"), frame)
                cv2.imwrite(str(captures_dir / f"result_{stamp}.jpg"), result)
                print(f"已保存到 {captures_dir}")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
