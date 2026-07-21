#!/usr/bin/env python3
"""将下载方案 k230/main_360.py 的“最大黑色 Blob”思路移植到 USB 摄像头。

这是对照测试版：黑色二值化 -> 形态学闭合 -> 最大 A4 比例连通块 -> 外接框中心。
它不依赖 K230/OpenMV，只需要 Ubuntu 的 Python3 + OpenCV。

运行：
  python3 source_style_target.py --camera 4
  python3 source_style_target.py --camera 4 --width 1280 --height 720
"""

from __future__ import annotations

import argparse
import time
from typing import Optional, Tuple

import cv2
import numpy as np


VIEW = "Source-style black blob"
MASK_VIEW = "Source-style black mask"
CONTROLS = "Controls"


def nothing(_: int) -> None:
    pass


def create_controls() -> None:
    cv2.namedWindow(CONTROLS, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROLS, 430, 150)
    cv2.createTrackbar("Black max", CONTROLS, 85, 255, nothing)
    cv2.createTrackbar("Min pixels", CONTROLS, 300, 10000, nothing)
    cv2.createTrackbar("Close size", CONTROLS, 5, 21, nothing)


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
        raise RuntimeError("摄像头无法读取画面")
    return cap


def find_largest_a4_blob(frame: np.ndarray) -> Optional[Tuple[int, int, int, int, int]]:
    """对应原方案的 find_blobs + max(pixels)，增加 A4 比例和边界过滤。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    black_max = cv2.getTrackbarPos("Black max", CONTROLS)
    _, mask = cv2.threshold(gray, black_max, 255, cv2.THRESH_BINARY_INV)

    close_size = max(1, cv2.getTrackbarPos("Close size", CONTROLS))
    if close_size % 2 == 0:
        close_size += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (close_size, close_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    cv2.imshow(MASK_VIEW, mask)

    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    min_pixels = cv2.getTrackbarPos("Min pixels", CONTROLS)
    image_h, image_w = mask.shape
    candidates = []
    for label in range(1, count):  # 0 是黑色背景
        x, y, width, height, pixels = stats[label]
        if pixels < min_pixels or width < 20 or height < 20:
            continue
        # 完整靶纸不会贴着画面边缘；排除桌沿、窗口边框等背景。
        if x <= 2 or y <= 2 or x + width >= image_w - 2 or y + height >= image_h - 2:
            continue
        ratio = min(width, height) / max(width, height)
        # A4 短边/长边为 0.707；保留透视导致的合理变化。
        if not 0.45 <= ratio <= 0.95:
            continue
        candidates.append((pixels, x, y, width, height))

    if not candidates:
        return None
    pixels, x, y, width, height = max(candidates, key=lambda item: item[0])
    return int(x), int(y), int(width), int(height), int(pixels)


def main() -> None:
    parser = argparse.ArgumentParser(description="移植 K230 方案的黑色 Blob 靶框测试")
    parser.add_argument("--camera", type=int, default=4)
    # K230 原方案使用 VGA；默认此分辨率，先测试最高稳定帧率。
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    cap = open_camera(args.camera, args.width, args.height)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"K230 思路移植版：/dev/video{args.camera} {actual_w}x{actual_h}；q 或 ESC 退出。")
    create_controls()
    cv2.namedWindow(VIEW, cv2.WINDOW_NORMAL)
    cv2.namedWindow(MASK_VIEW, cv2.WINDOW_NORMAL)
    fps = 0.0
    previous = time.perf_counter()

    try:
        while True:
            ok, frame = cap.read()
            now = time.perf_counter()
            elapsed = now - previous
            previous = now
            if elapsed > 0:
                fps = (0.9 * fps + 0.1 / elapsed) if fps else 1.0 / elapsed
            if not ok:
                break

            result = frame.copy()
            h, w = result.shape[:2]
            center = (w // 2, h // 2)
            cv2.drawMarker(result, center, (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
            blob = find_largest_a4_blob(frame)
            if blob is None:
                cv2.putText(result, "black blob: NOT FOUND", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            else:
                x, y, width, height, pixels = blob
                target = (x + width // 2, y + height // 2)
                cv2.rectangle(result, (x, y), (x + width, y + height), (255, 0, 255), 2)
                cv2.drawMarker(result, target, (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
                cv2.line(result, center, target, (0, 255, 0), 1)
                cv2.putText(result, "black blob: FOUND", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                cv2.putText(result, f"error_x: {target[0] - center[0]:+d} px", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(result, f"error_y: {target[1] - center[1]:+d} px", (15, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(result, f"pixels: {pixels}", (15, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(result, f"FPS: {fps:.1f}", (w - 130, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow(VIEW, result)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
