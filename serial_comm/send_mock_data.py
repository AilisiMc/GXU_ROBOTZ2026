#!/usr/bin/env python3
"""向终端或真实串口连续发送模拟视觉误差。

无单片机时：python3 send_mock_data.py --dry-run
STM32 到手后：python3 send_mock_data.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import math
import time

from protocol import encode_aim
from serial_port import SerialOutput


def mock_sample(frame_index: int) -> tuple[bool, int, int]:
    """循环模拟搜索、捕获和误差逐渐收敛。"""
    phase = frame_index % 120
    if phase < 15:
        return False, 0, 0
    tracking_frame = phase - 15
    decay = math.exp(-tracking_frame / 18.0)
    error_x = round(180 * decay * math.cos(tracking_frame * 0.18))
    error_y = round(-110 * decay * math.cos(tracking_frame * 0.15))
    return True, error_x, error_y


def main() -> None:
    parser = argparse.ArgumentParser(description="连续发送模拟 AIM 串口数据")
    parser.add_argument("--port", help="例如 /dev/ttyACM0 或 /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=20.0, help="发送频率，默认 20 Hz")
    parser.add_argument("--dry-run", action="store_true", help="只打印，不打开串口")
    args = parser.parse_args()

    if args.rate <= 0:
        raise ValueError("rate 必须大于 0")
    if not args.dry_run and not args.port:
        parser.error("请提供 --port，或使用 --dry-run")

    output = SerialOutput(args.port, args.baudrate, args.dry_run)
    print(
        f"开始以 {args.rate:g} Hz 发送，波特率 {args.baudrate}；"
        "按 Ctrl+C 停止。"
    )

    frame_index = 0
    try:
        while True:
            valid, error_x, error_y = mock_sample(frame_index)
            packet = encode_aim(valid, error_x, error_y)
            output.write(packet)
            frame_index += 1
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print("\n已停止发送。")
    finally:
        output.close()


if __name__ == "__main__":
    main()
