#!/usr/bin/env python3
"""通过伪终端模拟 STM32 主状态机，与真实摄像头视觉程序联调。"""

from __future__ import annotations

import argparse
import os
import pty
import select
import sys
import time
import tty

from aiming_protocol import decode_aim_status, encode_start, encode_stop


def main() -> None:
    parser = argparse.ArgumentParser(description="STM32 打靶主状态机模拟器")
    parser.add_argument("--watchdog-ms", type=int, default=300)
    parser.add_argument("--confirm-packets", type=int, default=3)
    args = parser.parse_args()
    if args.watchdog_ms <= 0 or args.confirm_packets <= 0:
        parser.error("watchdog-ms 和 confirm-packets 必须大于 0")

    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    slave_path = os.ttyname(slave_fd)

    print(f"虚拟STM32串口：{slave_path}")
    print("另开终端运行：")
    print(
        f"  python3 vision/aiming_state_machine.py --camera 4 --port {slave_path}"
    )
    print("本终端命令：a/b/c/d + 回车开始对应点；s + 回车停止；q + 回车退出。")

    active_point: str | None = None
    control_state = "IDLE"
    stable_packets = 0
    last_vision_time: float | None = None
    last_print_time = 0.0
    timed_out = False
    rx_buffer = bytearray()

    try:
        while True:
            readable, _, _ = select.select([master_fd, sys.stdin], [], [], 0.05)
            now = time.monotonic()

            if sys.stdin in readable:
                command = sys.stdin.readline().strip().lower()
                if command in {"a", "b", "c", "d"}:
                    active_point = command.upper()
                    control_state = "AIMING"
                    stable_packets = 0
                    last_vision_time = now
                    timed_out = False
                    os.write(master_fd, encode_start(active_point))
                    print(
                        f"[STM32] DRIVE→PRESET→AIMING，发送 START,{active_point} "
                        "（模拟车辆已停稳、预置角已完成）"
                    )
                elif command in {"s", "stop"}:
                    os.write(master_fd, encode_stop())
                    active_point = None
                    control_state = "IDLE"
                    stable_packets = 0
                    print("[STM32] 发送 STOP，激光关闭，状态回到 IDLE")
                elif command in {"q", "quit", "exit"}:
                    break
                elif command:
                    print("请输入 a/b/c/d、s 或 q")

            if master_fd in readable:
                chunk = os.read(master_fd, 4096)
                if chunk:
                    rx_buffer.extend(chunk)
                if len(rx_buffer) > 4096:
                    rx_buffer.clear()
                    print("[STM32] 接收缓存溢出，已清空并关闭激光")
                    control_state = "SAFE"
                    stable_packets = 0

            while b"\n" in rx_buffer:
                raw_line, _, remainder = rx_buffer.partition(b"\n")
                rx_buffer = bytearray(remainder)
                raw_line = raw_line.rstrip(b"\r")
                if not raw_line:
                    continue
                try:
                    aim = decode_aim_status(raw_line)
                except (UnicodeError, ValueError) as error:
                    print(f"[STM32] 丢弃非法视觉包：{error}")
                    continue

                last_vision_time = now
                timed_out = False
                if active_point is None:
                    continue

                if not aim.valid:
                    next_state = "SEARCHING"
                    stable_packets = 0
                elif not aim.stable:
                    next_state = "TRACKING"
                    stable_packets = 0
                else:
                    stable_packets += 1
                    next_state = "LOCK_CONFIRM"

                if next_state != control_state:
                    control_state = next_state
                    print(f"[STM32] state={control_state}")

                if now - last_print_time >= 0.25 and aim.valid:
                    print(
                        f"[STM32] dx={aim.error_x:+d}, dy={aim.error_y:+d}, "
                        f"visual_stable={int(aim.stable)}"
                    )
                    last_print_time = now

                if stable_packets >= args.confirm_packets:
                    # 真实STM32还要同时检查：到点、停车、预置完成、激光许可。
                    print(
                        f"[STM32] FIRE point={active_point}：连续{stable_packets}个"
                        "新鲜稳定包，模拟开激光后立即关闭"
                    )
                    os.write(master_fd, encode_stop())
                    active_point = None
                    control_state = "IDLE"
                    stable_packets = 0

            if (
                active_point is not None
                and last_vision_time is not None
                and (now - last_vision_time) * 1000.0 > args.watchdog_ms
                and not timed_out
            ):
                timed_out = True
                control_state = "SAFE_TIMEOUT"
                stable_packets = 0
                print(
                    f"[STM32] SAFE_TIMEOUT：超过{args.watchdog_ms} ms无视觉包，"
                    "停止云台并关闭激光"
                )
    finally:
        os.close(master_fd)
        os.close(slave_fd)


if __name__ == "__main__":
    main()
