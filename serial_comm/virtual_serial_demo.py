#!/usr/bin/env python3
"""无需 STM32 的虚拟串口收发演示。

程序在 Linux 中创建一对伪终端：笔记本端发送 AIM，STM32 模拟端解析并回复状态。
"""

from __future__ import annotations

import os
import pty
import select
import threading
import time
import tty

from protocol import decode_aim, encode_aim


def stm32_simulator(master_fd: int, stop_event: threading.Event) -> None:
    """模拟 STM32：逐行接收误差，并在满足条件时回复 LOCKED。"""
    buffer = bytearray()
    while not stop_event.is_set():
        ready, _, _ = select.select([master_fd], [], [], 0.1)
        if not ready:
            continue
        data = os.read(master_fd, 256)
        if not data:
            continue
        buffer.extend(data)
        while b"\n" in buffer:
            raw_line, _, remainder = buffer.partition(b"\n")
            buffer = bytearray(remainder)
            try:
                message = decode_aim(raw_line)
            except ValueError as exc:
                print(f"[STM32模拟端] 丢弃错误数据：{exc}")
                continue

            if not message.valid:
                print("[STM32模拟端] AIM valid=0：目标丢失，云台保持")
                os.write(master_fd, b"STATE,SEARCHING\n")
                continue

            locked = abs(message.error_x) <= 8 and abs(message.error_y) <= 8
            state = "LOCKED" if locked else "TRACKING"
            print(
                f"[STM32模拟端] ex={message.error_x:+4d}, "
                f"ey={message.error_y:+4d} -> {state}"
            )
            os.write(master_fd, f"STATE,{state}\n".encode("ascii"))


def read_reply(slave_fd: int) -> str:
    ready, _, _ = select.select([slave_fd], [], [], 0.2)
    if not ready:
        return "无回复"
    return os.read(slave_fd, 256).decode("ascii", errors="replace").strip()


def main() -> None:
    master_fd, slave_fd = pty.openpty()
    tty.setraw(master_fd)
    tty.setraw(slave_fd)
    print(f"已创建虚拟串口：{os.ttyname(slave_fd)}")
    print("协议：AIM,valid,error_x,error_y\\n\n")

    stop_event = threading.Event()
    receiver = threading.Thread(
        target=stm32_simulator, args=(master_fd, stop_event), daemon=True
    )
    receiver.start()

    # 模拟：先丢失目标，再逐步接近中心，最后再次丢失。
    samples = [
        (False, 0, 0),
        (True, 160, -95),
        (True, 100, -60),
        (True, 55, -32),
        (True, 25, -16),
        (True, 12, -9),
        (True, 7, -5),
        (True, 3, -2),
        (False, 0, 0),
    ]

    try:
        for valid, error_x, error_y in samples:
            packet = encode_aim(valid, error_x, error_y)
            os.write(slave_fd, packet)
            print(f"[笔记本视觉端] 发送 {packet.decode().strip()}")
            print(f"[笔记本视觉端] 收到 {read_reply(slave_fd)}\n")
            time.sleep(0.25)
    finally:
        stop_event.set()
        receiver.join(timeout=1.0)
        os.close(slave_fd)
        os.close(master_fd)


if __name__ == "__main__":
    main()

