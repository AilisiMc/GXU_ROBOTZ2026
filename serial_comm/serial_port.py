#!/usr/bin/env python3
"""Linux 串口写入工具，仅使用 Python 标准库。"""

from __future__ import annotations

import os
import termios


def open_serial(port: str, baudrate: int = 115200) -> int:
    baud_constants = {
        9600: termios.B9600,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    if baudrate not in baud_constants:
        raise ValueError("支持的波特率：9600、57600、115200")

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = baud_constants[baudrate]
    attrs[5] = baud_constants[baudrate]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


class SerialOutput:
    """统一真实串口与只打印模式。"""

    def __init__(self, port: str | None, baudrate: int, dry_run: bool) -> None:
        if not dry_run and not port:
            raise ValueError("真实发送必须指定串口端口")
        self.dry_run = dry_run
        self.fd = None if dry_run else open_serial(port, baudrate)

    def write(self, packet: bytes) -> None:
        if self.dry_run:
            print(f"[SERIAL] {packet.decode('ascii').strip()}")
        else:
            os.write(self.fd, packet)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

