#!/usr/bin/env python3
"""非阻塞双向串口；真实串口和 dry-run 使用同一发送接口。"""

from __future__ import annotations

import os
import select

from serial_port import open_serial


class DuplexSerial:
    def __init__(self, port: str | None, baudrate: int, dry_run: bool) -> None:
        if not dry_run and not port:
            raise ValueError("真实通信必须指定串口端口")
        self.dry_run = dry_run
        self.fd = None if dry_run else open_serial(port, baudrate)
        self._rx_buffer = bytearray()

    def write(self, packet: bytes) -> None:
        if self.dry_run:
            print(f"[SERIAL] {packet.decode('ascii').strip()}")
        elif self.fd is not None:
            os.write(self.fd, packet)

    def read_lines(self) -> list[str]:
        """读取当前已经到达的完整行；没有数据时立即返回，不阻塞视频循环。"""
        if self.fd is None:
            return []

        while True:
            readable, _, _ = select.select([self.fd], [], [], 0.0)
            if not readable:
                break
            chunk = os.read(self.fd, 4096)
            if not chunk:
                break
            self._rx_buffer.extend(chunk)
            if len(self._rx_buffer) > 4096:
                self._rx_buffer.clear()
                raise ValueError("串口接收缓存溢出，已清空")

        lines: list[str] = []
        while b"\n" in self._rx_buffer:
            raw_line, _, remainder = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(remainder)
            raw_line = raw_line.rstrip(b"\r")
            if raw_line:
                lines.append(raw_line.decode("ascii", errors="strict"))
        return lines

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
