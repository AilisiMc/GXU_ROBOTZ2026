#!/usr/bin/env python3
"""视觉组与 STM32 之间的最小文本协议。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AimMessage:
    valid: bool
    error_x: int
    error_y: int


def encode_aim(valid: bool, error_x: int, error_y: int) -> bytes:
    """编码为：AIM,valid,error_x,error_y\n。"""
    if not valid:
        error_x, error_y = 0, 0
    return f"AIM,{int(valid)},{int(error_x)},{int(error_y)}\n".encode("ascii")


def decode_aim(line: bytes | bytearray | memoryview | str) -> AimMessage:
    """解析并校验一行 AIM 数据；格式错误时抛出 ValueError。"""
    if not isinstance(line, str):
        line = bytes(line).decode("ascii", errors="strict")
    parts = line.strip().split(",")
    if len(parts) != 4 or parts[0] != "AIM":
        raise ValueError(f"格式错误：{line!r}")
    valid_value, error_x, error_y = map(int, parts[1:])
    if valid_value not in (0, 1):
        raise ValueError("valid 只能为 0 或 1")
    if not -4096 <= error_x <= 4096 or not -4096 <= error_y <= 4096:
        raise ValueError("像素误差超出允许范围")
    if valid_value == 0 and (error_x != 0 or error_y != 0):
        raise ValueError("目标无效时误差必须为 0")
    return AimMessage(bool(valid_value), error_x, error_y)
