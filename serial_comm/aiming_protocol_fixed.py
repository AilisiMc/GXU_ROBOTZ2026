#!/usr/bin/env python3
"""Ubuntu 视觉状态机与 STM32 主状态机之间的最小文本协议（CRLF 修复版）。

与 ``aiming_protocol.py`` 的唯一区别是：所有发送行都以 ``\r\n``
结尾，适配 STM32 CDC 串口按 CRLF 分行的接收逻辑。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


POINTS = {"A", "B", "C", "D"}


@dataclass(frozen=True)
class AimStatus:
    valid: bool
    stable: bool
    error_x: int
    error_y: int


@dataclass(frozen=True)
class ControlCommand:
    action: str
    point: Optional[str] = None


def encode_aim_status(valid: bool, stable: bool, error_x: int, error_y: int) -> bytes:
    """编码为 ``AIM,valid,stable,error_x,error_y\r\n``。"""
    if not valid:
        stable = False
        error_x = 0
        error_y = 0
    return (
        f"AIM,{int(valid)},{int(stable)},{int(error_x)},{int(error_y)}\r\n"
    ).encode("ascii")


def decode_aim_status(line: bytes | bytearray | memoryview | str) -> AimStatus:
    if not isinstance(line, str):
        line = bytes(line).decode("ascii", errors="strict")
    parts = line.strip().split(",")
    if len(parts) != 5 or parts[0] != "AIM":
        raise ValueError(f"AIM 格式错误：{line!r}")
    valid_value, stable_value, error_x, error_y = map(int, parts[1:])
    if valid_value not in (0, 1) or stable_value not in (0, 1):
        raise ValueError("valid 和 stable 只能为 0 或 1")
    if stable_value and not valid_value:
        raise ValueError("stable=1 时 valid 必须为 1")
    if not -4096 <= error_x <= 4096 or not -4096 <= error_y <= 4096:
        raise ValueError("像素误差超出允许范围")
    if not valid_value and (error_x != 0 or error_y != 0):
        raise ValueError("valid=0 时误差必须为 0")
    return AimStatus(bool(valid_value), bool(stable_value), error_x, error_y)


def encode_start(point: str) -> bytes:
    point = point.upper()
    if point not in POINTS:
        raise ValueError("点位只能是 A、B、C、D")
    return f"START,{point}\r\n".encode("ascii")


def encode_stop() -> bytes:
    return b"STOP\r\n"


def decode_control(line: bytes | bytearray | memoryview | str) -> ControlCommand:
    if not isinstance(line, str):
        line = bytes(line).decode("ascii", errors="strict")
    parts = line.strip().upper().split(",")
    if parts == ["STOP"]:
        return ControlCommand("STOP")
    if len(parts) == 2 and parts[0] == "START" and parts[1] in POINTS:
        return ControlCommand("START", parts[1])
    raise ValueError(f"控制命令格式错误：{line!r}")
