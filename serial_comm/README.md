# 串口通信模块

本目录提供 Linux 视觉程序与 STM32 之间的串口通信和文本协议。

| 文件 | 用途 |
| --- | --- |
| `serial_port.py` | 打开、配置 Linux 串口设备。 |
| `duplex_serial.py` | 双向串口读写封装，供视觉状态机使用。 |
| `aiming_protocol_fixed.py` | 稳定版视觉程序使用的瞄准协议；发送消息以 `\r\n` 结束。 |

`vision/Aiming_Stable.py` 直接依赖以上三个文件，缺少其中任何一个都会导致程序启动失败或无法正常通信。

## 串口协议

所有文本均使用 ASCII 编码，并以 CRLF（`\r\n`）作为一行结束符。

| 方向 | 消息 | 含义 |
| --- | --- | --- |
| STM32 → Linux | `START,A\r\n` | 开始 A 点识别；点位可为 `A`、`B`、`C`、`D`。 |
| STM32 → Linux | `STOP\r\n` | 停止识别与瞄准。 |
| Linux → STM32 | `AIM,valid,stable,error_x,error_y\r\n` | 识别和云台误差数据。 |

其中：

- `valid`：目标有效为 `1`，未识别到目标为 `0`。
- `stable`：目标稳定、允许执行打靶为 `1`；否则为 `0`。
- `error_x`、`error_y`：目标相对画面中心的像素误差。
- 当 `valid=0` 时，视觉端会发送 `AIM,0,0,0,0\r\n`。

## 其他文件

- `aiming_protocol.py`：早期 LF（`\n`）行尾版本，仅作兼容/参考；稳定版不使用它。
- `protocol.py`：早期通用协议文件，供旧程序参考。
- `send_mock_data.py`：手动发送测试串口数据的工具。
