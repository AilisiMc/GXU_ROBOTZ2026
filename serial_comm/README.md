# 视觉—STM32 串口测试

通信参数：`115200, 8N1`，每条消息以换行符结束。

视觉发送：

```text
AIM,1,-45,18
AIM,0,0,0
```

字段依次为：消息类型、目标是否有效、横向像素误差、纵向像素误差。

## 无单片机测试

```bash
cd ~/ROBOTZ2026/serial_comm
python3 virtual_serial_demo.py
```

程序会在 Linux 中创建虚拟串口，模拟视觉端发送和 STM32 端回复。

只观察连续模拟输出：

```bash
python3 send_mock_data.py --dry-run --rate 20
```

按 `Ctrl+C` 停止。

## STM32 到手后

先查看端口：

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

再发送模拟数据：

```bash
python3 send_mock_data.py --port /dev/ttyACM0 --rate 20
```

USB CDC 通常是 `/dev/ttyACM0`，CH340 通常是 `/dev/ttyUSB0`。
