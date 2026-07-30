# 视觉识别模块

本目录是小车打靶任务的 Linux 视觉程序。程序使用 USB 摄像头识别 A4 靶纸和目标区域，计算目标相对画面中心的像素误差，并通过串口把结果发送给 STM32，由 STM32 控制云台完成对准。

## 日常运行版本

`Aiming_Stable.py` 是当前联调和验收使用的稳定版入口。它加入了连续帧确认、空间位置筛选、面积变化筛选和指数平滑，用于减少实验室环境中杂物、反光或短暂漏检造成的云台误动作。

使用真实串口时，命令格式如下：

```bash
python3 vision/Aiming_Stable.py \
  --camera 4 \
  --port /dev/ttyACM0 \
  --baudrate 115200 \
  --send-rate 20
```

若系统中的串口名称不同，可先查看：

```bash
ls -l /dev/serial/by-id/
```

然后将 `--port` 后面的内容替换为对应设备路径。没有连接 STM32 时，可用 `--dry-run` 查看识别与状态机输出：

```bash
python3 vision/Aiming_Stable.py --camera 4 --dry-run --auto-start A
```

## 程序运行方式

启动后会显示原始画面、黑色区域掩膜和控制滑条窗口。键盘操作如下：

| 按键 | 功能 |
| --- | --- |
| `a` / `b` / `c` / `d` | 模拟 STM32 发送 `START,A`、`START,B`、`START,C`、`START,D`。 |
| `x` | 模拟 STM32 发送 `STOP`。 |
| `s` | 保存当前画面，便于排查识别效果。 |
| `r` | 重置滑条参数。 |
| `q` | 退出程序。 |

实际联调时，STM32 向 Linux 发送 `START,点位` 后，视觉程序进入识别和跟踪；收到 `STOP` 后停止瞄准。视觉端持续向 STM32 发送 `AIM,valid,stable,error_x,error_y`，详细格式见 [串口通信说明](../serial_comm/README.md)。

## 文件说明

| 文件 | 作用 |
| --- | --- |
| `Aiming_Stable.py` | 当前稳定版主程序，也是推荐运行入口。 |
| `Aiming_Test.py` | 早期状态机测试版本，可用于对照状态转换和基础识别流程。 |
| `Origin.py` | 最初的 A4 靶纸识别程序，用于观察基础识别效果。 |
| `source_style_target.py` | 采用最大黑色连通块思路的对照识别程序。 |
| `target_test.py` | 红色靶心检测测试工具。 |
| `serial_diagnose.py` | 独立串口诊断工具，可分别测试 CRLF 和 LF 行尾。 |
| `aiming_state_machine_fixed.py` | 为旧版状态机准备的 CRLF 协议兼容入口；稳定版运行时使用 `Aiming_Stable.py`。 |

## 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--camera` | `4` | 摄像头编号，对应 `/dev/video4`。 |
| `--send-rate` | `20` | 向 STM32 发送 `AIM` 数据的频率，单位 Hz。当前联调推荐 20 Hz。 |
| `--aim-offset-x` | `-7` | 黄色准星的水平像素补偿，负值表示向左。 |
| `--aim-offset-y` | `33` | 黄色准星的垂直像素补偿，正值表示向下。 |
| `--lock-frames` | `5` | 连续稳定多少帧后，将 `stable` 置为 `1`。 |
| `--lock-threshold` | `10` | 判定对准时允许的像素误差范围。 |
| `--spatial-gate-px` | `60` | 新检测结果与此前可靠位置相差过大时的筛选范围。 |

激光落点与黄色准星存在固定偏差时，可以小步调整 `--aim-offset-x` 和 `--aim-offset-y`。建议每次只改变 2～5 像素，记录修改前后的落点，再确定最终参数。

## 依赖与目录

程序依赖 Python 3、OpenCV（`cv2`）、NumPy，并会从项目根目录下的 `serial_comm/` 导入串口通信模块。运行过程中保存的截图、标定记录和 Python 缓存属于测试过程产生的数据，可用于后续复盘和参数调整。
