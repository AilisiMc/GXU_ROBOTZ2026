# 视觉状态机使用说明

## 文件分工

- `rectangle_target.py`：冻结的稳定识别版，协议仍为 `AIM,valid,dx,dy`。
- `aiming_state_machine.py`：复制识别逻辑后增加的视觉状态机版。
- `../serial_comm/aiming_protocol.py`：新状态机协议。
- `../serial_comm/mock_stm32_controller.py`：无F407时使用的STM32模拟器。

## Ubuntu视觉状态

```text
WAIT --START,A/C/D/B--> SEARCH --发现目标--> TRACK --连续稳定--> LOCKED
 ^                         ^                  |                    |
 |                         +----目标丢失------+----目标偏离--------+
 +----------------------------- STOP -----------------------------+
```

`HOLD` 只用于画面观察，不会作为有效目标，也不会累计锁定帧。

默认锁定条件：误差绝对值不超过5像素，连续8个真实识别帧，窗口内抖动不超过5像素。

## 串口协议

STM32发送：

```text
START,A
START,C
STOP
```

Ubuntu以约20 Hz持续发送：

```text
AIM,valid,stable,error_x,error_y
```

例如：

```text
AIM,0,0,0,0
AIM,1,0,23,-8
AIM,1,1,3,-2
```

`stable=1`只是Ubuntu的视觉稳定意见。STM32仍须检查到点、停车、预置完成和数据新鲜度，才允许开激光。

## 单独测试视觉状态机

```bash
cd ~/ROBOTZ2026
python3 vision/aiming_state_machine.py --camera 4 --dry-run --auto-start A
```

画面窗口快捷键：

- `a/b/c/d`：模拟对应点位的START；
- `x`：模拟STOP并回到WAIT；
- `s`：保存原图和结果图到 `vision/fsm_captures/`；
- `r`：恢复识别滑条；
- `q`或Esc：退出。

## 与虚拟STM32联调

终端1：

```bash
cd ~/ROBOTZ2026
python3 serial_comm/mock_stm32_controller.py
```

它会显示一个端口，例如 `/dev/pts/2`。

终端2：

```bash
cd ~/ROBOTZ2026
python3 vision/aiming_state_machine.py --camera 4 --port /dev/pts/2
```

回到终端1输入 `a`、`c`、`d`或`b`并回车。模拟器连续收到3个新鲜的 `stable=1` 后打印模拟FIRE并发送STOP。超过300 ms没有视觉包时进入 `SAFE_TIMEOUT`。

## F407到手后

确认实际端口后运行，例如：

```bash
python3 vision/aiming_state_machine.py --camera 4 --port /dev/ttyACM0
```

相机和激光安装后，用下面两个参数补偿两者不完全平行造成的固定偏移：

```bash
--aim-offset-x 0 --aim-offset-y 0
```

偏移量必须通过实际激光落点标定，不能只根据相机画面猜测。
