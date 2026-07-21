# 7月20日视觉组阶段总结

## 1. 当前阶段结论

目前已经完成：

- Ubuntu下USB摄像头采集；
- A4靶纸黑胶带矩形识别；
- 旋转矩形和B点强透视梯形识别；
- 近距离与远距离识别参数测试；
- 像素误差 `error_x/error_y` 计算；
- 串口模拟输出；
- Ubuntu视觉子状态机；
- 虚拟STM32双向联调程序；
- 视觉数据超时安全方案。

当前识别测试结果基本满意。下一阶段重点不再是继续频繁重写识别算法，而是与F407、电控云台、激光进行闭环联调和标定。

## 2. 运行环境

- Ubuntu 22.04；
- 系统Python：`/usr/bin/python3`；
- OpenCV：4.5.4；
- 项目目录：`~/ROBOTZ2026`。

如果终端前面显示 `(base)`，先退出Conda环境：

```bash
conda deactivate
```

确认OpenCV：

```bash
which python3
python3 -c "import cv2; print(cv2.__version__)"
```

正常结果应包含：

```text
/usr/bin/python3
4.5.4
```

## 3. 摄像头设备确认

摄像头重新插拔后，`/dev/video`编号可能发生变化。

```bash
ls -l /dev/video*
```

例如出现 `/dev/video4`，程序参数使用：

```text
--camera 4
```

如果摄像头打不开：

1. 关闭之前尚未退出的Python、浏览器或摄像头程序；
2. 直接连接电脑测试，暂时去掉3米USB延长线；
3. 换USB接口；
4. 重新执行 `ls -l /dev/video*`；
5. 必要时用 `sudo dmesg -w` 观察重新插拔日志。

## 4. 稳定识别版

文件：

```text
vision/rectangle_target.py
```

这份文件作为当前稳定识别基线保留，不在状态机联调过程中随意重写。

运行：

```bash
cd ~/ROBOTZ2026
python3 vision/rectangle_target.py --camera 4 --dry-run
```

推荐测试参数：

```text
Black max   = 100
White S max = 120
White V min = 115
Min area    = 700
Close size  = 7
Hold frames = 5
```

参数含义：

- `Black max`：小于该灰度的像素进入黑胶带掩膜；
- `White S max`：允许白纸发生偏蓝、偏色的最大饱和度；
- `White V min`：白纸最低亮度；
- `Min area`：候选轮廓最小面积；
- `Close size`：连接黑胶带小断口的闭运算尺寸；
- `Hold frames`：短暂漏检时保留显示的帧数，HOLD不会发送有效目标。

窗口观察标准：

- `Black mask`中四条黑胶带应主要显示为白色；
- `White paper mask`中A4纸内部应形成独立白色区域；
- 绿色框应跟随靶纸，而不是纸板、门框或其他背景；
- 绿色十字应位于靶纸几何中心附近。

快捷键：

- `s`：保存原图和结果图；
- `r`：恢复程序默认滑条；
- `q`或Esc：退出。

截图目录：

```text
vision/captures/
```

诊断识别问题时优先提供：

```text
a4_raw_时间.jpg
```

原始摄像头帧比桌面截图更适合分析。

## 5. 当前识别逻辑

识别采用两条路径：

1. 主路径：在Black mask中寻找闭合黑色外环及其内部四边形孔洞；
2. 备用路径：寻找白色A4内部，再验证四周存在黑胶带。

主路径可以处理：

- 画面内旋转；
- B点斜视造成的梯形透视；
- 白纸被自动白平衡拍成偏蓝色的情况。

备用路径用于黑胶带反光、局部断线等情况。

靶心使用四边形两条对角线交点，不检测远距离红点。红色圆和红色中心点线宽过细，不适合作为远距离主要识别依据。

## 6. Ubuntu视觉状态机版

文件：

```text
vision/aiming_state_machine.py
```

视觉状态：

```text
WAIT
  ↓ START,A/C/D/B
SEARCH
  ↓ 发现目标
TRACK
  ↓ 连续稳定
LOCKED
  ↓ 目标偏离
TRACK
  ↓ 目标丢失
SEARCH
  ↓ STOP
WAIT
```

职责划分：

- Ubuntu负责摄像头、靶纸识别、误差和图像稳定性；
- STM32负责车辆流程、点位、云台PID、预置角、超时和激光开关；
- Ubuntu不能直接决定或控制激光开启。

单独测试状态机：

```bash
cd ~/ROBOTZ2026
python3 vision/aiming_state_machine.py \
  --camera 4 \
  --dry-run \
  --auto-start A \
  --lock-threshold 10 \
  --lock-frames 5 \
  --jitter-limit 8
```

快捷键：

- `a/b/c/d`：模拟对应点位的START；
- `x`：模拟STOP并回到WAIT；
- `s`：保存状态机截图；
- `r`：恢复识别滑条；
- `q`或Esc：退出。

状态机截图目录：

```text
vision/fsm_captures/
```

## 7. 锁定参数建议

最初的调试参数为5像素、8帧，比较保守。考虑总流程限时3分钟，培训阶段建议先使用：

```text
lock threshold = 10像素
lock frames    = 5帧
jitter limit   = 8像素
STM32确认包    = 2包
```

在约18 FPS时，视觉稳定确认约为：

```text
5 / 18 ≈ 0.28秒
```

加上STM32两包确认，总确认时间约0.4秒。搜索和云台转动耗时通常大于视觉确认时间。

注意：激光光斑直径不超过0.5 cm，不代表瞄准误差必须小于0.5 cm。评分按照落点距离靶心的2、4、6、8、10 cm圆环计算。

最终建议根据画面中靶框大小设置自适应阈值：

```text
严格锁定阈值 ≈ 靶框短边像素数 × 4%
放宽锁定阈值 ≈ 靶框短边像素数 × 8%
```

不要因为追求过严的像素误差导致某一点无限等待。

## 8. 串口协议

状态机版采用新五字段协议。

STM32发送：

```text
START,A
START,C
START,D
START,B
STOP
```

Ubuntu发送：

```text
AIM,valid,stable,error_x,error_y
```

示例：

```text
AIM,0,0,0,0
AIM,1,0,25,-8
AIM,1,1,3,-2
```

含义：

- `valid=0`：没有当前可靠目标；
- `valid=1, stable=0`：目标有效，STM32继续云台PID；
- `valid=1, stable=1`：Ubuntu认为图像已稳定，但STM32仍要检查车辆停车、点位、预置完成和数据新鲜度。

稳定识别版仍使用旧四字段协议：

```text
AIM,valid,error_x,error_y
```

与F407正式联调时应采用状态机版五字段协议，不能混用解析程序。

## 9. 虚拟STM32联调

终端1：

```bash
cd ~/ROBOTZ2026
python3 serial_comm/mock_stm32_controller.py --confirm-packets 2
```

程序会输出类似：

```text
虚拟STM32串口：/dev/pts/2
```

终端2使用实际显示的端口：

```bash
cd ~/ROBOTZ2026
python3 vision/aiming_state_machine.py \
  --camera 4 \
  --port /dev/pts/2 \
  --lock-threshold 10 \
  --lock-frames 5 \
  --jitter-limit 8
```

回到终端1输入 `a`、`c`、`d`或`b`并回车，模拟STM32到达对应点并完成云台预置。

虚拟STM32连续收到指定数量的 `stable=1` 后打印模拟FIRE并发送STOP。超过300 ms没有视觉数据时进入：

```text
SAFE_TIMEOUT
```

## 10. F407接入后的运行方式

单片机插入Ubuntu后确认端口：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
```

常见情况：

- STM32 USB CDC：`/dev/ttyACM0`；
- CH340 USB转TTL：`/dev/ttyUSB0`。

端口由Ubuntu实际枚举结果确定，不由STM32型号直接确定。

示例：

```bash
cd ~/ROBOTZ2026
python3 vision/aiming_state_machine.py \
  --camera 4 \
  --port /dev/ttyACM0 \
  --lock-threshold 10 \
  --lock-frames 5 \
  --jitter-limit 8
```

通信参数：

```text
115200 baud，8N1，换行符结尾
```

## 11. 云台与整车流程建议

推荐STM32主流程：

```text
DRIVE
  ↓ 到达A/C/D/B
PRESET
  ↓ 转到该点预置yaw/pitch
AIMING
  ↓ 根据视觉dx/dy执行PID
LOCK_CONFIRM
  ↓ 数据新鲜、车辆停稳、视觉稳定
FIRE
  ↓ 关闭激光并发送STOP
DRIVE
```

需要提前标定四组预置角：

```text
A: yaw_A, pitch_A
C: yaw_C, pitch_C
D: yaw_D, pitch_D
B: yaw_B, pitch_B
```

预置角负责让靶纸进入摄像头视野，视觉负责最后微调。行驶时可以提前朝向下一点的预置方向，但不建议依靠剧烈晃动中的画面持续追踪完成打靶。

## 12. 时间与安全策略

文档规定整个流程限时3分钟。建议每个打靶点：

```text
前2秒：严格阈值，争取进入2 cm圆；
第2～3秒：适当放宽阈值，优先完成打靶；
超过3秒：结束该点，继续整车流程。
```

四次打靶最大预算约12秒，避免影响上高台和夹取任务。

STM32必须实现：

- 超过300 ms没有合法视觉包：关闭激光、停止使用旧误差；
- `valid=0`：不得继续使用上一帧误差；
- 车辆未停车：不得开激光；
- 非A/C/D/B打靶阶段：不得开激光；
- 程序退出、串口掉线、摄像头掉线：激光默认关闭；
- 激光开启后按规定时间关闭，不能依赖Ubuntu持续发送开激光命令。

## 13. 激光与相机标定

黄色瞄准点目前默认是图像中心，但相机光轴和激光方向不可能安装得绝对重合。

状态机支持：

```text
--aim-offset-x
--aim-offset-y
```

示例：

```bash
python3 vision/aiming_state_machine.py \
  --camera 4 \
  --dry-run \
  --auto-start A \
  --aim-offset-x 6 \
  --aim-offset-y -4
```

最终偏移必须在云台、摄像头和激光安装完成后，根据真实激光落点标定。标定误差、舵机回差和结构晃动对得分的影响可能大于视觉5～10像素的差别。

## 14. 下一阶段待办

1. 与电控确认五字段串口协议、误差正负方向和换行格式；
2. F407到手后确认 `/dev/ttyACM*` 或 `/dev/ttyUSB*`；
3. 打通Ubuntu到STM32的真实串口；
4. 确认 `error_x/error_y` 对应的yaw/pitch转动方向；
5. 调整云台PID，避免过冲和来回振荡；
6. 标定A、C、D、B四组预置角；
7. 标定相机与激光固定偏移；
8. 实测每个点从停车到开火所需时间；
9. 加入单点3秒超时及严格/放宽两阶段策略；
10. 完成整车A-C-D-B四点联调。

## 15. 今日保留原则

- 不随意修改已经通过测试的 `rectangle_target.py`；
- 新功能优先在 `aiming_state_machine.py` 中验证；
- 每次修改后同时回测近距离、远距离和B点斜视；
- 识别失败时保存原始摄像头帧，不只保存桌面截图；
- 先保证四次打靶可靠完成，再追求更高精度和更复杂协议。
