# 电控组任务与状态机建议

本文档面向电控组，依据当前培训任务、视觉组现有程序和串口联调方案整理。核心原则是：

```text
Ubuntu视觉端负责：看见靶纸、计算偏差、报告是否稳定。
STM32电控端负责：整车流程、循迹、停车、云台PID、激光开关和安全保护。
```

不要把激光开关交给Ubuntu直接控制。Ubuntu只提供视觉结果，最终是否开激光必须由STM32结合车辆状态判断。

## 1. 电控组需要完成的事情

### 1.1 小车运动与点位判断

电控组需要让小车按文档路线行驶：

```text
A -> C -> D -> B
```

需要完成：

- 黑线循迹；
- 判断是否到达A、C、D、B四个打靶点；
- 到点后停车；
- 停车后等待车身稳定一小段时间，建议 `300~600 ms`；
- 打靶完成后继续进入下一个点位。

视觉组不负责循迹，也不负责判断小车是否到达点位。

### 1.2 云台控制

电控组需要控制二维云台：

- yaw轴：水平转动；
- pitch轴：上下转动；
- 根据视觉发来的 `error_x/error_y` 做闭环控制；
- 注意视觉误差已经以“激光补偿后的黄色瞄准点”为零点，STM32不要再次叠加
  摄像头与激光笔的32 mm机械偏移；
- 给舵机或云台电机做限幅，避免撞机械极限；
- 云台PID建议放在STM32上运行。

视觉端只发像素偏差，例如：

```text
error_x = +25
error_y = -8
```

具体转多少PWM、往哪个方向转、PID参数怎么调，应该由STM32负责。

### 1.3 四个点位的云台预置角

电控组需要提前标定四组预置角：

```text
A: yaw_A, pitch_A
C: yaw_C, pitch_C
D: yaw_D, pitch_D
B: yaw_B, pitch_B
```

标定方法：

1. 把车分别放到A、C、D、B四个实际停车位置。
2. 保持车辆正式比赛时的朝向。
3. 手动调整云台，让摄像头能看到完整靶纸。
4. 记录当前yaw和pitch角度。
5. 正式运行时，到达某点后先转到该点预置角，再启动视觉瞄准。

预置角的作用是让靶纸快速进入画面。视觉只负责最后的小范围微调。

### 1.4 激光控制

激光必须由STM32控制，且默认关闭。

允许开激光的条件建议同时满足：

- 当前处于A、C、D、B打靶点；
- 小车已经停车；
- 云台已经完成预置角；
- 最近 `300 ms` 内收到有效视觉数据；
- 视觉目标有效；
- 瞄准满足锁定条件；
- 没有串口异常、摄像头异常或云台极限异常。

激光开启后应短时间关闭，不要依赖Ubuntu持续发送命令来关闭激光。

## 2. 推荐串口协议

### 2.1 STM32发送给Ubuntu

STM32在到达某个点，并完成停车和预置角后，发送：

```text
START,A
START,C
START,D
START,B
```

打靶完成或放弃当前点后，发送：

```text
STOP
```

### 2.2 Ubuntu发送给STM32

Ubuntu以约 `20 Hz` 发送：

```text
AIM,valid,stable,error_x,error_y
```

示例：

```text
AIM,0,0,0,0
AIM,1,0,25,-8
AIM,1,1,3,-2
```

字段含义：

| 字段 | 含义 |
|---|---|
| `valid` | `1`表示找到靶纸，`0`表示没有可靠目标 |
| `stable` | `1`表示视觉认为目标已稳定，`0`表示还未稳定 |
| `error_x` | 靶心相对瞄准点的横向像素偏差 |
| `error_y` | 靶心相对瞄准点的纵向像素偏差 |

坐标约定：

```text
error_x > 0：靶心在画面右侧
error_x < 0：靶心在画面左侧
error_y > 0：靶心在画面下方
error_y < 0：靶心在画面上方
```

STM32需要把这个坐标方向映射到云台yaw/pitch方向。如果舵机方向反了，只改STM32里的方向符号，不要改视觉协议定义。

### 2.3 通信参数

建议使用：

```text
115200 baud
8N1
ASCII文本
每条消息以 \n 结束
```

物理链路优先级：

```text
STM32 USB CDC > CH340 USB转TTL
```

如果STM32 USB CDC枚举成功，Ubuntu通常会出现：

```text
/dev/ttyACM0
```

如果使用CH340，Ubuntu通常会出现：

```text
/dev/ttyUSB0
```

实际端口必须插入单片机后在Ubuntu里查看，不能只根据芯片型号猜。

## 3. STM32主状态机

推荐电控主状态机如下：

```text
IDLE
  -> DRIVE
  -> ARRIVE
  -> SETTLING
  -> PRESET
  -> AIMING
  -> FIRE
  -> NEXT_POINT
  -> DONE
```

可以用一个变量记录当前点位：

```c
current_point = A / C / D / B
```

不建议给A、C、D、B分别写四套重复状态机。点位不同只影响预置角和路线阶段，状态逻辑应复用。

## 4. 各状态职责

| 状态 | STM32要做什么 | 退出条件 |
|---|---|---|
| `IDLE` | 等待启动按键或上位命令，激光关闭 | 收到启动 |
| `DRIVE` | 按黑线循迹前往当前点 | 检测到当前点 |
| `ARRIVE` | 停车，清零运动控制输出 | 确认车辆停住 |
| `SETTLING` | 等待车身和云台机械振动衰减 | `300~600 ms` 到时 |
| `PRESET` | 云台转到当前点预置角 | 角度到位或超时 |
| `AIMING` | 发送 `START,点位`，接收视觉误差，运行云台PID | 锁定成功或瞄准超时 |
| `FIRE` | 开激光短时间打靶，然后关激光，发送 `STOP` | 激光驻留结束 |
| `NEXT_POINT` | 切换到下一个点 | 还有点则回到 `DRIVE`，否则 `DONE` |
| `DONE` | 停车，关激光，结束任务 | 无 |
| `FAULT` | 异常保护，停车关激光 | 人工复位 |

## 5. AIMING状态内部逻辑

`AIMING`是电控和视觉配合最关键的状态。

推荐逻辑：

```text
进入AIMING:
  发送 START,current_point
  清空锁定计数
  记录 aiming_start_time

循环执行:
  如果超过300 ms没收到视觉包:
    关闭激光
    云台停止或进入小范围搜索
    清空锁定计数

  如果收到 AIM,0,0,0,0:
    目标丢失
    云台在预置角附近小范围搜索
    清空锁定计数

  如果收到 AIM,1,0,dx,dy:
    目标有效但未稳定
    根据 dx/dy 做云台PID
    清空或降低锁定计数

  如果收到 AIM,1,1,dx,dy:
    继续微调云台
    锁定计数 + 1

  如果锁定计数达到2~3包:
    进入 FIRE

  如果AIMING持续超过3秒:
    按超时策略处理
```

培训阶段建议：

```text
Ubuntu锁定条件：10像素，连续5帧，抖动不超过8像素
STM32确认条件：连续2包 stable=1
单点瞄准上限：约3秒
视觉数据超时：约300 ms
```

## 6. 搜索策略

不要每次都让云台盲目转满360度。

推荐顺序：

```text
1. 到点后先转到该点预置角。
2. 如果 valid=0，在预置角附近左右小范围扫描，例如 ±20度。
3. 发现目标后停止扫描，进入PID微调。
4. 如果 3~5 秒仍找不到，再扩大扫描范围。
5. 超过单点时间预算后放弃该点，继续整体流程。
```

行驶过程中可以提前把云台转向下一个点的大致预置角，但不要在车身剧烈晃动时开激光。

## 7. 时间策略

文档要求整体流程限时约3分钟，因此单点打靶不能无限等待。

建议每个点：

```text
0~2秒：严格瞄准，争取高分
2~3秒：适当放宽，优先完成打靶
超过3秒：放弃该点或按当前最好结果打一次，然后继续流程
```

四个点位总瞄准预算约 `12秒`。搜索和调云台耗时通常比视觉连续帧确认更关键。

注意：文档中的 `激光光斑直径 <= 0.5 cm` 是光斑大小要求，不等于瞄准误差必须小于 `0.5 cm`。评分按靶心圆环距离计算，所以应该在时间和精度之间做取舍。

## 8. 必须实现的安全保护

STM32至少要实现以下保护：

| 异常 | STM32处理 |
|---|---|
| 超过300 ms没收到视觉包 | 关闭激光，停止使用旧误差 |
| `valid=0` | 关闭激光，不使用上一帧dx/dy继续控制 |
| 车辆未停车 | 禁止开激光 |
| 不在A/C/D/B打靶状态 | 禁止开激光 |
| 云台到机械极限 | 停止继续往极限方向输出 |
| 串口解析错误 | 丢弃该帧，不更新控制量 |
| Ubuntu程序退出 | 由通信超时自动保护 |
| 单点瞄准超时 | 关闭激光，进入下一流程或FAULT |

激光控制必须是“默认关、满足条件才开、开后定时关”。

## 9. 与视觉组联调顺序

建议按这个顺序联调：

1. 电控单独完成循迹和A/C/D/B到点停车。
2. 电控单独完成云台yaw/pitch基础控制和限幅。
3. 电控给每个点标定预置角。
4. Ubuntu和STM32打通串口，先只看能否收到 `AIM`。
5. 电控发送 `START,A`，确认Ubuntu从 `WAIT` 进入识别。
6. 手动移动靶纸，确认 `error_x/error_y` 正负方向。
7. 单轴测试云台PID，先yaw后pitch，不要两个轴一起调。
8. 加入 `stable=1` 后的开激光流程。
9. 加入300 ms视觉超时保护。
10. 按A、C、D、B四点完整跑一遍。

## 10. 电控组最终交付检查表

- 能稳定循迹并到达A、C、D、B。
- 到点后能停车并等待稳定。
- 每个点有独立云台预置角。
- 能发送 `START,A/C/D/B` 和 `STOP`。
- 能解析 `AIM,valid,stable,error_x,error_y`。
- `valid=1` 时云台能按dx/dy闭环调整。
- `stable=1` 连续确认后能短时开激光。
- 视觉断线、目标丢失、串口异常时激光自动关闭。
- 单点瞄准不会无限等待。
- 四个点能完成完整流程。

## 11. 最简伪代码

```c
while (1) {
    update_uart_rx();
    update_line_tracking();
    update_gimbal();
    update_timers();

    switch (state) {
    case IDLE:
        laser_off();
        if (start_pressed()) {
            current_point = POINT_A;
            state = DRIVE;
        }
        break;

    case DRIVE:
        laser_off();
        line_follow_to(current_point);
        if (arrived_at(current_point)) {
            stop_car();
            state = ARRIVE;
        }
        break;

    case ARRIVE:
        stop_car();
        reset_timer();
        state = SETTLING;
        break;

    case SETTLING:
        if (timer_ms() > 500) {
            set_gimbal_preset(current_point);
            state = PRESET;
        }
        break;

    case PRESET:
        if (gimbal_preset_done() || timer_ms() > 1000) {
            uart_send_start(current_point);
            clear_lock_count();
            reset_aiming_timer();
            state = AIMING;
        }
        break;

    case AIMING:
        laser_off();

        if (vision_timeout_ms() > 300) {
            clear_lock_count();
            gimbal_search_near_preset();
            break;
        }

        if (!vision.valid) {
            clear_lock_count();
            gimbal_search_near_preset();
            break;
        }

        gimbal_pid_update(vision.error_x, vision.error_y);

        if (vision.stable) {
            lock_count++;
        } else {
            lock_count = 0;
        }

        if (lock_count >= 2) {
            state = FIRE;
            reset_timer();
        } else if (aiming_timer_ms() > 3000) {
            state = NEXT_POINT;
        }
        break;

    case FIRE:
        laser_on();
        if (timer_ms() > FIRE_TIME_MS) {
            laser_off();
            uart_send_stop();
            state = NEXT_POINT;
        }
        break;

    case NEXT_POINT:
        if (current_point == POINT_B) {
            state = DONE;
        } else {
            current_point = next_point(current_point);
            state = DRIVE;
        }
        break;

    case DONE:
        stop_car();
        laser_off();
        break;

    case FAULT:
    default:
        stop_car();
        laser_off();
        break;
    }
}
```
