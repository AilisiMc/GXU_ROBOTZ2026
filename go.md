 目前实测参数：

  Black max   = 100
  White S max = 120
  White V min = 115
  Min area    = 700
  Close size  = 7
  Hold frames = 5

  状态机参数：

  lock threshold = 10 px
  lock frames    = 5
  jitter limit   = 8 px
  send rate      = 20 Hz

  激光补偿参数：

  laser offset x = 0 mm
  laser offset y = 32 mm
  target width   = 297 mm
  target height  = 210 mm
  aim offset x   = 0 px
  aim offset y   = 0 px
  
python3 vision/aiming_state_machine.py     --camera 4     --port /dev/ttyACM0     --aim-offset-x 0     --aim-offset-y 33 --aim-offset-x 0

  运行命令：

  ROBOTZ2026/vision/aiming_state_machine_stable.py

  在 Ubuntu 终端运行：

  cd /home/fishros/ROBOTZ2026

  python3 vision/aiming_state_machine_stable.py \
    --camera 4 \
    --port /dev/ttyACM0 \
    --baudrate 115200 \
    --send-rate 20 \
    --lock-threshold 10


  START,A

  视觉终端应显示：

  [FSM] RX START point=A

  随后视觉端以20 Hz发送：

  AIM,1,0,error_x,error_y
  AIM,1,1,error_x,error_y

  打靶结束后STM32发送：

  STOP

  终端应显示：

  [FSM] RX STOP -> WAIT

  如果摄像头不是 /dev/video4，先执行：

  ls -l /dev/video*

  如果串口不是 /dev/ttyACM0，执行：

  ls -l /dev/ttyACM* /dev/ttyUSB*

  仅测试摄像头和识别、不连接STM32时运行：

  cd /home/fishros/ROBOTZ2026

  python3 vision/aiming_state_machine_stable.py \
    --camera 4 \
    --dry-run \
    --auto-start A



    --port /dev/ttyACM0
    
    
    
    
    
    
    
python3 vision/aiming_state_machine_stable.py     --camera 4     --port /dev/ttyACM0     --baudrate 115200     --send-rate 20     --lock-threshold 10 --aim-offset-y 33 --lock-frame 30

