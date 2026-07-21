 目前实测推荐参数：

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

  运行命令：

  cd ~/ROBOTZ2026

  python3 vision/aiming_state_machine.py \
    --camera 4 \
    --dry-run \
    --auto-start A \
    --lock-threshold 10 \
    --lock-frames 5 \
    --jitter-limit 8 \
    --laser-offset-x-mm 0 \
    --laser-offset-y-mm 32 \
    --target-width-mm 297 \
    --target-height-mm 210
    4. 不要在正式运行时使用--dry-run

  --dry-run每秒打印约20行，4分钟约4800行。一般不会卡死，但终端滚动和大量输出会浪
  费资源。连接STM32后使用：


    --port /dev/ttyACM0

