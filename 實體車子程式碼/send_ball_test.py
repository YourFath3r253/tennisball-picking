"""方向測試：從筆電經 USB (/dev/ttyACM0) 送假的 Jetson 指令給 STM32。

用法:
  python3 send_ball_test.py 60 25     # 送 BALL,60,25 (球 60 cm、偏右 25 度)，每 0.2 s 送一次直到 Ctrl-C
  python3 send_ball_test.py off       # 送 BALL_OFF
STM32 收到 BALL 之後會一直依最後一筆角度轉；Ctrl-C 後會自動送 BALL_OFF (之後是巡邏直走)。
"""
import sys
import time

import serial

ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)
if sys.argv[1] == "off":
    ser.write(b"BALL_OFF\n")
    print("sent BALL_OFF")
    sys.exit(0)

dist, ang = float(sys.argv[1]), float(sys.argv[2])
line = ("BALL,%.1f,%.1f\n" % (dist, ang)).encode()
print("sending", line.strip().decode(), "every 0.2 s (Ctrl-C to stop)")
try:
    while True:
        ser.write(line)
        time.sleep(0.2)
except KeyboardInterrupt:
    ser.write(b"BALL_OFF\n")
    print("\nsent BALL_OFF")
