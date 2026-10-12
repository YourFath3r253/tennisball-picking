"""讀 STM32 的 GYRO 回傳（USB /dev/ttyACM0），即時印出陀螺儀角度 vs 編碼器角度，並存 CSV。

STM32 每 100 ms 送一行：
GYRO,時間ms,陀螺儀角度,編碼器角度,陀螺儀正常(0/1),左目標rpm,左實際rpm,右目標rpm,右實際rpm

用法: python3 read_gyro_serial.py [輸出檔名.csv] [序列埠]
按 Ctrl-C 結束。印出的 Δ 是從程式啟動算起的變化量，方便用手轉車子比對。
"""
import csv
import sys
import time

import serial

out = sys.argv[1] if len(sys.argv) > 1 else time.strftime("gyro_log_%Y%m%d_%H%M%S.csv")
port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM0"

ser = serial.Serial(port, 115200, timeout=0.5)
f = open(out, "w", newline="")
w = csv.writer(f)
w.writerow(["laptop_s", "stm32_ms", "gyro_deg", "encoder_deg", "gyro_ok",
            "target_left_rpm", "actual_left_rpm", "target_right_rpm", "actual_right_rpm"])
t0 = time.time()
g0 = e0 = None
print("logging to", out, "(Ctrl-C to stop)")
try:
    while True:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if not line.startswith("GYRO,"):
            continue
        p = line.split(",")
        if len(p) != 9:
            continue
        w.writerow(["%.3f" % (time.time() - t0)] + p[1:])
        f.flush()
        g, e, ok = float(p[2]), float(p[3]), p[4]
        if g0 is None:
            g0, e0 = g, e
        print("gyro %8.1f (Δ%+7.1f)  encoder %8.1f (Δ%+7.1f)  gyro_ok=%s  L %s/%s  R %s/%s"
              % (g, g - g0, e, e - e0, ok, p[5], p[6], p[7], p[8]), flush=True)
except KeyboardInterrupt:
    pass
finally:
    f.close()
    print("saved", out)
