"""
[Sean 2026-09-20] 獨立硬體測試專用：筆電直接接STM32(ST-Link虛擬COM口)，
不需要Jetson、不需要視覺，只負責讀取STM32主動送出的RPM,...telemetry並存成CSV。
跟Jetson端的drain_rpm_telemetry()不一樣，這支程式完全不送任何指令給STM32，
只有讀取，沒有send_command()搶佔UART buffer的問題。

用法：
    python3 read_rpm_serial.py [PORT] [CSV_PATH]
    預設 PORT=/dev/ttyACM0（Linux上接筆電時的裝置節點，如果不是這個，先用
    `ls /dev/ttyACM*` 確認），預設 CSV_PATH=rpm_log.csv
    按 Ctrl+C 停止，會自動把CSV flush、關檔。
"""

import csv
import json
import sys
import time
from collections import deque

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
CSV_PATH = sys.argv[2] if len(sys.argv) > 2 else "rpm_log.csv"
BAUD = 115200

# [Sean 2026-09-20] 即時網頁儀表板用：每筆資料同時寫進這個小json檔(只留最新N筆)，
# 讓本機網頁用fetch()輪詢，做出像示波器一樣會捲動的即時圖表。
LIVE_JSON_PATH = "rpm_live.json"
LIVE_WINDOW = 200


def main():
    print("Opening", PORT, "@", BAUD)
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(2)
    ser.reset_input_buffer()

    csv_file = open(CSV_PATH, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "laptop_time_s",
        "stm32_ms",
        "target_left_rpm",
        "actual_left_rpm",
        "target_right_rpm",
        "actual_right_rpm",
    ])

    t0 = time.time()
    row_count = 0
    live_buf = deque(maxlen=LIVE_WINDOW)

    print("Reading RPM telemetry, press Ctrl+C to stop...")
    print("Live dashboard JSON:", LIVE_JSON_PATH)

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if not line.startswith("RPM,"):
                print("(ignored)", line)
                continue

            parts = line.split(",")
            if len(parts) != 6:
                print("(malformed)", line)
                continue

            t = time.time() - t0
            writer.writerow([t] + parts[1:])
            row_count += 1

            if row_count % 10 == 0:
                csv_file.flush()

            try:
                live_buf.append({
                    "t": t,
                    "target_left": float(parts[2]),
                    "actual_left": float(parts[3]),
                    "target_right": float(parts[4]),
                    "actual_right": float(parts[5]),
                })
                with open(LIVE_JSON_PATH, "w") as f:
                    json.dump(list(live_buf), f)
            except ValueError:
                pass

            print(
                "t=%.2fs  stm32_ms=%s  L target=%s actual=%s  R target=%s actual=%s"
                % (t, parts[1], parts[2], parts[3], parts[4], parts[5])
            )

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        csv_file.close()
        ser.close()
        print("Rows saved:", row_count)
        print("CSV saved:", CSV_PATH)


if __name__ == "__main__":
    main()
