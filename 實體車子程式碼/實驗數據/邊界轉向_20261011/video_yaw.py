"""從車上相機影片估計車身轉角（原地轉時畫面整體水平移動）。

只用畫面上方的遠景（牆、桌腳），避開近處地板的視差和左下角疊字。
每兩幀用 ORB 特徵配對，取配對點水平角度差的中位數當這一幀的轉角，沿時間累加。
用法: python3 video_yaw.py <TEST_NAME>   （讀 <TEST_NAME>.mp4 和 <TEST_NAME>_court.csv）
輸出: <TEST_NAME>_yaw.csv（每幀時間、轉角增量、累積轉角、配對數、court 狀態）
"""
import csv
import math
import sys

import cv2
import numpy as np

FOCAL_PX = 390.0      # D 程式的 FOCAL_LENGTH_PX（512 寬）
CX = 283.0       # D 程式 estimate_ball_position 的 image_cx
BAND = (0, 140)       # 上方遠景
# 疊字區塊不取特徵點：左上 "No ball detected"、右上 "MOTOR: ON/OFF"
TEXT_BOXES = [(0, 0, 220, 40), (330, 0, 480, 40)]
MIN_MATCHES = 8


def bearing(x):
    return math.atan((x - CX) / FOCAL_PX)


def band_mask():
    m = np.full((BAND[1] - BAND[0], 512), 255, np.uint8)
    for x0, y0, x1, y1 in TEXT_BOXES:
        m[max(0, y0 - BAND[0]):max(0, y1 - BAND[0]), x0:x1] = 0
    return m


MASK = band_mask()


def frame_yaw(orb, bf, g0, g1):
    k0, d0 = orb.detectAndCompute(g0, MASK)
    k1, d1 = orb.detectAndCompute(g1, MASK)
    if d0 is None or d1 is None or len(k0) < MIN_MATCHES or len(k1) < MIN_MATCHES:
        return None, 0
    matches = bf.match(d0, d1)
    dth = []
    for m in matches:
        x0, y0 = k0[m.queryIdx].pt
        x1, y1 = k1[m.trainIdx].pt
        if abs(y1 - y0) > 12:
            continue
        # 鏡頭向左轉 -> 景物往右移 -> bearing 變大；車身逆時針(左轉)為正
        dth.append(bearing(x1) - bearing(x0))
    if len(dth) < MIN_MATCHES:
        return None, len(dth)
    dth = np.array(dth)
    med = np.median(dth)
    inl = dth[np.abs(dth - med) < math.radians(2.0)]
    if len(inl) < MIN_MATCHES:
        return None, len(inl)
    return math.degrees(float(np.median(inl))), len(inl)


def main(name):
    raw = open(name + "_court.csv", "rb").read().replace(b"\0", b"").decode("utf-8", "ignore")
    rows = [r for r in csv.DictReader(raw.splitlines()) if r.get("frame") and r.get("time_s")]
    # court 不是每個迴圈都處理，用 frame 欄位對照影片幀號，中間的幀線性內插時間
    by_frame = {int(r["frame"]): r for r in rows}
    fr_known = sorted(by_frame)
    t_known = [float(by_frame[k]["time_s"]) for k in fr_known]
    cap = cv2.VideoCapture(name + ".mp4")
    nvid = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    orb = cv2.ORB_create(nfeatures=800, fastThreshold=10)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    out = csv.writer(open(name + "_yaw.csv", "w"))
    out.writerow(["frame", "time_s", "dyaw_deg", "yaw_deg", "n_inliers", "state", "line_side"])
    prev = None
    yaw = 0.0
    i = 0
    ok = fail = 0
    while True:
        ret, fr = cap.read()
        if not ret:
            break
        g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)[BAND[0]:BAND[1], :]
        fno = i + 1
        t = float(np.interp(fno, fr_known, t_known))
        near = by_frame.get(fno)
        st = near["state"] if near else ""
        side = near["line_side"] if near else ""
        d, n = (0.0, 0) if prev is None else frame_yaw(orb, bf, prev, g)
        if d is None:
            fail += 1
            d_out = ""
        else:
            ok += 1
            yaw += d
            d_out = "%.2f" % d
        out.writerow([i + 1, "%.3f" % t, d_out, "%.2f" % yaw, n, st, side])
        prev = g
        i += 1
    print("%s: video frames=%d court rows=%d tracked=%d failed=%d final yaw=%.1f deg"
          % (name, nvid, len(rows), ok, fail, yaw))


if __name__ == "__main__":
    main(sys.argv[1])
