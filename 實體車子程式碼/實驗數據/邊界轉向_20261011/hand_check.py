"""每兩幀估計相似變換（RANSAC），記錄上下位移 ty 和畫面傾斜角 roll。
車子自己原地轉：主要只有水平位移；被手拿起/推動：ty 和 roll 會明顯變化。"""
import csv, math, sys
import cv2, numpy as np
sys.path.insert(0, '.')
from video_yaw import BAND, MASK
name = sys.argv[1]
cap = cv2.VideoCapture(name + ".mp4")
orb = cv2.ORB_create(nfeatures=800, fastThreshold=10)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
out = csv.writer(open(name + "_motion.csv", "w")); out.writerow(["frame", "ty_px", "roll_deg", "n"])
prev = None; i = 0
while True:
    ok, f = cap.read()
    if not ok: break
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)[BAND[0]:BAND[1], :]
    ty = roll = ""; n = 0
    if prev is not None:
        k0, d0 = orb.detectAndCompute(prev, MASK); k1, d1 = orb.detectAndCompute(g, MASK)
        if d0 is not None and d1 is not None and len(k0) > 8 and len(k1) > 8:
            ms = bf.match(d0, d1)
            if len(ms) >= 8:
                p0 = np.float32([k0[m.queryIdx].pt for m in ms]); p1 = np.float32([k1[m.trainIdx].pt for m in ms])
                M, inl = cv2.estimateAffinePartial2D(p0, p1, method=cv2.RANSAC, ransacReprojThreshold=3.0)
                if M is not None and inl is not None and inl.sum() >= 8:
                    ty = "%.2f" % M[1, 2]; roll = "%.2f" % math.degrees(math.atan2(M[1, 0], M[0, 0])); n = int(inl.sum())
    out.writerow([i + 1, ty, roll, n]); prev = g; i += 1
print(name, "frames", i)
