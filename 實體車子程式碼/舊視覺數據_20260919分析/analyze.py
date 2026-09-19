import csv
import statistics as st
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Noto Serif CJK TC"
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt

def load(path):
    rows = []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)
    return rows

def col(rows, name, cast=float):
    out = []
    for r in rows:
        v = r.get(name, "")
        if v == "":
            continue
        try:
            out.append(cast(v))
        except ValueError:
            pass
    return out

def summarize_static(path, label):
    rows = load(path)
    if not rows:
        print(f"{label}: 空檔案")
        return
    dist = col(rows, "dist_cm")
    bearing = col(rows, "bearing_deg")
    conf = col(rows, "conf")
    n = len(rows)
    print(f"--- {label} ({n} 幀) ---")
    if dist:
        print(f"  dist_cm: mean={st.mean(dist):.2f} std={st.pstdev(dist):.2f} min={min(dist):.2f} max={max(dist):.2f}")
    if bearing:
        print(f"  bearing_deg: mean={st.mean(bearing):.2f} std={st.pstdev(bearing):.2f} min={min(bearing):.2f} max={max(bearing):.2f}")
    if conf:
        print(f"  conf: mean={st.mean(conf):.3f} min={min(conf):.3f}")

print("=" * 60)
print("靜態校正測試（6/21，無 motor_on，球固定在已知位置）")
print("=" * 60)
summarize_static("test_50cm_center.csv", "test_50cm_center (球在正中央50cm)")
summarize_static("test_50cm_left.csv", "test_50cm_left (球在左側50cm)")
summarize_static("test_50cm_right.csv", "test_50cm_right (球在右側50cm)")
summarize_static("test_distance.csv", "test_distance")
summarize_static("distance_log_512x384.csv", "distance_log_512x384")

print()
print("=" * 60)
print("唯一有 motor_on 欄位的檔案：distance_512x384_uart.csv (9/9)")
print("=" * 60)

rows = load("distance_512x384_uart.csv")
t = col(rows, "time")
bearing = col(rows, "bearing_deg")
smooth_bearing = col(rows, "smooth_bearing_deg")
cx = col(rows, "cx")
dist = col(rows, "dist_cm")
motor_on = [int(float(r["motor_on"])) for r in rows if r.get("motor_on", "") != ""]

print(f"總幀數: {len(rows)}, 時間範圍: {t[0]:.2f}s ~ {t[-1]:.2f}s (共 {t[-1]-t[0]:.2f} 秒)")
print(f"bearing_deg: min={min(bearing):.2f} max={max(bearing):.2f} mean={st.mean(bearing):.2f}")
print(f"dist_cm: min={min(dist):.2f} max={max(dist):.2f} mean={st.mean(dist):.2f}")

# 計算 bearing_deg 正負號翻轉次數 (概略估計「轉向修正次數/震盪次數」)
sign_changes = 0
for i in range(1, len(smooth_bearing)):
    if smooth_bearing[i-1] == 0:
        continue
    if (smooth_bearing[i] > 0) != (smooth_bearing[i-1] > 0):
        sign_changes += 1
print(f"smooth_bearing_deg 正負翻轉次數（概略震盪指標）: {sign_changes} 次")

# motor_on 從 0->1 的次數 (代表車子偵測到球、開始動作的次數)
motor_transitions = sum(1 for i in range(1, len(motor_on)) if motor_on[i] == 1 and motor_on[i-1] == 0)
print(f"motor_on 從關到開的次數: {motor_transitions}")

fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

axes[0].plot(t, bearing, label="bearing_deg (原始)", alpha=0.4, color="gray")
axes[0].plot(t, smooth_bearing, label="smooth_bearing_deg (平滑後)", color="tab:blue")
axes[0].axhline(0, color="black", linewidth=0.8, linestyle="--")
axes[0].set_ylabel("角度 (deg)")
axes[0].legend(loc="upper right")
axes[0].set_title("球的角度 vs 時間 (2026-09-09, distance_512x384_uart.csv)\n離散轉向：正值=球在右側，負值=球在左側，理想上應該收斂到0附近")

axes[1].plot(t, cx, color="tab:orange")
axes[1].set_ylabel("cx (畫面X座標, px)")
axes[1].axhline(256, color="black", linewidth=0.8, linestyle="--", label="畫面中心(256px)")
axes[1].legend(loc="upper right")

axes[2].plot(t, dist, color="tab:green", label="dist_cm")
ax2 = axes[2].twinx()
t_motor = [r["time"] for r in rows if r.get("motor_on", "") != ""]
t_motor = [float(x) for x in t_motor]
ax2.step(t_motor, motor_on, color="tab:red", where="post", alpha=0.5, label="motor_on")
ax2.set_ylabel("motor_on (0/1)", color="tab:red")
ax2.set_ylim(-0.1, 1.3)
axes[2].set_ylabel("距離 (cm)")
axes[2].set_xlabel("時間 (秒)")
axes[2].legend(loc="upper left")

plt.tight_layout()
plt.savefig("discrete_baseline_20260909.png", dpi=130)
print("圖存成 discrete_baseline_20260909.png")
