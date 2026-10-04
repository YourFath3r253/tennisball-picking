"""實體車 vs 模擬 原地轉向步階響應比較圖 (模擬真實化 R5 驗證)。
左：實體車 Kp=0.3 (run14) / 0.6 (run15) / 1.5 (run12) 各取一段，對齊到「開始轉」的時間
右：模擬，同等增益 (K = 0.00539*Kp px->rad/s)，理想相機 vs 真實相機模型 (FRAME_AGE 0.45 s)
"""
import csv
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
REAL = '/home/sean/ros2_ws/實體車子程式碼/實驗數據/'
SP = '/home/sean/ros2_ws/experiments/實驗數據/step_response_sim/'
OUT = sys.argv[1] if len(sys.argv) > 1 else '/home/sean/ros2_ws/experiments/實驗數據/step_sim_vs_real.png'


def real_segment(path, t0, t1):
    rows = [r for r in csv.DictReader(open(path)) if t0 <= float(r['time']) <= t1]
    # 對齊到第一次角度明顯變化 (開始轉) 前 0.5 s
    b0 = float(rows[0]['bearing_deg'])
    start = next(float(r['time']) for r in rows if abs(float(r['bearing_deg']) - b0) > 1.5) - 0.5
    return [(float(r['time']) - start, float(r['bearing_deg'])) for r in rows if float(r['time']) >= start]


def sim_series(path):
    rows = [r for r in csv.DictReader(open(path)) if r['new_vision'] == '1' and r['measured_bearing_deg']]
    return [(float(r['t']), float(r['measured_bearing_deg'])) for r in rows]


fig, (a, b) = plt.subplots(1, 2, figsize=(13, 4.8), sharey=True)
for label, path, t0, t1, c in [('實體 Kp=0.3 (run14)', REAL + 'P=0.3重測/run14.csv', 153, 172, 'tab:green'),
                               ('實體 Kp=0.6 (run15)', REAL + 'P=0.6/run15.csv', 25, 40, 'tab:orange'),
                               ('實體 Kp=1.5 (run12)', REAL + 'P=1.5發散/run12.csv', 149, 164, 'tab:red')]:
    try:
        seg = real_segment(path, t0, t1)
        seg = [(t, -v) if seg[0][1] < 0 else (t, v) for t, v in seg]  # 都翻成從正角度開始
        a.plot([t for t, _ in seg], [v for _, v in seg], '.-', color=c, label=label, ms=3, lw=1)
    except StopIteration:
        pass
a.axhspan(-4, 4, color='gray', alpha=0.15)
a.axhline(0, color='k', lw=0.6)
a.set_title('實體車 (視覺 CSV，球的角度)')
a.set_xlabel('時間 (s)')
a.set_ylabel('球偏離畫面中心角度 (°)')
a.set_xlim(0, 12)
a.legend(fontsize=8)

for label, f, c, ls in [('模擬 理想相機 Kp=0.6 等效', 'stepK_ideal_0.0032.csv', 'tab:blue', '--'),
                        ('模擬 真實相機 Kp=0.3 等效', 'stepK_0.0016.csv', 'tab:green', '-'),
                        ('模擬 真實相機 Kp=0.6 等效', 'stepK_0.0032.csv', 'tab:orange', '-'),
                        ('模擬 真實相機 Kp=1.5 等效', 'stepK_0.0081.csv', 'tab:red', '-')]:
    try:
        s = sim_series(SP + f)
    except FileNotFoundError:
        continue
    b.plot([t + 0.5 for t, _ in s], [v for _, v in s], '.' + ls, color=c, label=label, ms=3, lw=1)
b.axhspan(-4, 4, color='gray', alpha=0.15)
b.axhline(0, color='k', lw=0.6)
b.set_title('模擬 (相機延遲模型：4.4Hz + 畫面延遲 0.45 s)')
b.set_xlabel('時間 (s)')
b.set_xlim(0, 12)
b.legend(fontsize=8)
plt.tight_layout()
plt.savefig(OUT, dpi=130)
print('saved', OUT)
