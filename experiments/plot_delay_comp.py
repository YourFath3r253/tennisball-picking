"""相機延遲補償的 Gazebo 步階響應比較圖 (experiments/實驗數據/step_response_sim/comp_*.csv、robust_*.csv)。
用法: python3 experiments/plot_delay_comp.py  -> experiments/實驗數據/delay_comp_step.png
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
D = Path(__file__).resolve().parent / '實驗數據' / 'step_response_sim'


def series(name):
    rows = [r for r in csv.DictReader(open(D / name)) if r['new_vision'] == '1' and r['raw_bearing_deg']]
    return [float(r['t']) for r in rows], [float(r['raw_bearing_deg']) for r in rows]


fig, axes = plt.subplots(1, 2, figsize=(13, 4.6), sharey=True)
ax = axes[0]
for f, label, c in [('comp_kp0.6_off_D0.8.csv', '無補償 Kp=0.6 (現在實體車用的)', 'tab:orange'),
                    ('comp_kp1.5_off_D0.8.csv', '無補償 Kp=1.5', 'tab:red'),
                    ('comp_kp1.5_gyro_D0.8.csv', '延遲補償 (陀螺儀, D=0.8 s) Kp=1.5', 'tab:blue'),
                    ('comp_kp1.5_encoder_D0.8.csv', '延遲補償 (編碼器, D=0.8 s) Kp=1.5', 'tab:green')]:
    t, b = series(f)
    ax.plot(t, b, '.-', ms=3, lw=1.2, color=c, label=label)
ax.axhspan(-4, 4, color='gray', alpha=0.15)
ax.axhline(0, color='k', lw=0.6)
ax.set_title('Gazebo 原地轉向步階響應 (畫面延遲 0.60 s，實體車資料流)')
ax.set_xlabel('時間 (s)')
ax.set_ylabel('球偏離畫面中心角度 (°，右為正)')
ax.set_xlim(0, 10)
ax.legend(fontsize=8)

ax = axes[1]
for f, label, c, ls in [('robust_fa0.45_kp0.6_off.csv', '無補償 Kp=0.6，實際延遲 0.45 s', 'tab:orange', ':'),
                        ('robust_fa0.78_kp0.6_off.csv', '無補償 Kp=0.6，實際延遲 0.78 s', 'tab:orange', '-'),
                        ('robust_fa0.45_kp1.5_gyro.csv', '補償 (假設 D=0.8) Kp=1.5，實際延遲 0.45 s', 'tab:blue', ':'),
                        ('robust_fa0.78_kp1.5_gyro.csv', '補償 (假設 D=0.8) Kp=1.5，實際延遲 0.78 s', 'tab:blue', '-')]:
    t, b = series(f)
    ax.plot(t, b, '.' + ls, ms=3, lw=1.2, color=c, label=label)
ax.axhspan(-4, 4, color='gray', alpha=0.15)
ax.axhline(0, color='k', lw=0.6)
ax.set_title('強健性：實際延遲跟假設不一樣時')
ax.set_xlabel('時間 (s)')
ax.set_xlim(0, 10)
ax.legend(fontsize=8)
plt.tight_layout()
out = Path(__file__).resolve().parent / '實驗數據' / 'delay_comp_step.png'
plt.savefig(out, dpi=130)
print('saved', out)
