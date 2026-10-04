"""每個 run 的指標 (給模擬真實化/邊界提案比較用)：
結束原因、撿到幾顆、撿完(或最後一顆)的模擬時間、里程計誤差、航向誤差、真實路徑長、ALIGN 次數、轉向次數。
用法: python3 run_metrics.py run116 run117 ...
"""
import csv
import math
import sys
from pathlib import Path

DATA = Path('/home/sean/ros2_ws/experiments/實驗數據')


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def metrics(run):
    d = DATA / run
    res = {}
    if (d / 'result.txt').exists():
        for line in (d / 'result.txt').read_text().split():
            k, v = line.split('=', 1)
            res[k] = v
    touches = list(csv.DictReader(open(d / 'ball_touches.csv')))
    sim_t = [float(t['sim_elapsed_sec']) for t in touches if t.get('sim_elapsed_sec')]
    rows = [r for r in csv.DictReader(open(d / 'trajectory.csv')) if r.get('real_x')]
    errs = [math.hypot(float(r['x']) - float(r['real_x']), float(r['y']) - float(r['real_y'])) for r in rows]
    yaws = [abs(math.degrees(wrap(float(r['yaw']) - float(r['real_yaw'])))) for r in rows]
    path = sum(math.hypot(float(rows[i]['real_x']) - float(rows[i - 1]['real_x']),
                          float(rows[i]['real_y']) - float(rows[i - 1]['real_y'])) for i in range(1, len(rows)))
    aligns = sum(1 for i in range(1, len(rows)) if rows[i]['state'] == 'ALIGN' and rows[i - 1]['state'] != 'ALIGN')
    bounces = None
    if (d / 'bounces.csv').exists():
        bounces = len(list(csv.DictReader(open(d / 'bounces.csv'))))
    return {
        'run': run, 'reason': res.get('reason', '?'), 'picked': f"{len(touches)}/{res.get('num_balls', '?')}",
        'last_pick_sim_s': round(sim_t[-1], 1) if sim_t else None, 'run_sim_s': res.get('sim_sec'),
        'wall_s': res.get('wall_sec'), 'path_m': round(path, 1),
        'odo_err_mean_m': round(sum(errs) / len(errs), 3) if errs else None,
        'odo_err_max_m': round(max(errs), 3) if errs else None,
        'odo_err_final_m': round(errs[-1], 3) if errs else None,
        'yaw_err_max_deg': round(max(yaws), 2) if yaws else None,
        'yaw_err_final_deg': round(yaws[-1], 2) if yaws else None,
        'align_entries': aligns, 'bounces': bounces,
    }


if __name__ == '__main__':
    for r in sys.argv[1:]:
        print(metrics(r))
