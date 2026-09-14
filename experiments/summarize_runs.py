"""整理批次實驗數據：每個 manifest 裡的 run 各算 撿球數/總球數、撿滿時間、平均每顆球間隔、
里程計誤差 (平均/最大/最後)、航向誤差，並依球數做平均。
用法: python3 experiments/summarize_runs.py <manifest.csv> [<manifest.csv> ...] [--out summary.csv]
manifest 格式 (run_random_batch.sh 產生): seed,run_dir,result,layout_json
"""
import csv
import json
import math
import re
import sys
from pathlib import Path

DATA = Path(__file__).resolve().parent / '實驗數據'


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def run_stats(run_dir, n_balls):
    d = DATA / run_dir
    touches = list(csv.DictReader(open(d / 'ball_touches.csv')))
    rows = [r for r in csv.DictReader(open(d / 'trajectory.csv')) if r.get('real_x')]
    picked = len(touches)
    total_t = float(rows[-1]['t']) if rows else float('nan')
    times = [float(t['elapsed_sec']) for t in touches]
    pick_done_t = times[-1] if times else float('nan')
    mean_interval = pick_done_t / picked if picked else float('nan')
    errs = [math.hypot(float(r['x']) - float(r['real_x']), float(r['y']) - float(r['real_y'])) for r in rows]
    yaws = [abs(math.degrees(wrap(float(r['yaw']) - float(r['real_yaw'])))) for r in rows]
    path = sum(math.hypot(float(rows[i]['real_x']) - float(rows[i - 1]['real_x']),
                          float(rows[i]['real_y']) - float(rows[i - 1]['real_y'])) for i in range(1, len(rows)))
    return {
        'run': run_dir, 'balls': n_balls, 'picked': picked, 'success': int(picked == n_balls),
        'time_all_picked_s': round(pick_done_t, 1), 'run_time_s': round(total_t, 1),
        'sec_per_ball': round(mean_interval, 1), 'path_m': round(path, 1),
        'pos_err_mean_m': round(sum(errs) / len(errs), 3), 'pos_err_max_m': round(max(errs), 3),
        'pos_err_final_m': round(errs[-1], 3), 'yaw_err_mean_deg': round(sum(yaws) / len(yaws), 2),
        'yaw_err_max_deg': round(max(yaws), 2),
    }


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = None
    if '--out' in sys.argv:
        out = sys.argv[sys.argv.index('--out') + 1]
        args = [a for a in args if a != out]
    stats = []
    for mf in args:
        for row in csv.DictReader(open(mf)):
            if not row['run_dir'].startswith('run'):
                continue
            layout = json.load(open(row['layout_json']))
            s = run_stats(row['run_dir'], len(layout))
            s['seed'] = row['seed']; s['result'] = re.sub(r'：.*', '', row['result'])
            stats.append(s)
    keys = ['balls', 'seed', 'run', 'result', 'picked', 'success', 'time_all_picked_s', 'run_time_s',
            'sec_per_ball', 'path_m', 'pos_err_mean_m', 'pos_err_max_m', 'pos_err_final_m',
            'yaw_err_mean_deg', 'yaw_err_max_deg']
    print(','.join(keys))
    for s in stats:
        print(','.join(str(s[k]) for k in keys))
    print('\n=== 依球數平均 ===')
    print('balls,runs,success_rate,mean_time_all_picked_s,mean_sec_per_ball,mean_path_m,mean_pos_err_m,mean_pos_err_final_m,mean_yaw_err_deg,max_pos_err_m')
    for n in sorted({s['balls'] for s in stats}):
        g = [s for s in stats if s['balls'] == n]
        ok = [s for s in g if s['success']]
        avg = lambda k, grp=g: sum(s[k] for s in grp) / len(grp)
        print(f"{n},{len(g)},{len(ok)}/{len(g)},{avg('time_all_picked_s', ok) if ok else float('nan'):.1f},"
              f"{avg('sec_per_ball', ok) if ok else float('nan'):.1f},{avg('path_m'):.1f},{avg('pos_err_mean_m'):.3f},"
              f"{avg('pos_err_final_m'):.3f},{avg('yaw_err_mean_deg'):.2f},{max(s['pos_err_max_m'] for s in g):.3f}")
    if out:
        with open(out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys); w.writeheader()
            for s in stats:
                w.writerow({k: s[k] for k in keys})
        print(f'saved {out}')


if __name__ == '__main__':
    main()
