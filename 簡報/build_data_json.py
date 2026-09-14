"""把 run_data_batch.sh 的三個 manifest 整理成簡報用的 data_0915.json。
用法: python3 簡報/build_data_json.py
"""
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'experiments'))
from summarize_runs import run_stats  # noqa: E402

LAYOUTS = ROOT / 'experiments' / '實驗數據' / 'layouts'
runs = []
for n in (10, 15, 20):
    mf = LAYOUTS / f'batch_data{n}_manifest.csv'
    if not mf.exists():
        continue
    for row in csv.DictReader(open(mf)):
        if not row['run_dir'].startswith('run'):
            continue
        layout = json.load(open(ROOT / row['layout_json']))
        s = run_stats(row['run_dir'], len(layout))
        runs.append({'balls': s['balls'], 'run': s['run'], 'picked': s['picked'], 'success': s['success'],
                     'time': s['time_all_picked_s'] if s['success'] else s['run_time_s'], 'path': s['path_m'],
                     'pos_mean': s['pos_err_mean_m'], 'pos_max': s['pos_err_max_m'], 'pos_final': s['pos_err_final_m'],
                     'yaw_mean': s['yaw_err_mean_deg'], 'yaw_max': s['yaw_err_max_deg'], 'sec_per_ball': s['sec_per_ball']})

groups = []
for n in sorted({r['balls'] for r in runs}):
    g = [r for r in runs if r['balls'] == n]
    ok = [r for r in g if r['success']]
    avg = lambda k, grp: round(sum(r[k] for r in grp) / len(grp), 3) if grp else float('nan')
    groups.append({'balls': n, 'runs': len(g), 'success': len(ok),
                   'time': round(avg('time', ok), 1), 'sec_per_ball': round(avg('sec_per_ball', ok), 1),
                   'path': round(avg('path', g), 1), 'pos_err': avg('pos_mean', g), 'pos_final': avg('pos_final', g),
                   'yaw_err': round(avg('yaw_mean', g), 2)})
out = {'groups': groups, 'runs': runs,
       'note': '撿完時間 = 最後一顆球進車廂的時刻；成功 = 全部撿進後車廂 (結束時用 Gazebo 真實座標稽核)；里程計誤差 = 輪速+陀螺儀推算位置 vs Gazebo 真實位置'}
(Path(__file__).resolve().parent / 'data_0915.json').write_text(json.dumps(out, ensure_ascii=False, indent=1))
print(json.dumps(groups, ensure_ascii=False, indent=1))
