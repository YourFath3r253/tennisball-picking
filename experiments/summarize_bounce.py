"""整理 D 同學邊界反彈提案的批次結果：每個角度 x 每個佈局的撿球數、撿完時間 (模擬時間)、轉向次數。
用法: python3 experiments/summarize_bounce.py <manifest.csv> [--halfgrid <halfgrid_manifest.csv>]
manifest: run_bounce_batch.sh 產生 (angle,seed,run_dir,result,layout_json)
halfgrid: run_halfgrid_batch.sh 產生 (layout,run_dir,result)，當作弓字路徑的比較基準
撿完時間：撿滿的 run 用最後一顆球的模擬時間；沒撿滿的 run 標記逾時 (撿到幾顆)。
"""
import csv
import re
import sys
from collections import defaultdict
from pathlib import Path

DATA = Path('/home/sean/ros2_ws/experiments/實驗數據')


def run_info(run_dir):
    d = DATA / run_dir
    res = {}
    if (d / 'result.txt').exists():
        for line in (d / 'result.txt').read_text().split():
            k, v = line.split('=', 1)
            res[k] = v
    touches = list(csv.DictReader(open(d / 'ball_touches.csv')))
    last = float(touches[-1]['sim_elapsed_sec']) if touches else None
    n = int(res.get('num_balls', 0))
    return {'picked': len(touches), 'n': n, 'done': len(touches) == n and n > 0,
            'time': last, 'bounces': int(res['bounce_count']) if 'bounce_count' in res else None,
            'reason': res.get('reason', '?')}


def main():
    args = sys.argv[1:]
    halfgrid = None
    if '--halfgrid' in args:
        halfgrid = args[args.index('--halfgrid') + 1]
        args = [a for a in args if a not in ('--halfgrid', halfgrid)]
    rows = list(csv.DictReader(open(args[0])))
    table = defaultdict(dict)
    seeds = []
    for r in rows:
        if not r['run_dir'].startswith('run'):
            continue
        info = run_info(r['run_dir'])
        info['run'] = r['run_dir']
        table[r['angle']][r['seed']] = info
        if r['seed'] not in seeds:
            seeds.append(r['seed'])
    if halfgrid:
        for r in csv.DictReader(open(halfgrid)):
            if not r['run_dir'].startswith('run'):
                continue
            seed = re.search(r'seed(\d+)', r['layout']).group(1)
            info = run_info(r['run_dir'])
            info['run'] = r['run_dir']
            table['弓字(半場)'][seed] = info

    head = '| 方法 | ' + ' | '.join(f'佈局 {s}' for s in seeds) + ' | 撿滿 | 平均撿完時間 (撿滿的) | 平均轉向次數 |'
    print(head)
    print('|' + '---|' * (len(seeds) + 4))
    for key in table:
        cells, times, done, bounces = [], [], 0, []
        for s in seeds:
            i = table[key].get(s)
            if i is None:
                cells.append('-')
                continue
            if i['done']:
                cells.append(f"{i['time']:.0f} s ({i['run']})")
                times.append(i['time'])
                done += 1
            else:
                cells.append(f"{i['picked']}/{i['n']} {i['reason']} ({i['run']})")
            if i['bounces'] is not None:
                bounces.append(i['bounces'])
        label = key if not key.replace('.', '').isdigit() else f'轉 {key}°'
        mean_t = f'{sum(times) / len(times):.0f} s' if times else '-'
        mean_b = f'{sum(bounces) / len(bounces):.1f}' if bounces else '-'
        print(f'| {label} | ' + ' | '.join(cells) + f' | {done}/{len(seeds)} | {mean_t} | {mean_b} |')


if __name__ == '__main__':
    main()
