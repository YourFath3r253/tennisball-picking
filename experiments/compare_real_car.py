"""實體車模式的模擬 (run_real_car.sh) vs 實體車 10/11 實測：同一套指標並排比較。

用法: python3 experiments/compare_real_car.py <runN> [<runN> ...]
實測資料：實體車子程式碼/jetson_data_20261012/models (10/11 yellow_* 跟 distance_512x384_uart，D 程式現行版本)
比較：
  1. BALL 送出間隔 (D 的 ball CSV 相鄰兩列)
  2. COURT 處理間隔 (court CSV 相鄰兩列)
  3. 送給 STM32 的 EDGE (照 EdgeUartGate 重建) 當下的 near_y
  4. 模擬：畫面年齡 (相機緩衝區)；轉向：TurnPrecise 的編碼器角度 vs Gazebo 真實轉角、花多久
  5. 撿球數、EDGE 次數
"""
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
REAL = ROOT / '實體車子程式碼' / 'jetson_data_20261012' / 'models'
REAL_TESTS = ['yellow_20261011_185514', 'yellow_20261011_185857', 'yellow_v2_20261011_192546',
              'yellow_20261011_191346', 'distance_512x384_uart']


def read(path):
    if not Path(path).exists():
        return []
    with open(path, errors='ignore') as f:
        return [r for r in csv.DictReader(line.replace('\0', '') for line in f)]


def q(x):
    x = np.sort(np.asarray(x, dtype=float))
    if len(x) == 0:
        return '  (無)'
    return f'n={len(x):4d}  p10 {x[len(x) // 10]:.3f}  中位 {x[len(x) // 2]:.3f}  p90 {x[(9 * len(x)) // 10]:.3f}'


def intervals(rows, key):
    t = np.array([float(r[key]) for r in rows if r.get(key)])
    d = np.diff(t)
    return d[(d > 0) & (d < 1.0)]


def sent_edges(court):
    """照 D 的 EdgeUartGate (冷卻 0.5 s、連續 SAFE 0.5 s 才重新上膛) 重建真的送出去的 EDGE。"""
    armed, last, clear_since, out = True, None, None, []
    for r in court:
        if not r.get('time_s'):
            continue
        t = float(r['time_s'])
        if r['state'] == 'SAFE':
            if clear_since is None:
                clear_since = t
            if not armed and t - last >= 0.5 and t - clear_since >= 0.5:
                armed = True
        elif r['state'] == 'EDGE':
            clear_since = None
            if armed:
                armed = False
                last = t
                out.append(r)
    return out


def real_stats():
    ball, court = [], []
    for n in REAL_TESTS:
        ball.append(read(REAL / f'{n}.csv'))
        court.append(read(REAL / f'{n}_court.csv'))
    bdt = np.concatenate([intervals(b, 'time') for b in ball if b])
    cdt = np.concatenate([intervals(c, 'time_s') for c in court if c])
    edges = [e for c in court for e in sent_edges(c)]
    ny = [float(e['near_y_norm']) for e in edges if e.get('near_y_norm')]
    return bdt, cdt, ny, edges


def turn_stats(run):
    ev = read(run / 'stm32_events.csv')
    traj = read(run / 'real_traj.csv')
    if not traj:
        return []
    t = np.array([float(r['sim_t']) for r in traj])
    yaw = np.unwrap(np.radians([float(r['real_yaw_deg']) for r in traj]))
    out, start = [], None
    for e in ev:
        if e['event'] == 'TURN_START':
            start = (float(e['sim_t']), float(e['detail']))
        elif e['event'] == 'TURN_DONE' and start is not None:
            t0, cmd = start
            t1 = float(e['sim_t'])
            if t0 >= t[0] and t1 <= t[-1]:
                real = np.degrees(np.interp(t1, t, yaw) - np.interp(t0, t, yaw))
                enc = float(e['detail'].split('deg')[0])
                out.append((cmd, enc, real, t1 - t0))
            start = None
    return out


def main():
    bdt, cdt, ny, edges = real_stats()
    print('=== 實體車 10/11 (D 程式現行版，教室木地板 + 黃毛線) ===')
    print(f'BALL 間隔      {q(bdt)}')
    print(f'COURT 間隔     {q(cdt)}')
    print(f'送出 EDGE 時 near_y {q(ny)}   (共 {len(edges)} 次；L/C/R = '
          f'{sum(e["line_side"] == "LEFT" for e in edges)}/{sum(e["line_side"] == "CENTER" for e in edges)}/'
          f'{sum(e["line_side"] == "RIGHT" for e in edges)})')
    for name in sys.argv[1:]:
        run = ROOT / 'experiments' / '實驗數據' / name
        meta = (run / 'meta.txt').read_text() if (run / 'meta.txt').exists() else ''
        world = next((l.split('=', 1)[1] for l in meta.splitlines() if l.startswith('world=')), '?')
        res = (run / 'result.txt').read_text().replace('\n', ' ') if (run / 'result.txt').exists() else ''
        b, c = read(run / 'sim.csv'), read(run / 'sim_court.csv')
        se = sent_edges(c)
        sny = [float(e['near_y_norm']) for e in se if e.get('near_y_norm')]
        print(f'\n=== 模擬 {name} ({world}) {res}')
        print(f'BALL 間隔      {q(intervals(b, "time"))}')
        print(f'COURT 間隔     {q(intervals(c, "time_s"))}')
        print(f'送出 EDGE 時 near_y {q(sny)}   (共 {len(se)} 次；L/C/R = '
              f'{sum(e["line_side"] == "LEFT" for e in se)}/{sum(e["line_side"] == "CENTER" for e in se)}/'
              f'{sum(e["line_side"] == "RIGHT" for e in se)})')
        print(f'畫面年齡 (相機緩衝區模型) {q([float(r["frame_age"]) for r in c if r.get("frame_age")])}')
        ts = turn_stats(run)
        if ts:
            print('轉向 (TurnPrecise)：指令 / 編碼器量到 / Gazebo 真實 / 花費秒數')
            for cmd, enc, real, dur in ts[:12]:
                print(f'   {cmd:+5.0f}°  {enc:+7.1f}°  {real:+7.1f}°  {dur:5.2f}s')
            ratio = [abs(r / e) for _, e, r, _ in ts if abs(e) > 30]
            print(f'   真實/編碼器 比值 {q(ratio)}；花費 {q([d for *_, d in ts])}')


if __name__ == '__main__':
    main()
