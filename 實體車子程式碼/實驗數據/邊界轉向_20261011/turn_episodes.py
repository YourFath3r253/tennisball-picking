"""把每次「送給 STM32 的 EDGE」配上影片量到的車頭角度變化。
EDGE 送出時刻照 D 的 EdgeUartGate (冷卻 0.5 s、連續 SAFE 0.5 s 才重新上膛) 從 court.csv 重建。
"""
import csv, sys
import numpy as np

def sent_edges(court):
    armed, last_edge, clear_since, out = True, None, None, []
    for r in court:
        t = float(r['time_s']); st = r['state']
        if st == 'SAFE':
            if clear_since is None: clear_since = t
            if not armed and t - last_edge >= 0.5 and t - clear_since >= 0.5: armed = True
        elif st == 'EDGE':
            clear_since = None
            if armed:
                armed = False; last_edge = t; out.append((t, r['line_side']))
    return out

def main(name):
    Y = list(csv.DictReader(open(name + "_yaw.csv")))
    C = list(csv.DictReader(open(name + "_court.csv")))
    t = np.array([float(r['time_s']) for r in Y]); yaw = np.array([float(r['yaw_deg']) for r in Y])
    fail = np.array([r['dyaw_deg'] == '' for r in Y])
    rate = np.gradient(yaw, t)
    E = sent_edges(C)
    print("### %s: %d EDGE sent" % (name, len(E)))
    print(" #   t_edge side    pre-yaw  rot_start(+s) rot_dur  net_turn  mean°/s  fails  substeps(deg)")
    for k, (te, side) in enumerate(E):
        tn = E[k + 1][0] if k + 1 < len(E) else t[-1]
        y0 = float(np.interp(te, t, yaw))
        w = (t > te) & (t < tn)
        tt, yy, rr = t[w], yaw[w], rate[w]
        mov = np.abs(rr) > 15
        if not mov.any():
            print("%2d %7.1f %-6s %8.1f   (no rotation before next EDGE @%.1f)" % (k, te, side, y0, tn)); continue
        i0 = np.argmax(mov); i1 = len(mov) - 1 - np.argmax(mov[::-1])
        # 子段：轉動中間停超過 0.25 s 就切
        subs, cur = [], [i0]
        for i in range(i0 + 1, i1 + 1):
            if mov[i]:
                if tt[i] - tt[cur[-1]] > 0.25: subs.append(cur); cur = [i]
                else: cur.append(i)
        subs.append(cur)
        sub_deg = [round(float(yy[s[-1]] - (yy[s[0] - 1] if s[0] > 0 else y0)), 0) for s in subs]
        net = float(yy[i1] - y0); dur = tt[i1] - tt[i0]
        nf = int(fail[w][i0:i1 + 1].sum())
        print("%2d %7.1f %-6s %8.1f   %+6.2f      %5.2f   %7.1f  %7.1f  %4d  %s" % (k, te, side, y0, tt[i0] - te, dur, net, net / max(dur, 1e-3), nf, sub_deg))

if __name__ == "__main__":
    for n in sys.argv[1:]: main(n)
