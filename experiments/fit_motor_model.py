"""擬合底盤馬達模型 (tennis_bot/motor_model.py) 的未知參數：用實體車原地轉向 P 控制的 CSV。

離線閉迴路模擬 (不開 Gazebo，幾秒鐘就能跑一組參數)：
  視覺：輸出間隔 0.11/0.11/0.45 s 循環 (實測)，每次輸出的是 FRAME_AGE 秒前的畫面角度，
        再經過 D 同學程式的 EMA (alpha=0.35) 才送給 STM32 (跟實體車一樣)
  STM32：收到 BALL 就 y = Kp*(0-angle)，左右輪 -y/2、+y/2 (飽和 ±45)，輪速 PID 照 main.c
  馬達：motor_model.WheelMotor；車體轉速用實體車幾何 (r 0.052, L 0.243)

要對上的實體車數據 (實體車子程式碼/實驗數據，9/21)：
  Kp=0.3  開始轉 -> 進入 ±4° 約 5.3~7.6 s，沒有 overshoot (run14、記憶裡 run13/14 平均 7.4~7.6 s)
  Kp=0.6  約 2.0~3.3 s，overshoot 1.4~7.9° (run15 兩次)
  Kp=1.5  一直擺盪，擺幅 18~30°，等效延遲 (overshoot/過零角速度) 0.53~0.92 s 平均 0.70 s (run12)
未知參數：馬達靜摩擦死區 dead_pwm、時間常數 tau、畫面延遲 frame_age。
限制：PWM 1000 時轉速 = 550 rpm (暴衝紀錄)，所以 K = 550 / (1000 - dead)。

用法: python3 experiments/fit_motor_model.py
"""
import itertools
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src' / 'tennis_bot' / 'tennis_bot'))
from motor_model import ChassisMotorModel, STM32_DT  # noqa: E402

INTERVALS = (0.11, 0.11, 0.45)
EMA_ALPHA = 0.35
STEER_WHEEL_MAX = 45.0
FULL_PWM_RPM = 550.0

TARGETS = {0.3: {'rise': (5.3, 7.6), 'over': (0.0, 1.0)},
           0.6: {'rise': (2.0, 3.3), 'over': (1.4, 7.9)},
           1.5: {'tau': (0.53, 0.92), 'amp': (18.0, 30.0)}}


def closed_loop(kp, params, frame_age, b0=29.0, duration=20.0):
    """回傳 (時間列表, 原始量測角度列表)：跟實體車 CSV 一樣，每次視覺輸出一筆。"""
    m = ChassisMotorModel(params)
    yaw_hist = [(0.0, 0.0)]   # (t, 車體轉過的角度 deg，逆時針正)
    yaw = 0.0
    t = 0.0
    next_out = frame_age
    k = 0
    smooth = None
    detected = 0
    out_t, out_b = [], []
    while t < duration:
        if t >= next_out - 1e-9:
            # 量到 frame_age 秒前的角度：球在右邊 (+)，車逆時針轉 (yaw 增加) 球會更偏右
            tc = t - frame_age
            yaw_c = yaw_hist[0][1]
            for tt, yy in yaw_hist:
                if tt <= tc:
                    yaw_c = yy
                else:
                    break
            b = b0 + yaw_c
            smooth = b if smooth is None else EMA_ALPHA * b + (1 - EMA_ALPHA) * smooth
            detected += 1
            out_t.append(t)
            out_b.append(b)
            if detected >= 2:  # BALL_ON_CONFIRM_FRAMES
                y = kp * (0.0 - smooth)
                half = max(-STEER_WHEEL_MAX, min(STEER_WHEEL_MAX, y / 2.0))
                m.set_targets(-half, half)
            next_out = t + INTERVALS[k % 3]
            k += 1
        m.step()
        _, w = m.body_velocity()
        yaw += math.degrees(w) * STM32_DT
        t += STM32_DT
        yaw_hist.append((t, yaw))
        if len(yaw_hist) > 400:
            yaw_hist = yaw_hist[-300:]
    return out_t, out_b


def metrics(ts, bs):
    b0 = bs[0]
    s = 1 if b0 > 0 else -1
    i0 = next((i for i in range(len(bs)) if abs(bs[i] - b0) > 1.5), None)
    if i0 is None:
        return {'rise': None, 'over': 0.0, 'tau': None, 'amp': 0.0}
    ts0 = ts[i0 - 1]
    rise = next((ts[i] - ts0 for i in range(i0, len(bs)) if abs(bs[i]) <= 4.0), None)
    over = max([-s * b for b in bs[i0:]] + [0.0])
    taus, amps = [], []
    for i in range(1, len(bs)):
        if (bs[i - 1] > 0) != (bs[i] > 0):
            j0 = max([j for j in range(i) if ts[j] <= ts[i - 1] - 0.6] or [0])
            speed = (bs[i] - bs[j0]) / (ts[i] - ts[j0]) if ts[i] > ts[j0] else 0
            sg = 1 if bs[i] > 0 else -1
            ext = 0.0
            for j in range(i, len(bs)):
                if ts[j] - ts[i] > 4 or (bs[j] > 0) != (sg > 0):
                    break
                ext = max(ext, abs(bs[j]))
            if abs(speed) > 3:
                taus.append(ext / abs(speed))
                amps.append(ext)
    return {'rise': rise, 'over': over, 'tau': sum(taus) / len(taus) if taus else None,
            'amp': sum(amps) / len(amps) if amps else 0.0}


def dist_to_range(v, lo_hi):
    if v is None:
        return 10.0
    lo, hi = lo_hi
    if lo <= v <= hi:
        return 0.0
    return min(abs(v - lo), abs(v - hi)) / max(1e-6, (hi - lo) / 2 + 0.5)


def evaluate(dead, tau, frame_age, eta=1.0, static_extra=0.0):
    params = {'k_rpm_per_pwm': FULL_PWM_RPM / (1000.0 - dead), 'dead_pwm': dead, 'tau': tau,
              'yaw_efficiency': eta, 'static_pwm': dead + static_extra}
    res = {}
    cost = 0.0
    for kp, tgt in TARGETS.items():
        ts, bs = closed_loop(kp, params, frame_age)
        mt = metrics(ts, bs)
        res[kp] = mt
        for key, rng in tgt.items():
            cost += dist_to_range(mt[key], rng)
    return cost, res


def main():
    best = []
    # 死區上限 190：9/21 接好線後「固定 40 rpm 能正常自轉」，而 main.c 的積分分離在誤差 40 > 30 時把積分清掉，
    # 起步只有 P 項 5*40 = 200 PWM，所以靜摩擦一定小於 200 (不然 40 rpm 會卡住不動)。
    for dead, tau, fa, eta, sx in itertools.product((0, 50, 100, 150, 190),
                                                    (0.05, 0.1, 0.2, 0.35),
                                                    (0.35, 0.45, 0.55),
                                                    (0.5, 0.6, 0.65, 0.7, 0.75, 0.8, 0.9, 1.0),
                                                    (0,)):
        cost, res = evaluate(dead, tau, fa, eta, sx)
        best.append((cost, dead, tau, fa, eta, sx, res))
    best.sort(key=lambda x: x[0])
    fmt = lambda v: '-' if v is None else f'{v:.2f}'
    print('cost  dead  tau   frame_age eta static+ | Kp0.3 rise/over | Kp0.6 rise/over | Kp1.5 tau/amp')
    for cost, dead, tau, fa, eta, sx, res in best[:15]:
        print(f'{cost:5.2f} {dead:4d} {tau:5.2f} {fa:5.2f} {eta:4.2f} {sx:4d} | {fmt(res[0.3]["rise"])}/{fmt(res[0.3]["over"])} | '
              f'{fmt(res[0.6]["rise"])}/{fmt(res[0.6]["over"])} | {fmt(res[1.5]["tau"])}/{fmt(res[1.5]["amp"])}')
    # 對照：理想馬達 (死區 0、很快) 跟原本模擬的延遲 0.45
    cost, res = evaluate(0, 0.01, 0.45)
    print(f'對照 (無死區、tau 0.01、畫面延遲 0.45) cost {cost:.2f}: {res}')


if __name__ == '__main__':
    main()
