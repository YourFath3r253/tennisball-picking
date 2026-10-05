"""相機延遲補償 (簡化版 Smith predictor) 的離線設計：用跟擬合馬達模型同一套閉迴路模擬，先在這裡掃參數，
再到 Gazebo 驗證 (experiments/step_response_stm32.py --comp)。

問題：實體車的球角度是約 0.6 s 前的畫面 (再加 D 同學程式 EMA 平滑)，車子在這段時間已經轉了一些，
STM32 還照舊的角度轉 -> Kp 一大就衝過頭擺盪 (Kp=1.5)。

補償 (實體車 STM32 也做得到，只需要自己的轉角：陀螺儀或編碼器積分的 chassis_angle_deg)：
  收到 BALL,角度 b 的時候，認為這張畫面是 D 秒前拍的：
      目標航向 psi* = psi(現在 - D) - b          (球在右邊為正，逆時針為正)
  之後每 10 ms (STM32 TIM6) 用最新的轉角算誤差，不用等下一筆視覺：
      e = psi* - psi(現在)，  y = Kp * e，左輪 -y/2、右輪 +y/2 (飽和 ±45 rpm)
  沒有補償時就是原本的 y = Kp * (0 - b)，而且只在收到 BALL 時更新。

轉角來源：
  gyro     陀螺儀 = 車體真實轉角
  encoder  編碼器積分 (main.c chassis_angle_deg)：只算輪子運動學，原地轉時輪胎側滑
           (馬達模型 yaw_efficiency=0.70)，會高估轉角 1/0.70 倍
用法: python3 experiments/delay_comp_design.py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src' / 'tennis_bot' / 'tennis_bot'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from motor_model import ChassisMotorModel, STM32_DT, WHEEL_RADIUS_REAL, WHEEL_TRACK_REAL  # noqa: E402
from fit_motor_model import INTERVALS, EMA_ALPHA, STEER_WHEEL_MAX  # noqa: E402

FRAME_AGE = 0.60


def closed_loop(kp, comp=None, d_assumed=0.6, source='gyro', frame_age=FRAME_AGE, b0=29.0, duration=15.0):
    """comp=None 無補償；comp='heading' 用目標航向補償。回傳 (每次視覺輸出的時間, 原始角度)。"""
    m = ChassisMotorModel()
    yaw = 0.0          # 真實轉角 (deg)
    yaw_enc = 0.0      # 編碼器積分轉角 (deg)
    hist = []          # (t, 真實轉角, 量測轉角)
    t = 0.0
    next_out = frame_age
    k = 0
    smooth = None
    detected = 0
    psi_target = None
    out_t, out_b = [], []

    def at(time_, idx):
        val = hist[0][idx] if hist else 0.0
        for row in hist:
            if row[0] <= time_:
                val = row[idx]
            else:
                break
        return val

    while t < duration:
        meas = yaw if source == 'gyro' else yaw_enc
        hist.append((t, yaw, meas))
        if len(hist) > 300:
            hist = hist[-250:]
        if t >= next_out - 1e-9:
            b = b0 + at(t - frame_age, 1)
            smooth = b if smooth is None else EMA_ALPHA * b + (1 - EMA_ALPHA) * smooth
            detected += 1
            out_t.append(t)
            out_b.append(b)
            if detected >= 2:
                if comp == 'heading':
                    psi_target = at(t - d_assumed, 2) - smooth
                else:
                    y = kp * (0.0 - smooth)
                    half = max(-STEER_WHEEL_MAX, min(STEER_WHEEL_MAX, y / 2.0))
                    m.set_targets(-half, half)
            next_out = t + INTERVALS[k % 3]
            k += 1
        if comp == 'heading' and psi_target is not None:
            y = kp * (psi_target - meas)
            half = max(-STEER_WHEEL_MAX, min(STEER_WHEEL_MAX, y / 2.0))
            if half == 0.0:
                half = 1e-6  # 0,0 會被當成 Chassis_Stop (清積分)
            m.set_targets(-half, half)
        m.step()
        _, w = m.body_velocity()
        yaw += math.degrees(w) * STM32_DT
        wl = m.left.rpm * 2 * math.pi / 60.0
        wr = m.right.rpm * 2 * math.pi / 60.0
        yaw_enc += math.degrees(WHEEL_RADIUS_REAL * (wr - wl) / WHEEL_TRACK_REAL) * STM32_DT
        t += STM32_DT
    return out_t, out_b


def step_metrics(ts, bs, band=4.0):
    b0 = bs[0]
    s = 1 if b0 > 0 else -1
    i0 = next((i for i in range(len(bs)) if abs(bs[i] - b0) > 1.5), None)
    if i0 is None:
        return {'rise': None, 'settle': None, 'over': 0.0}
    ts0 = ts[i0 - 1]
    rise = next((ts[i] - ts0 for i in range(i0, len(bs)) if abs(bs[i]) <= band), None)
    last_out = max([i for i in range(i0, len(bs)) if abs(bs[i]) > band] or [i0])
    settle = ts[last_out + 1] - ts0 if last_out + 1 < len(bs) else None
    over = max([-s * b for b in bs[i0:]] + [0.0])
    return {'rise': rise, 'settle': settle, 'over': over}


def fmt(v):
    return '  -  ' if v is None else f'{v:5.2f}'


def main():
    print('== 1) 無補償 vs 補償 (陀螺儀、假設延遲 D)，起始 29°，進入/穩定在 ±4°')
    print('   方法                         Kp   上升(s) 穩定(s) overshoot(°)')
    cases = [('無補償', None, None, 'gyro')]
    cases += [(f'補償 陀螺儀 D={d:.1f}', 'heading', d, 'gyro') for d in (0.6, 0.8, 1.0)]
    cases += [('補償 編碼器 D=0.8', 'heading', 0.8, 'encoder')]
    for label, comp, d, src in cases:
        for kp in (0.3, 0.6, 1.0, 1.5, 2.0, 3.0):
            mt = step_metrics(*closed_loop(kp, comp, d or 0.6, src))
            print(f'   {label:26s} {kp:4.1f}  {fmt(mt["rise"])}  {fmt(mt["settle"])}  {fmt(mt["over"])}')
    print('\n== 2) 強健性：補償 (陀螺儀 D=0.8, Kp=1.5) 在真實延遲跟假設不一樣時')
    for fa in (0.42, 0.5, 0.6, 0.7, 0.78, 0.9):
        mt = step_metrics(*closed_loop(1.5, 'heading', 0.8, 'gyro', frame_age=fa))
        mt0 = step_metrics(*closed_loop(0.6, None, frame_age=fa))
        print(f'   真實畫面延遲 {fa:.2f} s：補償 Kp1.5 穩定 {fmt(mt["settle"])} s overshoot {fmt(mt["over"])}° | '
              f'無補償 Kp0.6 穩定 {fmt(mt0["settle"])} s overshoot {fmt(mt0["over"])}°')


if __name__ == '__main__':
    main()
