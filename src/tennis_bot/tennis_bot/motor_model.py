"""底盤馬達模型：實體車 STM32 輪速 PID (main.c 1:1 移植) + 直流減速馬達。

原本模擬是「下 40 rpm 就瞬間 40 rpm」(diff_drive 直接設輪子速度)，實體車不是這樣：
STM32 每 10 ms 讀一次編碼器 (11 PPR x 18.8 減速 x 4 倍頻 = 827.2 count/圈，10 ms 一個 count
就是 7.25 rpm，低速量起來是一格一格的)、低通濾波 (0.8/0.2)、PID (Kp=5, Ki=1, Kd=0.01，
有「動態積分分離」：誤差大於 max(30, 0.5*目標) 就把積分清掉)、PWM 限幅 ±1000，再推馬達。

這裡分成兩層 (都是純 Python、沒有 ROS，模擬節點跟離線擬合程式共用)：
  WheelMotor        一個輪子：STM32 PID 迴圈 + 馬達 (一階延遲 + 靜摩擦死區)
  ChassisMotorModel 左右兩輪 + 實體車的差速運動學 (輪半徑 0.052 m、輪距 0.243 m，取自 main.c)，
                    輸出車體 (v, ω)。用實體車的幾何算，模擬車的輪子尺寸不同也沒關係，
                    同樣的 rpm 指令在模擬跟實體車上車體動得一樣。

馬達參數來源：
  PWM 1000 -> 550 rpm —— 9/20 方向接反的「暴衝」紀錄 (rpm_log_standalone.csv、..._power_on.csv)：
                 PID 反向回授一路飽和到 PWM 1000，兩輪穩定在 -545~-555 rpm。
  其他 (死區、時間常數、轉向效率) 由 experiments/fit_motor_model.py 用實體車原地轉向 CSV 擬合，見 DEFAULT_PARAMS。
"""
import math

ENCODER_RESOLUTION = 11.0 * 18.8 * 4.0   # main.c ENCODER_RESOLUTION
STM32_DT = 0.01                          # TIM6 10 ms
WHEEL_RADIUS_REAL = 0.052                # main.c WHEEL_RADIUS
WHEEL_TRACK_REAL = 0.243                 # main.c WHEEL_TRACK

# 擬合結果 (experiments/fit_motor_model.py，2026-10-05)：用 Kp=0.3/0.6/1.5 三組實體車原地轉向數據一起擬合，
# 詳細見 experiments/實驗數據/motor_model_fit.md。
# 限制條件：9/21 接好線後「固定 ±40 rpm 能正常自轉」。main.c 積分分離在誤差 > 30 時清積分，起步只剩 P 項，
# 要能從 0 轉到誤差 < 30 (轉速 > 10 rpm) 開始積分，死區必須 <= ~134 PWM，所以搜尋上限 130。
DEFAULT_PARAMS = {
    'k_rpm_per_pwm': 550.0 / (1000.0 - 130.0),  # 由「PWM 1000 時 550 rpm」(暴衝紀錄) 跟死區推得 = 0.63
    'dead_pwm': 130.0,       # 擬合 (上限 134，見上)
    'static_pwm': 130.0,     # 靜摩擦取同值 (加大沒有比較好)
    'tau': 0.35,             # 擬合：馬達+車體慣量的一階時間常數 (s)
    'yaw_efficiency': 0.70,  # 擬合：原地轉時實際轉速 / 運動學算的轉速 (輪胎側滑、萬向輪拖行)
}


class WheelMotor:
    def __init__(self, k_rpm_per_pwm=0.55, dead_pwm=300.0, tau=0.10, static_pwm=None, substeps=10):
        self.k = k_rpm_per_pwm
        self.dead = dead_pwm
        self.static = dead_pwm if static_pwm is None else max(static_pwm, dead_pwm)
        self.tau = tau
        self.substeps = substeps
        # STM32 PID 狀態 (PID_Controller)
        self.kp, self.ki, self.kd = 5.0, 1.0, 0.01
        self.out_max = 1000.0
        self.target_rpm = 0.0
        self.integral = 0.0
        self.error_last = 0.0
        self.rpm_filter = 0.0
        self.pwm = 0.0
        # 物理狀態
        self.rpm = 0.0          # 輪子真實轉速
        self._count_acc = 0.0   # 編碼器累積的不足 1 count 的部分

    def stop(self):
        """Chassis_Stop()：目標歸零 + 清積分/微分歷史。"""
        self.target_rpm = 0.0
        self.integral = 0.0
        self.error_last = 0.0

    def _pid_calc(self, current_rpm):
        """main.c PID_Calc() 逐行對照。"""
        error = self.target_rpm - current_rpm
        dynamic_limit = abs(self.target_rpm * 0.50)
        if dynamic_limit < 30.0:
            dynamic_limit = 30.0
        if -dynamic_limit < error < dynamic_limit:
            self.integral += error
            self.integral = max(-8000.0, min(8000.0, self.integral))
        else:
            self.integral = 0.0
        derivative = error - self.error_last
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.error_last = error
        return max(-self.out_max, min(self.out_max, out))

    def step(self):
        """推進 10 ms (一個 TIM6 中斷)。"""
        # 1) 編碼器：這 10 ms 轉過的 count (M 法測速，整數，不足的留到下次)
        self._count_acc += self.rpm / 60.0 * ENCODER_RESOLUTION * STM32_DT
        counts = math.trunc(self._count_acc)
        self._count_acc -= counts
        rpm_meas = counts / ENCODER_RESOLUTION * 6000.0
        # 2) 低通濾波 + PID
        self.rpm_filter = 0.8 * self.rpm_filter + 0.2 * rpm_meas
        self.pwm = self._pid_calc(self.rpm_filter)
        # 3) 馬達：PWM 扣掉靜摩擦死區後一階逼近 K*PWM；靜止且 PWM 不夠大就推不動
        h = STM32_DT / self.substeps
        for _ in range(self.substeps):
            if abs(self.rpm) < 0.5 and abs(self.pwm) <= self.static:
                self.rpm = 0.0
                continue
            drive = math.copysign(max(0.0, abs(self.pwm) - self.dead), self.pwm)
            self.rpm += (self.k * drive - self.rpm) * h / self.tau


class ChassisMotorModel:
    def __init__(self, params=None):
        p = dict(DEFAULT_PARAMS)
        if params:
            p.update(params)
        self.left = WheelMotor(p['k_rpm_per_pwm'], p['dead_pwm'], p['tau'], p['static_pwm'])
        self.right = WheelMotor(p['k_rpm_per_pwm'], p['dead_pwm'], p['tau'], p['static_pwm'])
        self.yaw_efficiency = p['yaw_efficiency']

    def set_targets(self, left_rpm, right_rpm):
        """Chassis_SteerP()：只改目標，不清積分。兩輪都 0 時當作 Chassis_Stop() (清積分)。"""
        if left_rpm == 0.0 and right_rpm == 0.0:
            self.left.stop()
            self.right.stop()
        else:
            self.left.target_rpm = left_rpm
            self.right.target_rpm = right_rpm

    def step(self):
        self.left.step()
        self.right.step()

    def body_velocity(self):
        """實體車幾何的車體速度 (v [m/s], ω [rad/s]，逆時針為正)。"""
        wl = self.left.rpm * 2.0 * math.pi / 60.0
        wr = self.right.rpm * 2.0 * math.pi / 60.0
        v = WHEEL_RADIUS_REAL * (wl + wr) / 2.0
        w = WHEEL_RADIUS_REAL * (wr - wl) / WHEEL_TRACK_REAL * self.yaw_efficiency
        return v, w


def body_to_wheel_rpm(v, w):
    """車體 (v, ω) -> 實體車左右輪目標 rpm (控制節點用，等於實體車上要送給 STM32 的轉速)。"""
    wl = (v - w * WHEEL_TRACK_REAL / 2.0) / WHEEL_RADIUS_REAL
    wr = (v + w * WHEEL_TRACK_REAL / 2.0) / WHEEL_RADIUS_REAL
    return wl * 60.0 / (2.0 * math.pi), wr * 60.0 / (2.0 * math.pi)


class TargetRamp:
    """目標轉速的斜坡 (照 B 同學 main.c 的 Chassis_Forward 軟起步：從 15 rpm 起跳，之後每次指令最多爬 8 rpm)。

    為什麼需要：main.c 的 PID 有「誤差 > max(30, 0.5*目標) 就清積分」的積分分離，從靜止直接給 55 rpm 這種目標，
    起步只有 P 項，馬達模型預測會卡在 ~22 rpm、誤差一直 > 30、積分永遠不累積 (模型預測，未實測)。
    B 同學在直走用斜坡避開了這個問題；控制節點每 0.1 s 送一次指令，所以這裡是每 0.1 s 最多 ±8 rpm。
    """

    def __init__(self, start_rpm=15.0, step_rpm=8.0):
        self.start = start_rpm
        self.step_rpm = step_rpm
        self.current = 0.0

    def update(self, target):
        if target == 0.0:
            self.current = 0.0
            return 0.0
        if self.current == 0.0 or (self.current > 0) != (target > 0):
            # 從靜止 (或反向) 起步：直接跳到 ±15 rpm (目標更小就用目標)
            self.current = math.copysign(min(abs(target), self.start), target)
            return self.current
        if abs(target) > abs(self.current):
            self.current = math.copysign(min(abs(target), abs(self.current) + self.step_rpm), target)
        else:
            self.current = target
        return self.current
