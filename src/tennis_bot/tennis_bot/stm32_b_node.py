"""模擬裡的 STM32：照 B 同學最新的 main.c (2026-10-12「加入邊界判斷版本」) 一行一行移植。

  /jetson_uart (D 同學 Jetson 的 UART 字串) -> 主迴圈 (避線 > 追球 > 巡邏) -> 輪速目標
  -> TIM6 10 ms 中斷 (編碼器、低通、PID、PWM) -> 馬達模型 -> 車體速度 /cmd_vel (Gazebo)

原始碼對照：實體車子程式碼/B_main_加入邊界判斷版本_20261012.c
  - UART 收字：第一個字元只收 B/N/L/P/S/C；收完一行 command_ready=1，主迴圈處理前不收新的一行
  - COURT,<EDGE|SAFE>,<LEFT|RIGHT|CENTER|NONE>,<色>：EDGE 且沒在避線 -> 煞車、500 ms 後 Chassis_TurnPrecise：
      LEFT -> -240° (右轉)、RIGHT -> +240° (左轉)、其他 -> -360°；60 rpm；編碼器角度；±2° 到位才結束 (阻塞)
  - BALL,距離,角度：has_ball=1；距離 <= 25 cm 觸發盲衝 (直走 75 rpm，1 s)
  - BALL_OFF：has_ball=0；距離 <= 30 cm 時補盲衝
  - 追球 (B 同學的「點放等待」)：|角度| > 6° (距離 > 40 cm) / 4° -> 原地轉 0.2 s (55 rpm 若 |角度| > 14°，否則 40 rpm)，
      停到滿 1 s 再看；角度夠小 -> Chassis_Forward(75)；距離 <= 30 cm -> Forward(75)
  - 巡邏：Chassis_Forward(80) (編碼器角度維持直線 ±8 rpm、15 rpm 起步 +8 斜坡)
  - Chassis_SpinLeft/Right 每次呼叫都清 PID 積分；Chassis_Stop 清積分+微分；Forward 從靜止起步才清
主迴圈在實體車上一直跑 (一圈幾微秒)，這裡每 10 ms 中斷前跑 MAIN_PASSES 圈：
  「每圈都呼叫」的函式 (例如 SpinRight 每圈清積分、Forward 每圈爬 8 rpm) 效果跟實體車一樣。

環境變數：
  TENNISBOT_STM32_CHASE=b (預設，B 同學現行點放等待) | p (Sean 的 P 轉向：y=Kp*(0-角度)，每筆 BALL 更新)
  TENNISBOT_STEER_KP (p 模式用，預設 0.6)
  TENNISBOT_STM32_LOG_DIR：寫 stm32.csv (每 50 ms 一筆，同實體車 RPM telemetry 的週期) + stm32_events.csv
"""
import csv
import math
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String

from tennis_bot.motor_model import ChassisMotorModel, ENCODER_RESOLUTION, STM32_DT

WHEEL_RADIUS = 0.052          # main.c
WHEEL_TRACK = 0.243           # main.c
MAIN_PASSES = 12              # 每 10 ms 主迴圈跑幾圈 (>= 10 才能讓 Forward 斜坡一次爬完，跟實體車一樣)
CHASE_MODE = os.environ.get('TENNISBOT_STM32_CHASE', 'b')
STEER_KP = float(os.environ.get('TENNISBOT_STEER_KP', '0.6'))
STEER_WHEEL_MAX = 45.0
LOG_DIR = os.environ.get('TENNISBOT_STM32_LOG_DIR', '')
ALLOWED_FIRST = set('BNLPSC')
MAX_CATCHUP = 50


class Stm32BNode(Node):
    def __init__(self):
        super().__init__('stm32_b_node')
        self.m = ChassisMotorModel()
        self.pl, self.pr = self.m.left, self.m.right     # pid_left / pid_right (WheelMotor 內有 PID 狀態)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rpm_pub = self.create_publisher(Float32MultiArray, '/wheel_rpm', 10)
        self.event_pub = self.create_publisher(String, '/stm32_event', 50)
        self.create_subscription(String, '/jetson_uart', self._uart_cb, 100)
        self.create_subscription(JointState, '/joint_states', self._tick_cb, 50)
        # ---- main.c 全域變數 ----
        self.rx_pending = []          # UART 收到但還沒輪到的整行 (依到達順序)
        self.rx_line = None           # rx_buffer (command_ready=1 時的那一行)
        self.ball_distance_cm = 0.0
        self.ball_angle_deg = 0.0
        self.chassis_angle_deg = 0.0
        self.avoiding = 0
        self.avoid_start_time = 0
        self.has_ball = 0
        self.chassis_motion_state = 0
        self.locked_heading_deg = 0.0
        self.current_ramp_rpm = 0.0
        # main() 裡的區域變數
        self.blind_dash_active = 0
        self.blind_dash_start_time = 0
        self.last_seen_time = 0
        self.current_avoid_angle = -360.0
        # 追球的 static 變數
        self.step_start_time = 0
        self.turn_step_state = 0
        # Chassis_TurnPrecise (阻塞)
        self.turning = False
        self.turn_target = 0.0
        self.turn_start_angle = 0.0
        self.turn_t0 = 0.0
        # 時間
        self.sim_time = None
        self.tick_ms = 0
        self.ticks = 0
        self.state_name = 'BOOT'
        self.log = self.ev = None
        if LOG_DIR:
            Path(LOG_DIR).mkdir(parents=True, exist_ok=True)
            self._files = [open(Path(LOG_DIR) / 'stm32.csv', 'w', newline=''),
                           open(Path(LOG_DIR) / 'stm32_events.csv', 'w', newline='')]
            self.log = csv.writer(self._files[0])
            self.log.writerow(['sim_t', 'state', 'target_left_rpm', 'actual_left_rpm', 'target_right_rpm',
                               'actual_right_rpm', 'pwm_left', 'pwm_right', 'chassis_angle_deg', 'has_ball',
                               'ball_dist_cm', 'ball_angle_deg', 'avoiding', 'blind_dash'])
            self.ev = csv.writer(self._files[1])
            self.ev.writerow(['sim_t', 'event', 'detail', 'chassis_angle_deg'])
        self.get_logger().info(f'STM32 (B 同學 main.c 移植)，追球模式 {CHASE_MODE}'
                               + (f' Kp={STEER_KP}' if CHASE_MODE == 'p' else ''))

    # ---------------- 工具 ----------------
    def HAL_GetTick(self):
        return self.tick_ms

    def _event(self, name, detail=''):
        if self.ev:
            self.ev.writerow([f'{self.sim_time:.3f}', name, detail, f'{self.chassis_angle_deg:.2f}'])
        self.event_pub.publish(String(data=f'{self.sim_time:.3f},{name},{detail}'))

    # ---------------- UART (HAL_UART_RxCpltCallback) ----------------
    def _uart_cb(self, msg):
        line = msg.data.strip()
        if line:
            self.rx_pending.append(line)

    def _uart_isr(self):
        """收字中斷：command_ready==0 才收下一行；第一個字元不合法就丟掉。
        實體車主迴圈一圈只要幾微秒，UART 兩行之間 (115200 bps 一行約 1.5 ms) 一定處理得完，所以平常不會掉行；
        只有 Chassis_TurnPrecise 阻塞期間收到的會全部丟掉 (見 _tick_cb)。"""
        while self.rx_pending and self.rx_line is None:
            line = self.rx_pending.pop(0)
            if line[0] in ALLOWED_FIRST and len(line) < 99:
                self.rx_line = line

    # ---------------- 底盤函式 ----------------
    def Chassis_SpinLeft(self, rpm):
        self.chassis_motion_state = 0
        self.pl.target_rpm, self.pr.target_rpm = -rpm, rpm
        self.pl.integral = self.pr.integral = 0.0

    def Chassis_SpinRight(self, rpm):
        self.chassis_motion_state = 0
        self.pl.target_rpm, self.pr.target_rpm = rpm, -rpm
        self.pl.integral = self.pr.integral = 0.0

    def Chassis_Stop(self):
        self.chassis_motion_state = 0
        self.pl.target_rpm = self.pr.target_rpm = 0.0
        self.pl.integral = self.pr.integral = 0.0
        self.pl.error_last = self.pr.error_last = 0.0

    def Chassis_Forward(self, forward_rpm):
        if self.chassis_motion_state == 0:
            self.pl.integral = self.pr.integral = 0.0
            self.pl.error_last = self.pr.error_last = 0.0
            self.locked_heading_deg = self.chassis_angle_deg
            self.chassis_motion_state = 1
            self.current_ramp_rpm = 15.0
        if self.current_ramp_rpm < forward_rpm:
            self.current_ramp_rpm = min(forward_rpm, self.current_ramp_rpm + 8.0)
        else:
            self.current_ramp_rpm = forward_rpm
        heading_error = self.locked_heading_deg - self.chassis_angle_deg
        comp = 0.0
        if heading_error > 0.5 or heading_error < -0.5:
            comp = max(-8.0, min(8.0, heading_error * 1.0))
        self.pl.target_rpm = self.current_ramp_rpm - comp
        self.pr.target_rpm = self.current_ramp_rpm + comp

    def Chassis_SteerP(self, left, right):
        self.chassis_motion_state = 0
        self.pl.target_rpm, self.pr.target_rpm = left, right

    def _turn_precise_start(self, change, spin_rpm):
        self.turn_start_angle = self.chassis_angle_deg
        self.turn_target = self.chassis_angle_deg + change
        if change > 0:
            self.Chassis_SpinLeft(spin_rpm)
        else:
            self.Chassis_SpinRight(spin_rpm)
        self.turning = True
        self.turn_t0 = self.sim_time
        self._event('TURN_START', f'{change:+.0f}')

    def _turn_precise_poll(self):
        """while(1) { 誤差在 ±2° 內就 break; HAL_Delay(10); } -> Chassis_Stop()，回到 main 的避線分支。"""
        err = self.turn_target - self.chassis_angle_deg
        if -2.0 <= err <= 2.0:
            self.Chassis_Stop()
            self.turning = False
            self._event('TURN_DONE', f'{self.chassis_angle_deg - self.turn_start_angle:+.1f}deg '
                                     f'{self.sim_time - self.turn_t0:.2f}s')
            # main.c：TurnPrecise 回來之後
            self.avoiding = 0
            self.rx_line = None          # command_ready = 0; rx_index = 0; memset(rx_buffer)
            self.rx_pending.clear()
            self.last_seen_time = self.HAL_GetTick() - 2000

    # ---------------- 主迴圈 (while(1) 的一圈) ----------------
    def _main_pass(self):
        if self.blind_dash_active == 1 and (self.HAL_GetTick() - self.blind_dash_start_time > 1000):
            self.blind_dash_active = 0
        if self.rx_line is not None:
            line = self.rx_line
            if self.blind_dash_active and (self.HAL_GetTick() - self.blind_dash_start_time > 1500):
                self.blind_dash_active = 0
            if line.startswith('COURT,'):
                parts = line.split(',')
                if len(parts) >= 4:
                    status, pos = parts[1], parts[2]
                    if status == 'EDGE' and self.avoiding == 0:
                        self.avoiding = 1
                        self.has_ball = 0
                        self.blind_dash_active = 0
                        self.Chassis_Stop()
                        self.avoid_start_time = self.HAL_GetTick()
                        if pos == 'LEFT':
                            self.current_avoid_angle = -240.0
                        elif pos == 'RIGHT':
                            self.current_avoid_angle = 240.0
                        else:
                            self.current_avoid_angle = -360.0
                        self._event('EDGE', f'{pos} -> {self.current_avoid_angle:+.0f}')
            elif line.startswith('BALL,'):
                try:
                    _, dist, ang = line.split(',')[:3]
                    self.ball_distance_cm, self.ball_angle_deg = float(dist), float(ang)
                    ok = True
                except ValueError:
                    ok = False
                if ok:
                    self.has_ball = 1
                    self.last_seen_time = self.HAL_GetTick()
                    if self.avoiding == 0 and self.ball_distance_cm <= 25.0 and self.blind_dash_active == 0:
                        self.blind_dash_active = 1
                        self.blind_dash_start_time = self.HAL_GetTick()
                        self._event('BLIND_DASH', f'{self.ball_distance_cm:.1f}cm')
                    if CHASE_MODE == 'p' and self.avoiding == 0 and self.blind_dash_active == 0 \
                            and self.ball_distance_cm > 30.0:
                        # Sean 的 P 轉向 (9/21 實體車驗證版)：每筆 BALL 更新一次
                        y = STEER_KP * (0.0 - self.ball_angle_deg)
                        half = max(-STEER_WHEEL_MAX, min(STEER_WHEEL_MAX, y / 2.0))
                        self._p_targets = (-half, half)
            elif line.startswith('BALL_OFF') or line.startswith('NOBALL'):
                self.has_ball = 0
                if self.avoiding == 0 and self.blind_dash_active == 0 and 0.0 < self.ball_distance_cm <= 30.0:
                    self.blind_dash_active = 1
                    self.blind_dash_start_time = self.HAL_GetTick()
                    self._event('BLIND_DASH', f'lost@{self.ball_distance_cm:.1f}cm')
            self.rx_line = None

        # ---- 運動控制分流 ----
        if self.avoiding == 1:
            self.state_name = 'AVOID_BRAKE'
            if self.HAL_GetTick() - self.avoid_start_time > 500:
                self._turn_precise_start(self.current_avoid_angle, 60.0)
                self.state_name = 'AVOID_TURN'
                return False   # 進入阻塞的 TurnPrecise
        elif self.has_ball == 1 or self.blind_dash_active == 1:
            if self.blind_dash_active:
                self.state_name = 'BLIND_DASH'
                self.Chassis_Forward(75.0)
            elif self.ball_distance_cm <= 30.0:
                self.state_name = 'CHASE_NEAR'
                self.Chassis_Forward(75.0)
            elif CHASE_MODE == 'p':
                self.state_name = 'CHASE_P'
                tl, tr = getattr(self, '_p_targets', (0.0, 0.0))
                if abs(self.ball_angle_deg) <= 5.0:
                    self.Chassis_Forward(75.0)
                else:
                    self.Chassis_SteerP(tl, tr)
            else:
                turn_threshold = 6.0 if self.ball_distance_cm > 40.0 else 4.0
                if self.ball_angle_deg > turn_threshold or self.ball_angle_deg < -turn_threshold:
                    if self.turn_step_state == 0:
                        self.turn_step_state = 1
                        self.step_start_time = self.HAL_GetTick()
                    if self.turn_step_state == 1:
                        self.state_name = 'CHASE_SPIN'
                        spin = 55.0 if (self.ball_angle_deg > 14.0 or self.ball_angle_deg < -14.0) else 40.0
                        if self.ball_angle_deg > 0:
                            self.Chassis_SpinRight(spin)
                        else:
                            self.Chassis_SpinLeft(spin)
                        if self.HAL_GetTick() - self.step_start_time >= 200:
                            self.turn_step_state = 2
                    elif self.turn_step_state == 2:
                        self.state_name = 'CHASE_WAIT'
                        self.Chassis_Stop()
                        if self.HAL_GetTick() - self.step_start_time >= 1000:
                            self.turn_step_state = 0
                else:
                    self.state_name = 'CHASE_FWD'
                    self.turn_step_state = 0
                    self.Chassis_Forward(75.0)
        else:
            self.state_name = 'PATROL'
            self.Chassis_Forward(80.0)
        return True

    # ---------------- TIM6 10 ms 中斷 ----------------
    def _tim6_isr(self):
        self.m.step()   # 編碼器 (M 法)、低通、PID_Calc、PWM、馬達 (motor_model 跟 main.c 逐行對照)
        dist_l = self.pl.last_counts / ENCODER_RESOLUTION * (2.0 * math.pi * WHEEL_RADIUS)
        dist_r = self.pr.last_counts / ENCODER_RESOLUTION * (2.0 * math.pi * WHEEL_RADIUS)
        self.chassis_angle_deg += math.degrees((dist_r - dist_l) / WHEEL_TRACK)

    def _tick_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.sim_time is None:
            self.sim_time = stamp
            return
        steps = 0
        while self.sim_time + STM32_DT <= stamp + 1e-9 and steps < MAX_CATCHUP:
            self.sim_time += STM32_DT
            self.tick_ms += 10
            if self.turning:
                # 阻塞中：收進來的那行 (command_ready=1) 跟之後掉的字元，轉完都被 memset 清掉 = 全部丟失
                self.rx_pending.clear()
                self.rx_line = None
                self._turn_precise_poll()
            else:
                for _ in range(MAIN_PASSES):
                    self._uart_isr()
                    if not self._main_pass():
                        break
            self._tim6_isr()
            steps += 1
            self.ticks += 1
            if self.log and self.ticks % 100 == 0:
                for f in self._files:
                    f.flush()
            if self.log and self.ticks % 5 == 0:
                self.log.writerow([f'{self.sim_time:.3f}', self.state_name,
                                   f'{self.pl.target_rpm:.1f}', f'{self.pl.rpm:.2f}', f'{self.pr.target_rpm:.1f}',
                                   f'{self.pr.rpm:.2f}', f'{self.pl.pwm:.0f}', f'{self.pr.pwm:.0f}',
                                   f'{self.chassis_angle_deg:.2f}', self.has_ball, f'{self.ball_distance_cm:.1f}',
                                   f'{self.ball_angle_deg:.1f}', self.avoiding, self.blind_dash_active])
        if steps == MAX_CATCHUP:
            self.sim_time = stamp
        if steps == 0:
            return
        twist = Twist()
        twist.linear.x, twist.angular.z = self.m.body_velocity()
        self.cmd_pub.publish(twist)
        self.rpm_pub.publish(Float32MultiArray(data=[self.pl.target_rpm, self.pl.rpm, self.pr.target_rpm,
                                                     self.pr.rpm, self.pl.pwm, self.pr.pwm]))


def main(args=None):
    rclpy.init(args=args)
    node = Stm32BNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
