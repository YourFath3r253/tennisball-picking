"""模擬裡的「STM32 + 底盤馬達」：收左右輪目標 rpm，跑實體車的輪速 PID + 馬達模型，算出車體速度交給 Gazebo。

資料流 (跟實體車對應)：
  控制節點 --/wheel_target_rpm [左, 右]--> 本節點 (= STM32 的 pid_left/right.target_rpm)
  本節點每 10 ms 跑一次 motor_model (= TIM6 中斷：編碼器、濾波、PID、PWM、馬達)
  車體 (v, ω) 用實體車幾何算 -> /cmd_vel -> Gazebo diff_drive 只負責把車體照這個速度推動

時間用模擬時間：每收到一筆 /joint_states (100 Hz，封包時間戳) 就把模型往前推到那個時間點，
即時率 < 1 時馬達也不會變快。還沒收到任何目標 rpm 之前不發 /cmd_vel (不干擾直接發 cmd_vel 的測試工具)。

環境變數 TENNISBOT_MOTOR_MODEL=ideal：關掉馬達模型，目標 rpm 直接換成車體速度 (舊行為，對照用)。
另外發 /wheel_rpm [目標左, 實際左, 目標右, 實際右, PWM左, PWM右] (= STM32 的 RPM telemetry) 方便記錄。
"""
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from tennis_bot.motor_model import ChassisMotorModel, STM32_DT, WHEEL_RADIUS_REAL, WHEEL_TRACK_REAL

MOTOR_MODEL = os.environ.get('TENNISBOT_MOTOR_MODEL', 'real')
MAX_CATCHUP_STEPS = 50  # /joint_states 斷掉很久時，一次最多補 0.5 s，避免卡住


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.model = ChassisMotorModel()
        self.targets = (0.0, 0.0)
        self.active = False
        self.sim_time = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rpm_pub = self.create_publisher(Float32MultiArray, '/wheel_rpm', 10)
        self.create_subscription(Float32MultiArray, '/wheel_target_rpm', self._target_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._tick_cb, 10)
        self.get_logger().info(
            '馬達模型：' + ('理想 (目標 rpm 直接變成速度)' if MOTOR_MODEL == 'ideal'
                         else 'STM32 輪速 PID + 直流馬達 (motor_model.DEFAULT_PARAMS)'))

    def _target_cb(self, msg):
        if len(msg.data) < 2:
            return
        self.targets = (float(msg.data[0]), float(msg.data[1]))
        self.model.set_targets(*self.targets)
        self.active = True

    def _tick_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.sim_time is None:
            self.sim_time = stamp
            return
        if not self.active:
            self.sim_time = stamp
            return
        steps = 0
        while self.sim_time + STM32_DT <= stamp + 1e-9 and steps < MAX_CATCHUP_STEPS:
            self.model.step()
            self.sim_time += STM32_DT
            steps += 1
        if steps == MAX_CATCHUP_STEPS:
            self.sim_time = stamp
        if steps == 0:
            return

        twist = Twist()
        if MOTOR_MODEL == 'ideal':
            wl = self.targets[0] * 2 * math.pi / 60.0
            wr = self.targets[1] * 2 * math.pi / 60.0
            twist.linear.x = WHEEL_RADIUS_REAL * (wl + wr) / 2.0
            twist.angular.z = WHEEL_RADIUS_REAL * (wr - wl) / WHEEL_TRACK_REAL
        else:
            twist.linear.x, twist.angular.z = self.model.body_velocity()
        self.cmd_pub.publish(twist)

        tel = Float32MultiArray()
        tel.data = [self.targets[0], self.model.left.rpm, self.targets[1], self.model.right.rpm,
                    self.model.left.pwm, self.model.right.pwm]
        self.rpm_pub.publish(tel)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotorDriverNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
