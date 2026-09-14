"""滾輪轉 vs 不轉，原地旋轉 12 秒 + 直走 6 秒，比對 IMU 積分航向跟 /odom 真實航向的差。
用來檢查滾輪碰撞 (可能擦到地面) 會不會讓陀螺儀積分跟真實脫節。
用法: ROLLER_W=42.5 python3 experiments/roller_pickup/roller_spin_gyro_test.py <out_prefix>
"""
import csv, math, os, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

OUT = sys.argv[1]
ROLLER_W = float(os.environ.get('ROLLER_W', '0'))
PHASES = [('spin', 0.0, 0.7, 12.0), ('straight', 0.4, 0.0, 6.0), ('stop', 0.0, 0.0, 2.0)]


def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi


class T(Node):
    def __init__(self):
        super().__init__('roller_spin_gyro_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rpub = self.create_publisher(Twist, '/roller_cmd', 10)
        self.imu = []; self.od = []; self.phase = 'init'
        self.create_subscription(Imu, '/imu', lambda m: self.imu.append((m.header.stamp.sec + m.header.stamp.nanosec / 1e9, m.angular_velocity.z, self.phase)), 200)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.t0 = time.time(); self.create_timer(0.1, self.tick)

    def odom_cb(self, m):
        o = m.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        self.od.append((m.header.stamp.sec + m.header.stamp.nanosec / 1e9, yaw, self.phase))

    def tick(self):
        el = time.time() - self.t0; acc = 0.0
        for name, v, w, dur in PHASES:
            if el < acc + dur:
                self.phase = name
                tw = Twist(); tw.linear.x = v; tw.angular.z = w; self.pub.publish(tw)
                rt = Twist(); rt.angular.z = ROLLER_W; self.rpub.publish(rt)
                return
            acc += dur
        self.pub.publish(Twist()); self.rpub.publish(Twist())
        self.report(); raise SystemExit

    def report(self):
        for phase in ['spin', 'straight']:
            im = [m for m in self.imu if m[2] == phase]
            t0, t1 = im[0][0] + 2.0, im[-1][0] - 1.0
            win = [m for m in im if t0 <= m[0] <= t1]
            ods = [o for o in self.od if win[0][0] <= o[0] <= win[-1][0]]
            imu_dy = sum(win[i][1] * (win[i][0] - win[i - 1][0]) for i in range(1, len(win)))
            true_dy = sum(wrap(ods[i][1] - ods[i - 1][1]) for i in range(1, len(ods)))
            print(f'ROLLER_W={ROLLER_W} {phase}: 真實Δyaw={math.degrees(true_dy):+.2f}° IMU∫={math.degrees(imu_dy):+.2f}° 差={math.degrees(imu_dy - true_dy):+.3f}°', flush=True)


rclpy.init()
n = T()
try:
    rclpy.spin(n)
except SystemExit:
    pass
