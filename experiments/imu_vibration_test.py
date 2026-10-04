"""後車廂裝 N 顆球直線行駛，量陀螺儀振動 (ωx/ωy RMS) 跟航向積分誤差。
用法: python3 imu_vibration_test.py <n_balls> [seconds]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import GetEntityState, SetEntityState

N = int(sys.argv[1]) if len(sys.argv) > 1 else 6
SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
SLOTS = [(-0.08, -0.08), (-0.08, -0.19), (-0.18, -0.08), (-0.18, -0.19), (-0.28, -0.08), (-0.28, -0.19),
         (-0.13, -0.135), (-0.23, -0.135)]
BALLS = ['ball_1', 'ball_2', 'ball_3', 'ball_4', 'ball_5', 'ball_7', 'ball_8', 'ball_9', 'ball_10']
RX, RY = -9.0, 0.0


class T(Node):
    def __init__(self):
        super().__init__('imu_vib_test')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.roller = self.create_publisher(Twist, '/roller_cmd', 10)
        self.gc = self.create_client(GetEntityState, '/get_entity_state')
        self.sc = self.create_client(SetEntityState, '/set_entity_state')
        self.gc.wait_for_service(); self.sc.wait_for_service()
        self.rec = False
        self.n = 0; self.sx = 0.0; self.sy = 0.0; self.yaw_int = 0.0; self.last = None; self.gaps = 0
        self.create_subscription(Imu, '/imu', self.cb, 1000)

    def cb(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.rec:
            self.n += 1
            self.sx += m.angular_velocity.x ** 2
            self.sy += m.angular_velocity.y ** 2
            if self.last is not None:
                dt = st - self.last
                if dt > 0.0015:
                    self.gaps += 1
                if 0 < dt < 0.5:
                    self.yaw_int += m.angular_velocity.z * dt
        self.last = st

    def call(self, c, r):
        f = c.call_async(r); rclpy.spin_until_future_complete(self, f, timeout_sec=5.0); return f.result()

    def setp(self, name, x, y, z):
        r = SetEntityState.Request(); r.state.name = name
        r.state.pose.position.x, r.state.pose.position.y, r.state.pose.position.z = x, y, z
        r.state.pose.orientation.w = 1.0; r.state.reference_frame = 'world'
        self.call(self.sc, r)

    def yaw(self):
        r = GetEntityState.Request(); r.name = 'tennis_bot::base_link'
        o = self.call(self.gc, r).state.pose.orientation
        return math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))

    def spin_for(self, sec, tw=None, w=42.5):
        t0 = time.time()
        while time.time() - t0 < sec:
            if tw is not None: self.cmd.publish(tw)
            rt = Twist(); rt.angular.z = w; self.roller.publish(rt)
            rclpy.spin_once(self, timeout_sec=0.01)

    def run(self):
        self.setp('tennis_bot', RX, RY, 0.05)
        for i, b in enumerate(BALLS):
            if i < N:
                lx, ly = SLOTS[i]
                self.setp(b, RX + lx, RY + ly, 0.08)
            else:
                self.setp(b, -11.5, -5.0 + i, 0.033)
        self.spin_for(3.0, Twist())
        y0 = self.yaw()
        self.rec = True
        tw = Twist(); tw.linear.x = 0.4
        self.spin_for(SEC, tw)
        self.rec = False
        self.spin_for(1.0, Twist())
        y1 = self.yaw()
        real = math.degrees(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)))
        print(f'balls={N} imu_msgs={self.n} gaps={self.gaps} rms_wx={math.sqrt(self.sx / max(1, self.n)):.4f} '
              f'rms_wy={math.sqrt(self.sy / max(1, self.n)):.4f} rad/s  real_yaw_change={real:.3f}° '
              f'gyro_int={math.degrees(self.yaw_int):.3f}° err={math.degrees(self.yaw_int) - real:.3f}°', flush=True)


rclpy.init(); T().run(); rclpy.shutdown()
