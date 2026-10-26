"""滾輪轉速掃描：/roller_cmd W 一階一階往上，每階 2 s，讀 /joint_states 的左右滾輪角速度 (rad/s)。
用法: python3 experiments/sponge/roller_speed_scan.py "50,60,63,65,70,80"
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

rclpy.init()
n = rclpy.create_node('roller_speed_scan')
vel = {}
n.create_subscription(JointState, '/joint_states', lambda m: vel.update(
    {k: m.velocity[m.name.index(k)] for k in ('left_roller_joint', 'right_roller_joint') if k in m.name}), 10)
pub = n.create_publisher(Twist, '/roller_cmd', 10)
for W in [float(v) for v in sys.argv[1].split(',')]:
    t = Twist()
    t.angular.z = W
    t0 = time.time()
    samples = []
    while time.time() - t0 < 2.0:
        pub.publish(t)
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - t0 > 1.0 and vel:
            samples.append((vel.get('left_roller_joint', 0), vel.get('right_roller_joint', 0)))
    l = [s[0] for s in samples]
    r = [s[1] for s in samples]
    print(f'W={W:5.1f} 指令 {1.556 * W:6.1f} rad/s -> 左 {min(l):+7.1f}~{max(l):+7.1f}  右 {min(r):+7.1f}~{max(r):+7.1f}', flush=True)
pub.publish(Twist())
rclpy.shutdown()
