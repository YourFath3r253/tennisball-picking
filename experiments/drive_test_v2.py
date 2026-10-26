"""新車體 v2 底盤基本動作測試 (不經過馬達模型，直接 /cmd_vel)：原地轉、直走，看 Gazebo 真實位置。
用法: python3 experiments/drive_test_v2.py   (Gazebo + v2 要先開著)"""
import math
import time

import rclpy
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node

rclpy.init()
n = Node('drive_test_v2')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
g = n.create_client(GetEntityState, '/get_entity_state')
s = n.create_client(SetEntityState, '/set_entity_state')
g.wait_for_service()
s.wait_for_service()


def call(c, r):
    f = c.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    return f.result()


def pose():
    r = GetEntityState.Request()
    r.name = 'tennis_bot'
    st = call(g, r).state
    o = st.pose.orientation
    return st.pose.position, math.degrees(math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z)))


for i in range(1, 11):
    r = SetEntityState.Request()
    r.state.name = 'ball_%d' % i
    r.state.pose.position.x, r.state.pose.position.y, r.state.pose.position.z = 8.0 + i * 0.3, 8.0, 0.033
    r.state.reference_frame = 'world'
    call(s, r)
r = SetEntityState.Request()
r.state.name = 'tennis_bot'
r.state.pose.position.x, r.state.pose.position.z = -8.0, 0.01
r.state.pose.orientation.w = 1.0
r.state.reference_frame = 'world'
call(s, r)
time.sleep(1.5)
for name, v, w in (('原地轉 0.5 rad/s', 0.0, 0.5), ('原地轉 1.0 rad/s', 0.0, 1.0), ('前進 0.4 m/s', 0.4, 0.0)):
    p0, y0 = pose()
    t0 = time.time()
    while time.time() - t0 < 3.0:
        tw = Twist()
        tw.linear.x, tw.angular.z = v, w
        pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.02)
    p1, y1 = pose()
    pub.publish(Twist())
    time.sleep(1.0)
    dy = (y1 - y0 + 180) % 360 - 180
    print(f'{name}: 3 s 轉了 {dy:+.1f}° (指令 {math.degrees(w * 3):+.0f}°)，移動 {math.hypot(p1.x - p0.x, p1.y - p0.y):.2f} m (指令 {v * 3:.2f})')
rclpy.shutdown()
