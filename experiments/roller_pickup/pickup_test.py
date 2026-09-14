"""滾輪撿球物理實驗：把一顆球放在車頭正前方，滾輪轉、車子盲衝固定速度/時間，
用 Gazebo 真實座標看球最後在哪 (有沒有進後車廂)。
每個 trial: 重置車體到原點朝向 0、球放在軸心前方 dist 處 → 開滾輪 → 盲衝 → 停 → 再觀察 3 秒。
用法: python3 experiments/pickup_test.py <out.csv> "<omega_list>" "<speed_list>" [dash_dist]
  例: python3 experiments/pickup_test.py /tmp/x.csv "30,-30" "0.2" 0.8
判定「進後車廂」：球在 base_link 座標 x∈[-0.36,0], |y+0.135|<0.11, z∈[0.05,0.25] 且穩定。
"""
import csv, math, os, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetEntityState, SetEntityState

OUT = sys.argv[1]
OMEGAS = [float(x) for x in sys.argv[2].split(',')]
SPEEDS = [float(x) for x in sys.argv[3].split(',')]
DASH_DIST = float(sys.argv[4]) if len(sys.argv) > 4 else 0.8
BALL = 'ball_1'
AXLE = (0.080, -0.135)  # 軸心中點在 base_link 座標
BALL_AHEAD = 0.5        # 球放在軸心前方多遠 (m)
BALL_Y_OFF = float(os.environ.get('BALL_Y_OFF', '0'))  # 球左右偏移 (m，+ 為車體左側)
ROBOT_X = float(os.environ.get('ROBOT_X', '-6.0'))  # 車子重置的位置 (世界 x=0 有網子，要避開)
SETTLE_SEC = 3.0        # 停車後再觀察多久


def quat_rotate_inverse(qx, qy, qz, qw, vx, vy, vz):
    # world -> body
    cx, cy, cz, cw = -qx, -qy, -qz, qw
    tx = 2 * (cy * vz - cz * vy); ty = 2 * (cz * vx - cx * vz); tz = 2 * (cx * vy - cy * vx)
    return (vx + cw * tx + (cy * tz - cz * ty), vy + cw * ty + (cz * tx - cx * tz), vz + cw * tz + (cx * ty - cy * tx))


class T(Node):
    def __init__(self):
        super().__init__('pickup_test')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.roller = self.create_publisher(Twist, '/roller_cmd', 10)
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.get_cli.wait_for_service(); self.set_cli.wait_for_service()
        self.roller_vel = {}
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        self.w = csv.writer(open(OUT, 'w', newline=''))
        self.w.writerow(['trial', 'omega_cmd', 'speed', 'dash_sec', 'roller_rad_s', 'result', 'ball_lx', 'ball_ly', 'ball_lz', 'ball_max_z', 'lx_at_max_z', 'min_lx', 'ball_world_x', 'ball_world_y', 'robot_x', 'robot_y', 'note'])

    def js_cb(self, m):
        for n in ('left_roller_joint', 'right_roller_joint'):
            if n in m.name:
                self.roller_vel[n] = m.velocity[m.name.index(n)]

    def spin_for(self, sec, twist=None, roller=None):
        t0 = time.time()
        while time.time() - t0 < sec:
            if twist is not None: self.cmd.publish(twist)
            if roller is not None: self.roller.publish(roller)
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, cli, req):
        f = cli.call_async(req); rclpy.spin_until_future_complete(self, f, timeout_sec=5.0); return f.result()

    def get_pose(self, name):
        req = GetEntityState.Request(); req.name = name; req.reference_frame = 'world'
        r = self.call(self.get_cli, req)
        if r is None or not r.success: return None
        p, o = r.state.pose.position, r.state.pose.orientation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    def set_pose(self, name, x, y, z, yaw=0.0):
        req = SetEntityState.Request(); req.state.name = name
        req.state.pose.position.x = x; req.state.pose.position.y = y; req.state.pose.position.z = z
        req.state.pose.orientation.z = math.sin(yaw / 2); req.state.pose.orientation.w = math.cos(yaw / 2)
        req.state.reference_frame = 'world'
        return self.call(self.set_cli, req)

    def ball_local(self):
        rb = self.get_pose('tennis_bot::base_link'); b = self.get_pose(BALL)
        if rb is None or b is None: return None
        (rx, ry, rz), q = rb
        lx, ly, lz = quat_rotate_inverse(*q, b[0][0] - rx, b[0][1] - ry, b[0][2] - rz)
        return lx, ly, lz, b[0], (rx, ry)

    def run_trial(self, i, omega, speed):
        stop = Twist(); roller_off = Twist()
        self.spin_for(0.3, stop, roller_off)
        self.set_pose(BALL, 5.0, 5.0, 0.033)          # 先移開
        self.set_pose('tennis_bot', ROBOT_X, 0.0, 0.0, 0.0)
        self.spin_for(1.0, stop, roller_off)
        # 球放在軸心正前方 (車頭朝 +x)：世界座標 = base_link + (AXLE_x + BALL_AHEAD, AXLE_y)
        self.set_pose(BALL, ROBOT_X + AXLE[0] + BALL_AHEAD, AXLE[1] + BALL_Y_OFF, 0.033)
        roller = Twist(); roller.angular.z = omega
        self.spin_for(3.0, stop, roller)               # 滾輪先轉到穩定 (max_wheel_acceleration=10)
        rv = dict(self.roller_vel)
        dash = Twist(); dash.linear.x = speed
        dash_sec = DASH_DIST / speed
        max_z = 0.0; lx_at_max = 0.0; min_lx = 9.0; t0 = time.time()
        traj = open(OUT.replace('.csv', f'_traj{i}.csv'), 'w'); traj.write('t,lx,ly,lz,roller_l,roller_r,phase\n')
        t_start = time.time()
        def track(phase):
            nonlocal max_z, lx_at_max, min_lx
            bl = self.ball_local()
            if bl:
                if bl[2] > max_z: max_z, lx_at_max = bl[2], bl[0]
                min_lx = min(min_lx, bl[0])
                traj.write(f'{time.time()-t_start:.2f},{bl[0]:.3f},{bl[1]:.3f},{bl[2]:.3f},{self.roller_vel.get("left_roller_joint",0):.1f},{self.roller_vel.get("right_roller_joint",0):.1f},{phase}\n')
        while time.time() - t0 < dash_sec:
            self.cmd.publish(dash); self.roller.publish(roller)
            rclpy.spin_once(self, timeout_sec=0.02); track('dash')
        t0 = time.time()
        while time.time() - t0 < SETTLE_SEC:
            self.cmd.publish(stop); self.roller.publish(roller)
            rclpy.spin_once(self, timeout_sec=0.02); track('settle')
        traj.close()
        bl = self.ball_local()
        lx, ly, lz, bw, rw = bl
        in_basket = (-0.36 <= lx <= 0.0) and abs(ly + 0.135) < 0.11 and 0.05 <= lz <= 0.25
        if in_basket: result = 'IN_BASKET'
        elif lx > 0.05: result = 'STILL_AHEAD'
        elif lz < 0.05: result = 'ON_GROUND_BEHIND' if lx < -0.36 else 'ON_GROUND_UNDER'
        else: result = 'OTHER'
        rr = ', '.join(f'{k[:5]}={v:+.1f}' for k, v in rv.items())
        self.w.writerow([i, omega, speed, f'{dash_sec:.2f}', rr, result, f'{lx:.3f}', f'{ly:.3f}', f'{lz:.3f}', f'{max_z:.3f}', f'{lx_at_max:.3f}', f'{min_lx:.3f}', f'{bw[0]:.3f}', f'{bw[1]:.3f}', f'{rw[0]:.3f}', f'{rw[1]:.3f}', ''])
        print(f'trial {i}: yoff={BALL_Y_OFF:+.2f} omega={omega} v={speed} dash={dash_sec:.1f}s roller[{rr}] -> {result}  ball local=({lx:.3f},{ly:.3f},{lz:.3f}) max_z={max_z:.3f} (lx={lx_at_max:.3f}) min_lx={min_lx:.3f}', flush=True)


rclpy.init()
n = T()
i = 0
for omega in OMEGAS:
    for speed in SPEEDS:
        i += 1
        n.run_trial(i, omega, speed)
n.spin_for(0.3, Twist(), Twist())
print('PICKUP_TEST_DONE')
