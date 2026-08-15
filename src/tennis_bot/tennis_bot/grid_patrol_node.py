"""格點路徑巡邏，四個狀態：

  A 巡邏 (PATROL)      沿 32 格弓字型路徑走，0.4 m/s、轉向角速度上限 0.6 rad/s
  B 對準 (ALIGN)       發現球，原地左右轉，P control 對準畫面中心 (kp=0.005)
  C 接近 (APPROACH)    對準後直走，不邊走邊修正，一路走到相機看不到球為止
  D 盲衝 (BLIND_DASH)  相機看不到球 (死角範圍) 後固定時間直衝，蓋過死角距離

車體自身位置完全用輪速里程計算 (讀 /joint_states 拿左右驅動輪真實角速度，
輪子半徑 0.04m、輪距 0.27m，標準差速驅動運動學積分)，不用 /odom、不用
光達/AMCL。「有沒有真的碰到球」則是用 Gazebo 的真實座標判斷 (base_link
座標+姿態，轉換球的世界座標到局部座標系，檢查是否落在 front_chassis 的
實際範圍內)，這兩個是分開的：車體自己「以為」在哪裡 (拿去決定怎麼轉彎)，
跟「有沒有真的碰到」(拿去判斷有沒有成功) 用的資料來源不同。

會把整段軌跡、每顆球碰到的時間、總花費時間都記錄到 CSV，供之後畫圖/分析。
"""
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetEntityState, DeleteEntity

from tennis_bot.grid_waypoints import generate_grid_waypoints

# ---- 網格設定 ----
X_RANGE = (-11.0, 11.0)
Y_RANGE = (-4.5, 4.5)
GRID_COLS = 8
GRID_ROWS = 4
ARRIVE_TOLERANCE_M = 0.3

# ---- 輪速里程計參數 (跟 URDF / diff_drive plugin 設定一致) ----
WHEEL_RADIUS_M = 0.04
WHEEL_SEPARATION_M = 0.27
LEFT_WHEEL_JOINT = 'left_wheel_joint'
RIGHT_WHEEL_JOINT = 'right_wheel_joint'

# ---- 狀態 A 巡邏 ----
PATROL_SPEED = 0.4
PATROL_MAX_OMEGA = 0.6

# ---- 狀態 B 對準 ----
ALIGN_KP = 0.005
ALIGN_MAX_OMEGA = 0.6
PX_PER_RAD = 458.5  # 640px / 1.396rad 相機水平視角
ALIGN_THRESHOLD_PX = math.radians(5.0) * PX_PER_RAD  # ~40px，對應 5 度

# ---- 狀態 C 接近 ----
APPROACH_SPEED = 0.3
LOST_BALL_GRACE_SEC = 0.3  # 單幀漏檢的寬限期，避免雜訊誤判成「已經看不到了」

# ---- 狀態 D 盲衝 ----
BLIND_DASH_SPEED = 0.3
BLIND_DASH_DURATION_SEC = 2.0

# ---- 卡住偵測安全網 ----
STUCK_CHECK_INTERVAL_SEC = 5.0
STUCK_DISTANCE_THRESHOLD_M = 0.15
RECOVER_DURATION_SEC = 1.2
RECOVER_SPEED = -0.2

# ---- 前車體觸碰範圍 (base_link 局部座標系, 從 front_chassis.stl 量出來) ----
FRONT_TOUCH_X_RANGE = (-0.05, 0.35)
FRONT_TOUCH_Y_RANGE = (-0.35, 0.08)

NUM_BALLS = 10
BALL_NAME_PREFIX = 'ball_'

DATA_ROOT = Path('/home/sean/ros2_ws/experiments/實驗數據')


def next_run_dir():
    """自動找下一個沒用過的 runN 資料夾，每次執行都存到新的一個，不會互相覆蓋。"""
    DATA_ROOT.mkdir(parents=True, exist_ok=True)
    n = 1
    while (DATA_ROOT / f'run{n}').exists():
        n += 1
    run_dir = DATA_ROOT / f'run{n}'
    run_dir.mkdir(parents=True)
    return run_dir


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def quat_rotate_inverse(qx, qy, qz, qw, vx, vy, vz):
    cx, cy, cz, cw = -qx, -qy, -qz, qw
    tx = 2.0 * (cy * vz - cz * vy)
    ty = 2.0 * (cz * vx - cx * vz)
    tz = 2.0 * (cx * vy - cy * vx)
    rx = vx + cw * tx + (cy * tz - cz * ty)
    ry = vy + cw * ty + (cz * tx - cx * tz)
    rz = vz + cw * tz + (cx * ty - cy * tx)
    return rx, ry, rz


class GridPatrolNode(Node):
    def __init__(self):
        super().__init__('grid_patrol_node')

        self.waypoints, self.cell_numbers, cw, ch = generate_grid_waypoints(
            X_RANGE, Y_RANGE, GRID_COLS, GRID_ROWS
        )
        self.get_logger().info(f'{GRID_COLS}x{GRID_ROWS}={len(self.waypoints)} 格，每格 {cw:.2f} x {ch:.2f} m')

        # 從格 1 開始，不是從世界原點開始。車體要先被實際傳送到格 1 的座標
        # (跑之前用 reset_run.py)，這裡把里程計的起始值也設成一樣，這樣車體
        # 「自己以為的位置」從第一刻就跟真實位置對得上，不會一開始就有落差。
        start_x, start_y = self.waypoints[0]
        self.x = start_x
        self.y = start_y
        self.yaw = 0.0
        self.wp_index = 0

        # ---- 輪速里程計 (car 自己以為的位置，拿去決定怎麼開) ----
        self._have_odom = False
        self._last_odom_time = None
        self._wheel_vel = {LEFT_WHEEL_JOINT: 0.0, RIGHT_WHEEL_JOINT: 0.0}

        # ---- 視覺 ----
        self.last_target = None
        self.last_seen_time = None

        # ---- 狀態機 ----
        self.state = 'PATROL'  # PATROL / ALIGN / APPROACH / BLIND_DASH / RECOVER
        self.blind_dash_until = None
        self.recover_until = None
        self.stuck_check_pos = None
        self.stuck_check_time = None

        # ---- 球 ----
        self.remaining_balls = [f'{BALL_NAME_PREFIX}{i + 1}' for i in range(NUM_BALLS)]
        self.touched_count = 0
        self.touch_check_in_progress = False

        # ---- 記錄 ----
        self.run_dir = next_run_dir()
        self.get_logger().info(f'這次執行的資料會存到 {self.run_dir}')
        self.traj_file = open(self.run_dir / 'trajectory.csv', 'w', newline='')
        self.traj_writer = csv.writer(self.traj_file)
        self.traj_writer.writerow(['t', 'x', 'y', 'yaw', 'state', 'wp_index', 'cell_number'])
        self.touch_file = open(self.run_dir / 'ball_touches.csv', 'w', newline='')
        self.touch_writer = csv.writer(self.touch_file)
        self.touch_writer.writerow(['ball_name', 'elapsed_sec', 'wp_index_at_touch', 'cell_number_at_touch'])

        self.start_time = time.time()
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(Point, '/target_position', self._target_cb, 10)

        self.get_entity_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.delete_entity_cli = self.create_client(DeleteEntity, '/delete_entity')

        self.control_timer = self.create_timer(0.1, self._control_loop)
        self.log_timer = self.create_timer(0.5, self._log_trajectory)

        self.get_logger().info('開始巡邏。')

    # ---------------- 輪速里程計 ----------------
    def _joint_state_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        for name in (LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.velocity):
                    self._wheel_vel[name] = msg.velocity[idx]

        if self._last_odom_time is not None:
            dt = now - self._last_odom_time
            if 0 < dt < 0.5:
                v_left = WHEEL_RADIUS_M * self._wheel_vel[LEFT_WHEEL_JOINT]
                v_right = WHEEL_RADIUS_M * self._wheel_vel[RIGHT_WHEEL_JOINT]
                v = (v_left + v_right) / 2.0
                omega = (v_right - v_left) / WHEEL_SEPARATION_M
                self.yaw += omega * dt
                self.x += v * math.cos(self.yaw) * dt
                self.y += v * math.sin(self.yaw) * dt
                self._have_odom = True
        self._last_odom_time = now

    # ---------------- 視覺 ----------------
    def _target_cb(self, msg):
        self.last_target = msg
        if msg.z == 1.0:
            self.last_seen_time = self.get_clock().now().nanoseconds / 1e9
            if self.state == 'PATROL':
                self.state = 'ALIGN'
                self.get_logger().info(f'發現網球 -> 對準 (目前在往格 {self.cell_numbers[self.wp_index]} 的路上)')

    # ---------------- 觸碰判定 (前車體，用真實座標) ----------------
    def _check_touch(self):
        if not self.remaining_balls or self.touch_check_in_progress:
            return
        if not self.get_entity_cli.service_is_ready():
            return
        self.touch_check_in_progress = True
        req = GetEntityState.Request()
        req.name = 'tennis_bot::base_link'
        future = self.get_entity_cli.call_async(req)
        future.add_done_callback(self._on_robot_pose_for_touch)

    def _on_robot_pose_for_touch(self, future):
        try:
            resp = future.result()
        except Exception:
            self.touch_check_in_progress = False
            return
        if resp is None or not resp.success:
            self.touch_check_in_progress = False
            return
        p, o = resp.state.pose.position, resp.state.pose.orientation
        robot_pose = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))

        balls = list(self.remaining_balls)
        self._pending_ball_replies = len(balls)
        if self._pending_ball_replies == 0:
            self.touch_check_in_progress = False
            return
        for name in balls:
            req = GetEntityState.Request()
            req.name = name
            future = self.get_entity_cli.call_async(req)
            future.add_done_callback(lambda f, n=name, rp=robot_pose: self._on_ball_pose(f, n, rp))

    def _on_ball_pose(self, future, name, robot_pose):
        self._pending_ball_replies -= 1
        if self._pending_ball_replies <= 0:
            self.touch_check_in_progress = False
        try:
            resp = future.result()
        except Exception:
            return
        if resp is None or not resp.success or name not in self.remaining_balls:
            return

        (rx, ry, rz), (qx, qy, qz, qw) = robot_pose
        p = resp.state.pose.position
        dx, dy, dz = p.x - rx, p.y - ry, p.z - rz
        lx, ly, lz = quat_rotate_inverse(qx, qy, qz, qw, dx, dy, dz)

        in_front = (FRONT_TOUCH_X_RANGE[0] <= lx <= FRONT_TOUCH_X_RANGE[1]
                    and FRONT_TOUCH_Y_RANGE[0] <= ly <= FRONT_TOUCH_Y_RANGE[1])
        if in_front and name in self.remaining_balls:
            self.remaining_balls.remove(name)
            self.touched_count += 1
            elapsed = time.time() - self.start_time
            self.get_logger().info(f'前車體碰到 {name}！({self.touched_count}/{NUM_BALLS}) t={elapsed:.1f}s')
            if not self.done:  # 非同步 callback，結束後才回來的話檔案已經關了，不要再寫
                self.touch_writer.writerow([name, f'{elapsed:.2f}', self.wp_index, self._current_cell_number()])
                self.touch_file.flush()

            del_req = DeleteEntity.Request()
            del_req.name = name
            self.delete_entity_cli.call_async(del_req)

            if self.state in ('ALIGN', 'APPROACH', 'BLIND_DASH'):
                self.state = 'PATROL'

    def _current_cell_number(self):
        if 0 <= self.wp_index < len(self.cell_numbers):
            return self.cell_numbers[self.wp_index]
        return ''

    # ---------------- 記錄軌跡 ----------------
    def _log_trajectory(self):
        # _log_trajectory 是獨立的 0.5s timer，跟 _control_loop 的 done 判斷是分開的
        # 兩條路徑。結束時 _control_loop 關掉了檔案，但這個 timer 還是會繼續按表
        # 定時觸發，沒擋住的話下一次觸發就會對已關閉的檔案寫入而整個崩潰
        # (實測真的發生過，表面上看起來像「卡住」，其實是巡邏正常跑完之後才炸的)。
        if self.done or not self._have_odom:
            return
        elapsed = time.time() - self.start_time
        self.traj_writer.writerow([f'{elapsed:.2f}', f'{self.x:.3f}', f'{self.y:.3f}',
                                    f'{self.yaw:.3f}', self.state, self.wp_index, self._current_cell_number()])
        self.traj_file.flush()

    # ---------------- 控制迴圈 ----------------
    def _control_loop(self):
        if not self._have_odom or self.done:
            return

        if not self.remaining_balls or self.wp_index >= len(self.waypoints):
            self.done = True
            self.cmd_pub.publish(Twist())
            elapsed = time.time() - self.start_time
            self.get_logger().info(
                f'結束：碰到 {self.touched_count}/{NUM_BALLS} 顆球，'
                f'走了 {self.wp_index}/{len(self.waypoints)} 格，耗時 {elapsed:.1f}s'
            )
            self.log_timer.cancel()
            self.traj_file.close()
            self.touch_file.close()
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # ---- 卡住偵測安全網 ----
        if self.state == 'RECOVER':
            if now < self.recover_until:
                twist = Twist()
                twist.linear.x = RECOVER_SPEED
                twist.angular.z = 0.4
                self.cmd_pub.publish(twist)
                return
            self.state = 'PATROL'
            self.stuck_check_pos = (self.x, self.y)
            self.stuck_check_time = now

        if self.state != 'PATROL':
            self.stuck_check_pos = None
        elif self.stuck_check_pos is None:
            self.stuck_check_pos = (self.x, self.y)
            self.stuck_check_time = now
        elif now - self.stuck_check_time > STUCK_CHECK_INTERVAL_SEC:
            moved = math.hypot(self.x - self.stuck_check_pos[0], self.y - self.stuck_check_pos[1])
            if moved < STUCK_DISTANCE_THRESHOLD_M:
                self.get_logger().warn('卡住了，倒車重新來')
                self.state = 'RECOVER'
                self.recover_until = now + RECOVER_DURATION_SEC
                twist = Twist()
                twist.linear.x = RECOVER_SPEED
                twist.angular.z = 0.4
                self.cmd_pub.publish(twist)
                return
            self.stuck_check_pos = (self.x, self.y)
            self.stuck_check_time = now

        self._check_touch()

        if self.state == 'PATROL':
            self._do_patrol()
        elif self.state == 'ALIGN':
            self._do_align(now)
        elif self.state == 'APPROACH':
            self._do_approach(now)
        elif self.state == 'BLIND_DASH':
            self._do_blind_dash(now)

    # ---- 狀態 A ----
    def _do_patrol(self):
        target_x, target_y = self.waypoints[self.wp_index]
        dist = math.hypot(target_x - self.x, target_y - self.y)
        if dist <= ARRIVE_TOLERANCE_M:
            self.wp_index += 1
            return
        target_angle = math.atan2(target_y - self.y, target_x - self.x)
        err = angle_diff(target_angle, self.yaw)
        twist = Twist()
        twist.angular.z = max(-PATROL_MAX_OMEGA, min(PATROL_MAX_OMEGA, err * 1.2))
        speed_scale = max(0.0, 1.0 - abs(err) / (math.pi / 2))
        twist.linear.x = PATROL_SPEED * speed_scale
        self.cmd_pub.publish(twist)

    # ---- 狀態 B ----
    def _do_align(self, now):
        if self.last_target is None or self.last_target.z != 1.0:
            # 對準階段還沒開始靠近，看丟目標不是進了死角，是真的沒有目標了
            # (雜訊或球本來就不在那)，放棄追這顆球，回去巡邏
            if self._ball_truly_lost(now):
                self.get_logger().info('對準時看丟目標 -> 放棄，回到巡邏')
                self.state = 'PATROL'
            return
        error_x = 320.0 - self.last_target.x
        twist = Twist()
        if abs(error_x) > ALIGN_THRESHOLD_PX:
            omega = max(-ALIGN_MAX_OMEGA, min(ALIGN_MAX_OMEGA, error_x * ALIGN_KP))
            twist.angular.z = omega
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
        else:
            self.state = 'APPROACH'

    # ---- 狀態 C ----
    def _do_approach(self, now):
        if self.last_target is not None and self.last_target.z == 1.0:
            twist = Twist()
            twist.linear.x = APPROACH_SPEED
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
        elif self._ball_truly_lost(now):
            # 已經在直走接近了，看不到了 = 進了相機死角，交給盲衝蓋過去
            self.state = 'BLIND_DASH'
            self.blind_dash_until = now + BLIND_DASH_DURATION_SEC
            self.get_logger().info('看不到球了 (死角) -> 盲衝')

    def _ball_truly_lost(self, now):
        time_since_seen = now - self.last_seen_time if self.last_seen_time is not None else float('inf')
        return time_since_seen >= LOST_BALL_GRACE_SEC

    # ---- 狀態 D ----
    def _do_blind_dash(self, now):
        if now < self.blind_dash_until:
            twist = Twist()
            twist.linear.x = BLIND_DASH_SPEED
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
        else:
            self.state = 'PATROL'


def main(args=None):
    rclpy.init(args=args)
    node = GridPatrolNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
