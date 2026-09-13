"""格點路徑巡邏，四個狀態：

  A 巡邏 (PATROL)      沿 32 格弓字型路徑走，0.4 m/s、轉向角速度上限 0.6 rad/s
  B 對準 (ALIGN)       發現球，原地左右轉，P control 對準畫面中心 (kp=0.005)
  C 接近 (APPROACH)    對準後往前走，邊走邊用簡單PID修正方向 (球中心偏離畫面
                        中心的像素誤差)，一路走到相機看不到球為止
  D 盲衝 (BLIND_DASH)  相機看不到球 (死角範圍) 後固定時間直衝，蓋過死角距離
  E 原路倒車 (RETURNING) 追球 (B/C/D) 結束後，把追球期間送出的指令倒過來重播一遍，
                        不靠里程計，直接原路退回巡邏路線再繼續 PATROL

車體自身位置用輪速+陀螺儀算 (讀 /joint_states 拿左右驅動輪真實角速度算前進
速度，輪子半徑 0.04m、輪距 0.29m；轉向角速度改用 /imu 陀螺儀直接量到的角速度，
不再用左右輪速差推算，因為輪子在原地轉向 (ALIGN) 或撞到東西時容易打滑，輪速
差算出來的角速度會跟車體真實轉動的角度對不上)，不用 /odom、不用光達/AMCL。
陀螺儀目前是理想 sensor (URDF 裡沒有加 <noise>)，先驗證這個方向有沒有用。「有沒有真的碰到球」則是用 Gazebo 的真實座標判斷 (base_link
座標+姿態，轉換球的世界座標到局部座標系，檢查有沒有跟左右滾輪的位置
重疊)，這兩個是分開的：車體自己「以為」在哪裡 (拿去決定怎麼轉彎)，
跟「有沒有真的碰到」(拿去判斷有沒有成功) 用的資料來源不同。
左右滾輪 (left_roller/right_roller) 在 URDF 裡沒有開 collision，撿球判定
全部是軟體幾何判定，不靠物理碰撞，避免滾輪真的輾過球時的碰撞力沒被輪速
里程計記錄到卻讓車體有真實偏移。front_chassis 也沒有 collision (滾輪機構
要能穿過球)；左右驅動輪維持原本的 collision (要滾地才能開)。

會把整段軌跡、每顆球碰到的時間、總花費時間都記錄到 CSV，供之後畫圖/分析。
"""
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Empty
from gazebo_msgs.srv import GetEntityState, DeleteEntity, SetEntityState

from tennis_bot.grid_waypoints import generate_grid_waypoints

# ---- 網格設定 ----
X_RANGE = (-11.0, 11.0)
Y_RANGE = (-4.5, 4.5)

# 真牆的實際座標 (tennis_court.world 的 court_walls，跟 X_RANGE/Y_RANGE 那個
# 1公尺安全邊界不是同一件事，牆在更外面): x=±12, y=±5.5。撞牆判定直接用
# Gazebo 真實座標算「離牆多近」，不像實體車那樣需要額外裝光達才能知道——
# 這是模擬才有的優勢，判定準確、不會像里程計那樣被感測器雜訊誤導。
WALL_X_M = 12.0
WALL_Y_M = 5.5
WALL_CRASH_MARGIN_M = 0.2
GRID_COLS = 8
GRID_ROWS = 4
ARRIVE_TOLERANCE_M = 0.3

# ---- 輪速里程計參數 ----
# WHEEL_SEPARATION_M 原本抄 diff_drive plugin 裡的 0.27，但那是 plugin 自己換算
# cmd_vel 用的假設值，不是真正的輪距。實際從 Gazebo 查 left/right_drive_wheel
# 在 base_link 局部座標系的位置：左輪 y=0.01，右輪 y=-0.28，真實輪距是 0.29m。
# 這個常數用在把「量到的左右輪速差」換算回「車體真實轉了多少角速度」，數字
# 差 0.27 vs 0.29 (差約7.4%) 會讓每次轉向都被里程計系統性高估角度，多次累積
# 起來就是我們看到的角度誤差越滾越大的主因。
WHEEL_RADIUS_M = 0.04
WHEEL_SEPARATION_M = 0.29
# 里程計算的是「兩個驅動輪軸心中點」的運動，但 Gazebo GetEntityState 回報的是
# base_link 原點，兩者在車體座標差 (+0.080, -0.135) m (實測 left/right_drive_wheel
# 與 base_link 的世界座標算出來的)。原地轉的時候軸心不動、base_link 卻繞 0.157m
# 半徑畫圓，不換算的話會憑空多出最多 0.31m 的「誤差」。
AXLE_MID_IN_BASE = (0.080, -0.135)
LEFT_WHEEL_JOINT = 'left_wheel_joint'
RIGHT_WHEEL_JOINT = 'right_wheel_joint'

# ---- 狀態 A 巡邏 ----
PATROL_SPEED = 0.4
PATROL_MAX_OMEGA = 0.6

# ---- 狀態 B 對準 ----
# 純 P 控制。曾經以為兩顆球角度接近時的擺盪是控制過衝，加了D項沒有效果——
# 真正原因是 vision_node 那邊鎖定的目標本身在兩顆球之間切換，不是同一顆球的
# 誤差訊號在震盪，D項對不連續跳動的訊號沒有意義。改用 vision 端更嚴格的鎖定
# (鎖定期間完全不切換，除非真的超過2秒沒看到任何球) 才是對的方向。
ALIGN_KP = 0.005
ALIGN_MAX_OMEGA = 0.6
PX_PER_RAD = 458.5  # 640px / 1.396rad 相機水平視角
ALIGN_THRESHOLD_PX = math.radians(5.0) * PX_PER_RAD  # ~40px，對應 5 度

# ---- 狀態 C 接近 ----
APPROACH_SPEED = 0.3
LOST_BALL_GRACE_SEC = 0.3  # 單幀漏檢的寬限期，避免雜訊誤判成「已經看不到了」
# 原本 APPROACH 是完全不修正方向的直走，ALIGN 交接時殘留的 ±5° 誤差在距離遠時
# (接近3m偵測極限) 會被放大成明顯的橫向偏移。改成簡單 PID，邊走邊用同一個像素
# 誤差(球中心離畫面中心多遠)修正角速度，先用一組保守的參數試試看。
APPROACH_PID_KP = 0.0025
APPROACH_PID_KI = 0.0
APPROACH_PID_KD = 0.0008
APPROACH_MAX_OMEGA = 0.3

# ---- 狀態 D 盲衝 ----
BLIND_DASH_SPEED = 0.3
BLIND_DASH_DURATION_SEC = 2.0

# ---- 撿球判定：用左右滾輪的位置判斷 ----
# 前車體 (front_chassis) 跟左右驅動輪都保留原本的 collision (驅動輪要滾地才能跑，
# 前車體是刻意設計沒有 collision 讓滾輪撿球機構能穿過球)。左右滾輪 (left_roller/
# right_roller) 的 collision 拿掉，改用軟體幾何判定，避免滾輪真的輾過球時的碰撞力
# 沒被輪速里程計記錄到、卻讓車體有真實偏移。座標從 Gazebo 實際查出來 (base_link
# 局部座標系)：左滾輪 (0.1023, -0.065)，右滾輪 (0.1023, -0.205)，滾輪半徑 0.05m。
# 判定範圍 = 滾輪半徑 + 球半徑(0.033m)，球只要跟任一滾輪的圓形範圍有一點點重疊
# 就算碰到，不用整顆球都進去。
LEFT_ROLLER_TOUCH_LOCAL = (0.1023, -0.065)
RIGHT_ROLLER_TOUCH_LOCAL = (0.1023, -0.205)
ROLLER_TOUCH_RADIUS_M = 0.05 + 0.033

# run8 只測滾輪判定、run9 才加上原路倒車，兩個改動分開測試看各自的效果
RETURN_REPLAY_ENABLED = False

# 里程計積分的 dt 用 /joint_states 封包自帶的時間戳 (Gazebo 模擬時間) 相減。
# run63~70 實測模擬即時率只有 0.91~0.93，之前用真實時鐘算 dt，里程計走的距離
# 天生就多 7~10%，跟量到的 odom/real 距離比例幾乎一樣。實體車也是一樣的做法：
# 用編碼器封包上的時間戳，不用收到封包的電腦時間。
# 之前試過用 /clock 算 dt，但 /clock 只有 10Hz、/joint_states 30Hz，大半 dt 會是 0。

# ball_6 從 tennis_court.world 移除了 (一開場就會馬上撿到，測試不到東西，
# 而且之前出現過一次奇怪的碰撞行為)，場上現在固定是這 9 顆。
BALL_NAMES = ['ball_1', 'ball_2', 'ball_3', 'ball_4', 'ball_5',
              'ball_7', 'ball_8', 'ball_9', 'ball_10']
NUM_BALLS = len(BALL_NAMES)

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

        # 從格 1 開始，不是從世界原點開始。車體實際位置跟里程計起始值必須對得上，
        # 不然會整趟路線平移掉 (曾經忘記傳送車體，跑出完全不對的巡邏路線)。
        # 所以這裡自己呼叫 /set_entity_state 把車體傳送到格 1，不依賴外部先手動
        # 跑 reset_run.py。
        start_x, start_y = self.waypoints[0]
        self._teleport_to_start(start_x, start_y)
        # 傳送的是 base_link，里程計起點要放在軸心中點 (起始朝向 0，直接加偏移)
        self.x = start_x + AXLE_MID_IN_BASE[0]
        self.y = start_y + AXLE_MID_IN_BASE[1]
        self.yaw = 0.0
        self.wp_index = 0

        # ---- 輪速里程計 (car 自己以為的位置，拿去決定怎麼開) ----
        self._have_odom = False
        self._last_stamp = None  # /joint_states 封包的模擬時間戳
        self._wheel_vel = {LEFT_WHEEL_JOINT: 0.0, RIGHT_WHEEL_JOINT: 0.0}

        # ---- debug 對照組：同一份輪速+陀螺儀，但用真實時鐘的 dt 積分 (舊做法)，
        # 只寫進 trajectory.csv 比對，不拿去控制 ----
        self._last_odom_time = None
        self.x_wall = self.x
        self.y_wall = self.y
        self.yaw_wall = 0.0

        # ---- 陀螺儀：yaw 在 _imu_cb 裡直接積分，不用左右輪速差算 ----
        self._last_imu_stamp = None
        self._last_imu_time = None

        # ---- debug：追蹤中的目標球是幾號 (用真實座標反推，不是 vision_node 自己
        # 知道的，vision_node 只有像素座標，沒有球的身分) ----
        self._ball_real_positions = {}

        # ---- 狀態 C PID (邊接近邊修正方向) ----
        self._approach_integral = 0.0
        self._approach_prev_error = 0.0
        self._approach_prev_time = None

        # ---- 視覺 ----
        self.last_target = None
        self.last_seen_time = None

        # ---- 狀態機 ----
        self.state = 'PATROL'  # PATROL / ALIGN / APPROACH / BLIND_DASH / RETURNING
        self.blind_dash_until = None

        # ---- 追球動作紀錄，追完後原路倒車回去 (不靠里程計，直接反轉重播) ----
        self.chase_cmd_log = []
        self.return_index = 0

        # ---- 球 ----
        self.remaining_balls = list(BALL_NAMES)
        self.touched_count = 0
        self.touch_check_in_progress = False

        # ---- 記錄 ----
        self.run_dir = next_run_dir()
        self.get_logger().info(f'這次執行的資料會存到 {self.run_dir}')
        self.traj_file = open(self.run_dir / 'trajectory.csv', 'w', newline='')
        self.traj_writer = csv.writer(self.traj_file)
        self.traj_writer.writerow([
            't', 'x', 'y', 'yaw', 'state', 'wp_index', 'cell_number',
            'real_x', 'real_y', 'real_yaw', 'target_ball_guess',
            'x_wall', 'y_wall', 'yaw_wall',
        ])
        self.touch_file = open(self.run_dir / 'ball_touches.csv', 'w', newline='')
        self.touch_writer = csv.writer(self.touch_file)
        self.touch_writer.writerow(['ball_name', 'elapsed_sec', 'wp_index_at_touch', 'cell_number_at_touch'])

        # ---- debug：每次 _joint_state_cb 觸發時記錄 wall_dt vs sim_dt (封包時間戳)，
        # 累加起來的比值就是這次 run 的平均即時率 ----
        self.dt_debug_file = open(self.run_dir / 'dt_debug.csv', 'w', newline='')
        self.dt_debug_writer = csv.writer(self.dt_debug_file)
        self.dt_debug_writer.writerow(['t', 'wall_dt', 'sim_dt', 'state', 'dropped'])

        self.start_time = time.time()
        with open(self.run_dir / 'meta.txt', 'w') as f:
            f.write(f'start_epoch={self.start_time}\n')
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vision_reset_pub = self.create_publisher(Empty, '/vision_reset_lock', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)
        self.create_subscription(Point, '/target_position', self._target_cb, 10)

        self.get_entity_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.delete_entity_cli = self.create_client(DeleteEntity, '/delete_entity')

        # debug 用：定期查 Gazebo 真實座標，跟里程計「以為的」位置分開記錄，
        # 這樣才看得出來里程計什麼時候開始跟真實位置對不上 (追球轉彎、撞牆時最容易飄)。
        self.real_x = None
        self.real_y = None
        self.real_yaw = None
        self._real_pose_pending = False
        self.real_pose_timer = self.create_timer(0.2, self._poll_real_pose)  # 每次回來就寫一筆 trajectory

        self.control_timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('開始巡邏。')

    def _teleport_to_start(self, x, y):
        cli = self.create_client(SetEntityState, '/set_entity_state')
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/set_entity_state 服務沒回應，車體傳送失敗，Gazebo 有在跑嗎？')
            return
        req = SetEntityState.Request()
        req.state.name = 'tennis_bot'
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = 0.05
        req.state.pose.orientation.w = 1.0
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        self.get_logger().info(f'車體傳送到格 1 ({x:.3f}, {y:.3f})：{result.success if result else False}')

    # ---------------- 陀螺儀 ----------------
    def _imu_cb(self, msg):
        # 航向在這裡用 IMU 自己的封包時間戳積分 (50Hz，每筆剛好算一次)。之前是在
        # 30Hz 的 /joint_states callback 抓「最新一筆」角速度乘 dt，等於每秒有 20 筆
        # 陀螺儀資料被跳過或重複算，run72 實測每次轉向會隨機差 ±1°。
        now = self.get_clock().now().nanoseconds / 1e9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        omega = msg.angular_velocity.z
        if self._last_imu_stamp is not None:
            dt = stamp - self._last_imu_stamp
            if 0 < dt < 0.5:
                self.yaw += omega * dt
            wall_dt = now - self._last_imu_time
            if 0 < wall_dt < 0.5:
                self.yaw_wall += omega * wall_dt
        self._last_imu_stamp = stamp
        self._last_imu_time = now

    # ---------------- 輪速里程計 ----------------
    def _joint_state_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        for name in (LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.velocity):
                    self._wheel_vel[name] = msg.velocity[idx]

        wall_dt = (now - self._last_odom_time) if self._last_odom_time is not None else None
        sim_dt = (stamp - self._last_stamp) if self._last_stamp is not None else None

        v_left = WHEEL_RADIUS_M * self._wheel_vel[LEFT_WHEEL_JOINT]
        v_right = WHEEL_RADIUS_M * self._wheel_vel[RIGHT_WHEEL_JOINT]
        v = (v_left + v_right) / 2.0

        dropped = False
        if sim_dt is not None:
            if 0 < sim_dt < 0.5:
                self.x += v * math.cos(self.yaw) * sim_dt
                self.y += v * math.sin(self.yaw) * sim_dt
                self._have_odom = True
            else:
                dropped = True

        if wall_dt is not None and 0 < wall_dt < 0.5:
            self.x_wall += v * math.cos(self.yaw_wall) * wall_dt
            self.y_wall += v * math.sin(self.yaw_wall) * wall_dt

        if wall_dt is not None and not self.done:
            elapsed = time.time() - self.start_time
            self.dt_debug_writer.writerow([
                f'{elapsed:.3f}', f'{wall_dt:.4f}',
                f'{sim_dt:.4f}' if sim_dt is not None else '',
                self.state, int(dropped),
            ])

        self._last_odom_time = now
        self._last_stamp = stamp

    # ---------------- 視覺 ----------------
    def _target_cb(self, msg):
        self.last_target = msg
        if msg.z == 1.0:
            self.last_seen_time = self.get_clock().now().nanoseconds / 1e9
            if self.state == 'PATROL':
                self.state = 'ALIGN'
                self.chase_cmd_log = []
                guess = self._estimate_target_ball()
                self.get_logger().info(
                    f'發現網球 -> 對準 (猜是{guess or "?"}，目前在往格 {self.cell_numbers[self.wp_index]} 的路上)')

    # ---------------- 觸碰判定 (左右滾輪，用真實座標) ----------------
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

        p = resp.state.pose.position
        self._ball_real_positions[name] = (p.x, p.y)  # debug 用，猜目前追蹤的是幾號球

        (rx, ry, rz), (qx, qy, qz, qw) = robot_pose
        dx, dy, dz = p.x - rx, p.y - ry, p.z - rz
        lx, ly, lz = quat_rotate_inverse(qx, qy, qz, qw, dx, dy, dz)

        dist_left = math.hypot(lx - LEFT_ROLLER_TOUCH_LOCAL[0], ly - LEFT_ROLLER_TOUCH_LOCAL[1])
        dist_right = math.hypot(lx - RIGHT_ROLLER_TOUCH_LOCAL[0], ly - RIGHT_ROLLER_TOUCH_LOCAL[1])
        in_front = dist_left <= ROLLER_TOUCH_RADIUS_M or dist_right <= ROLLER_TOUCH_RADIUS_M
        if in_front and name in self.remaining_balls:
            self.remaining_balls.remove(name)
            self.touched_count += 1
            elapsed = time.time() - self.start_time
            self.get_logger().info(f'滾輪碰到 {name}！({self.touched_count}/{NUM_BALLS}) t={elapsed:.1f}s')
            if not self.done:  # 非同步 callback，結束後才回來的話檔案已經關了，不要再寫
                self.touch_writer.writerow([name, f'{elapsed:.2f}', self.wp_index, self._current_cell_number()])
                self.touch_file.flush()

            del_req = DeleteEntity.Request()
            del_req.name = name
            self.delete_entity_cli.call_async(del_req)

            if self.state in ('ALIGN', 'APPROACH', 'BLIND_DASH'):
                self._start_return()

    # ---------------- debug：真實座標 ----------------
    def _poll_real_pose(self):
        if self.done or self._real_pose_pending or not self.get_entity_cli.service_is_ready():
            return
        self._real_pose_pending = True
        req = GetEntityState.Request()
        req.name = 'tennis_bot::base_link'
        future = self.get_entity_cli.call_async(req)
        future.add_done_callback(self._on_real_pose)

    def _on_real_pose(self, future):
        self._real_pose_pending = False
        try:
            resp = future.result()
        except Exception:
            return
        if resp is None or not resp.success:
            return
        p, o = resp.state.pose.position, resp.state.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        ox, oy = AXLE_MID_IN_BASE
        self.real_x = p.x + ox * math.cos(yaw) - oy * math.sin(yaw)
        self.real_y = p.y + ox * math.sin(yaw) + oy * math.cos(yaw)
        self.real_yaw = yaw

        if not self.done:
            near_wall = (
                abs(self.real_x) >= WALL_X_M - WALL_CRASH_MARGIN_M
                or abs(self.real_y) >= WALL_Y_M - WALL_CRASH_MARGIN_M
            )
            if near_wall:
                self._finish_run('撞牆')
        # 真實座標一到手就立刻記錄，里程計跟真實位置才是同一瞬間的值
        # (之前是另一個 0.5s timer 記錄，兩邊最多差 0.2~0.5 秒，車在動時會看起來像誤差)
        self._log_trajectory()

    def _current_cell_number(self):
        if 0 <= self.wp_index < len(self.cell_numbers):
            return self.cell_numbers[self.wp_index]
        return ''

    # ---------------- debug：猜目前 vision 追蹤的是幾號球 ----------------
    # vision_node 只知道像素座標 (cx,cy)，不知道那是哪一顆球。這裡反過來用真實
    # 座標 (機器人真實位置/朝向 + 每顆球真實位置) 算出每顆球「應該在畫面哪個角度」，
    # 跟 vision 回報的像素位置換算出的角度比對，哪顆最接近就猜是那顆。
    # 是近似值 (用 base_link 的位置/朝向，沒有扣掉相機本身 0.135m 的左右偏移)，
    # 只用來debug，不是精確判定。
    def _estimate_target_ball(self):
        if (self.last_target is None or self.last_target.z != 1.0
                or self.real_x is None or self.real_yaw is None):
            return ''
        implied_bearing = (320.0 - self.last_target.x) / PX_PER_RAD
        best_name, best_diff = '', None
        for name, (bx, by) in self._ball_real_positions.items():
            if name not in self.remaining_balls:
                continue
            dx, dy = bx - self.real_x, by - self.real_y
            local_x = dx * math.cos(-self.real_yaw) - dy * math.sin(-self.real_yaw)
            local_y = dx * math.sin(-self.real_yaw) + dy * math.cos(-self.real_yaw)
            if local_x <= 0:
                continue  # 在車體後面，不可能是畫面裡看到的
            bearing = math.atan2(local_y, local_x)
            diff = abs(bearing - implied_bearing)
            if best_diff is None or diff < best_diff:
                best_diff, best_name = diff, name
        return best_name

    # ---------------- 記錄軌跡 ----------------
    def _log_trajectory(self):
        # 結束後檔案已經關了，非同步回來的 callback 不能再寫 (實測真的炸過)
        if self.done or not self._have_odom:
            return
        elapsed = time.time() - self.start_time
        real_x = f'{self.real_x:.3f}' if self.real_x is not None else ''
        real_y = f'{self.real_y:.3f}' if self.real_y is not None else ''
        real_yaw = f'{self.real_yaw:.3f}' if self.real_yaw is not None else ''
        self.traj_writer.writerow([f'{elapsed:.2f}', f'{self.x:.3f}', f'{self.y:.3f}',
                                    f'{self.yaw:.3f}', self.state, self.wp_index, self._current_cell_number(),
                                    real_x, real_y, real_yaw, self._estimate_target_ball(),
                                    f'{self.x_wall:.3f}', f'{self.y_wall:.3f}', f'{self.yaw_wall:.3f}'])
        self.traj_file.flush()

    def _finish_run(self, reason):
        self.done = True
        self.cmd_pub.publish(Twist())
        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f'結束({reason})：碰到 {self.touched_count}/{NUM_BALLS} 顆球，'
            f'走了 {self.wp_index}/{len(self.waypoints)} 格，耗時 {elapsed:.1f}s'
        )
        self.real_pose_timer.cancel()
        self.traj_file.close()
        self.touch_file.close()
        self.dt_debug_file.close()

    # ---------------- 控制迴圈 ----------------
    def _control_loop(self):
        if not self._have_odom or self.done:
            return

        if not self.remaining_balls:
            self._finish_run('撿滿')
            return
        if self.wp_index >= len(self.waypoints):
            self._finish_run('走完弓字路徑')
            return

        now = self.get_clock().now().nanoseconds / 1e9

        self._check_touch()

        if self.state == 'PATROL':
            self._do_patrol()
        elif self.state == 'ALIGN':
            self._do_align(now)
        elif self.state == 'APPROACH':
            self._do_approach(now)
        elif self.state == 'BLIND_DASH':
            self._do_blind_dash(now)
        elif self.state == 'RETURNING':
            self._do_return()

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
                how = '原路倒車回去' if RETURN_REPLAY_ENABLED else '直接走回巡邏路徑'
                self.get_logger().info(f'對準時看丟目標 -> 放棄，{how}')
                self._start_return()
            return
        error_x = 320.0 - self.last_target.x
        twist = Twist()
        if abs(error_x) > ALIGN_THRESHOLD_PX:
            omega = max(-ALIGN_MAX_OMEGA, min(ALIGN_MAX_OMEGA, error_x * ALIGN_KP))
            twist.angular.z = omega
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            self.chase_cmd_log.append((twist.linear.x, twist.angular.z))
        else:
            self.state = 'APPROACH'
            self._approach_integral = 0.0
            self._approach_prev_error = 0.0
            self._approach_prev_time = None

    # ---- 狀態 C ----
    def _do_approach(self, now):
        if self.last_target is not None and self.last_target.z == 1.0:
            error_x = 320.0 - self.last_target.x
            dt = now - self._approach_prev_time if self._approach_prev_time is not None else 0.1
            dt = dt if 0 < dt < 0.5 else 0.1
            self._approach_integral += error_x * dt
            derivative = (error_x - self._approach_prev_error) / dt
            omega = (APPROACH_PID_KP * error_x
                     + APPROACH_PID_KI * self._approach_integral
                     + APPROACH_PID_KD * derivative)
            omega = max(-APPROACH_MAX_OMEGA, min(APPROACH_MAX_OMEGA, omega))
            self._approach_prev_error = error_x
            self._approach_prev_time = now

            twist = Twist()
            twist.linear.x = APPROACH_SPEED
            twist.angular.z = omega
            self.cmd_pub.publish(twist)
            self.chase_cmd_log.append((twist.linear.x, twist.angular.z))
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
            self.chase_cmd_log.append((twist.linear.x, twist.angular.z))
        else:
            self.vision_reset_pub.publish(Empty())  # 盲衝結束才放開視覺的目標鎖定
            self._start_return()

    # ---- 狀態 E：原路倒車 ----
    # 追球 (ALIGN/APPROACH/BLIND_DASH) 期間車體會偏離巡邏路線，而輪速里程計在這幾個
    # 狀態特別容易飄 (原地轉向、盲衝時滑動比較大)，飄了之後直接回去 PATROL 只會照著
    # 不準的里程計亂衝，可能撞牆。改成把追球期間送出的每個指令記下來，追完後原封不動
    # 反過來重播一次 (方向相反、順序倒過來)，不管里程計準不準，物理上大致就能退回追球
    # 之前的位置跟朝向，才切回 PATROL。
    def _start_return(self):
        if not RETURN_REPLAY_ENABLED:
            self.chase_cmd_log = []
            self.state = 'PATROL'
            return
        self.state = 'RETURNING'
        self.return_index = len(self.chase_cmd_log) - 1

    def _do_return(self):
        if self.return_index < 0:
            self.chase_cmd_log = []
            self.state = 'PATROL'
            return
        linear_x, angular_z = self.chase_cmd_log[self.return_index]
        self.return_index -= 1
        twist = Twist()
        twist.linear.x = -linear_x
        twist.angular.z = -angular_z
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GridPatrolNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
