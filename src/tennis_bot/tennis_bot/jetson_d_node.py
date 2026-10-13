"""模擬裡的 Jetson：跑 D 同學 Jetson 上的同一份程式 (2026-10-12 版，realtime_camera_trt_distance_uart_court_line.py)。

  Gazebo 相機 (640x480, 30 Hz) -> [相機緩衝區] -> D 同學主迴圈 (黃線邊界 + YOLO) -> /jetson_uart (UART 字串)

跟實體車一樣的部分 (程式碼原文，見 jetson_d_code.py)：
  - YOLO：同一個 ONNX 模型、preprocess / parse_yolo_output / nms / estimate_ball_position / ema_update 原文
  - 黃線邊界：直接 import D 同學的 yellow_runtime (YellowRuntime) 跟 edge_uart_gate (EdgeUartGate)
  - 主迴圈邏輯照抄：每一輪先做 COURT (間隔 >= COURT_PERIOD_SEC)，YOLO 間隔 >= BALL_INFER_PERIOD_SEC 才跑；
    連續 2 次偵測到才送 BALL、連續 5 次 YOLO 沒看到才送 BALL_OFF；EMA 不會在看丟球時重置 (D 程式就是這樣)；
    COURT 每次碰到黃線只送一次 EDGE (EdgeUartGate)，SAFE 每 0.25 s 心跳一次
  - 開機先送 BALL_OFF 跟 COURT,SAFE,NONE,UNKNOWN

模擬才需要的 (Jetson Nano 的速度，筆電算得比較快，所以用實測時間)：
  - 每一輪花多久：從 10/11 實測 CSV 抽樣 (有跑 YOLO 的那輪 中位數 0.21 s，沒跑的 0.12 s；分布見 ITER_Q)，
    用模擬時間排程，字串在「實體車上會送出的時間」才發布
  - 相機緩衝區 (OpenCV V4L2 預設 4 個 buffer)：迴圈比相機 30 fps 慢時，cap.read() 拿到的是緩衝區裡最舊的那張
    -> 畫面會比「現在」舊大約 3 輪迴圈。這是推論 (沒有直接量過)：舊程式 0.11/0.11/0.45 s 的節奏用這個模型
    算出來大約 0.6~0.7 s，跟 9/21 擬合出來的畫面延遲 0.60 s 吻合。TENNISBOT_V4L2_BUFFERS 可以改。

時間來源：/joint_states 封包時間戳 (100 Hz 模擬時間)，跟 motor_driver_node / stm32_b_node 同一套。
輸出：/jetson_uart (std_msgs/String)；D 同學格式的 CSV (<TEST_NAME>.csv、<TEST_NAME>_court.csv) 寫到
TENNISBOT_JETSON_LOG_DIR (run_real_car.sh 會在結束後搬進 runN 資料夾)。
"""
import bisect
import csv
import os
import random
import time
from collections import deque
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from tennis_bot.jetson_d_code import D_DIR, OnnxYolo, load_d_main

V4L2_BUFFERS = int(os.environ.get('TENNISBOT_V4L2_BUFFERS', '4'))
LOG_DIR = os.environ.get('TENNISBOT_JETSON_LOG_DIR', '')
SEED = int(os.environ.get('TENNISBOT_JETSON_SEED', '0'))

# 10/11 實測 (yellow_* + distance_512x384_uart，D 程式現行版本) 每一輪迴圈的時間分位數 (0,5,...,100%)：
# 同一輪的 court 列跟 ball 列時間戳相同，用這個把「有跑 YOLO 並偵測到球」的那輪分出來 (n=118)，其他輪 n=1800
# (其他輪包含「有跑 YOLO 但沒看到球」，CSV 分不出來；run163 驗證：這樣分 COURT 間隔才會跟實測一樣)。
ITER_Q_YOLO = [0.122, 0.153, 0.158, 0.164, 0.172, 0.179, 0.195, 0.2, 0.204, 0.206, 0.209, 0.214, 0.218, 0.227,
               0.234, 0.24, 0.245, 0.251, 0.264, 0.272, 0.308]
ITER_Q_OTHER = [0.05, 0.053, 0.058, 0.063, 0.069, 0.072, 0.076, 0.082, 0.091, 0.104, 0.12, 0.135, 0.142, 0.151,
                0.156, 0.161, 0.167, 0.175, 0.189, 0.213, 0.506]
# court 那段處理時間 (CSV 的 process_ms)，n=1927
COURT_PROC_Q = [0.0027, 0.0034, 0.0044, 0.0092, 0.0149, 0.024, 0.0292, 0.0344, 0.0399, 0.0441, 0.0471, 0.0501,
                0.0535, 0.0568, 0.0607, 0.0672, 0.074, 0.082, 0.0924, 0.112, 0.3048]


def sample_q(rng, q):
    """從分位數表抽樣 (分位數之間線性內插)。"""
    u = rng.random() * (len(q) - 1)
    i = int(u)
    if i >= len(q) - 1:
        return q[-1]
    return q[i] + (q[i + 1] - q[i]) * (u - i)


class JetsonDNode(Node):
    def __init__(self):
        super().__init__('jetson_d_node')
        self.d = load_d_main()       # D 主程式的常數 + 函式 (原文)；也把 D_DIR 放進 sys.path
        from yellow_runtime import YellowRuntime   # noqa: E402  (D 同學原檔)
        from edge_uart_gate import EdgeUartGate    # noqa: E402
        d = self.d
        self.yolo = OnnxYolo()
        self.court_runtime = YellowRuntime(d['COURT_YELLOW_MODEL_PATH'], edge_y=d['COURT_EDGE_Y'],
                                           edge_confirm_frames=d['COURT_EDGE_CONFIRM_FRAMES'])
        self.court_gate = EdgeUartGate(d['COURT_EDGE_COOLDOWN_SEC'], d['COURT_EDGE_CLEAR_SEC'])
        self.rng = random.Random(SEED)
        self.bridge = CvBridge()
        self.uart_pub = self.create_publisher(String, '/jetson_uart', 50)
        # 影像用 best-effort (相機 topic 的標準做法)：reliable 傳 900 KB 的影像，訂閱端一慢就會卡在重傳
        self.create_subscription(Image, '/tennis_camera/image_raw', self._image_cb, qos_profile_sensor_data)
        # 時鐘只要最新的：best-effort、不排隊 (排隊的舊時間戳會讓時鐘卡住，見 _image_cb)
        self.create_subscription(JointState, '/joint_states', self._tick_cb, qos_profile_sensor_data)

        # ---- 相機緩衝區 (V4L2) ----
        self.done_queue = deque()    # (capture_stamp, bgr)：相機拍好、等 cap.read() 拿的
        self.held = False            # 程式手上拿著一個 buffer (處理中，下次 read 才還回去)
        # ---- D 主迴圈狀態 (變數名稱照 D 程式) ----
        self.frame_count = 0
        self.total_t0 = None
        self.smooth_pos = None
        self.motor_on = False
        self.detected_streak = 0
        self.missing_streak = 0
        self.court_last_process_time = 0.0
        self.court_last_uart_time = 0.0
        self.court_last_packet = None
        self.last_ball_infer_time = 0.0
        # ---- 模擬排程 ----
        self.now = None
        self.next_iter_t = None      # 下一輪迴圈開始 (= 下一次 cap.read()) 的模擬時間
        self.waiting_frame = False
        self.outbox = []             # (發送時間, 字串)
        self._dbg = {'t': 0.0, 'img': 0, 'drop': 0}
        self._trace = deque(maxlen=400)
        self._trace_dumped = False
        self.frame_ages = []

        self.ball_csv = self.court_csv = None
        if LOG_DIR:
            Path(LOG_DIR).mkdir(parents=True, exist_ok=True)
            name = os.environ.get('TEST_NAME', 'sim')
            self._files = [open(Path(LOG_DIR) / f'{name}.csv', 'w', newline=''),
                           open(Path(LOG_DIR) / f'{name}_court.csv', 'w', newline='')]
            self.ball_csv = csv.writer(self._files[0])
            self.ball_csv.writerow(['frame', 'time', 'conf', 'cx', 'cy', 'box_w', 'box_h', 'ball_pixel', 'z_cm',
                                    'x_cm', 'y_cm', 'dist_cm', 'bearing_deg', 'smooth_z_cm', 'smooth_x_cm',
                                    'smooth_dist_cm', 'smooth_bearing_deg', 'motor_on', 'sim_stamp', 'frame_age'])
            self.court_csv = csv.writer(self._files[1])
            self.court_csv.writerow(['frame', 'time_s', 'packet', 'state', 'line_side', 'color', 'near_y_norm',
                                     'confirm_count', 'position_state', 'color_status', 'raw_label',
                                     'raw_confidence', 'process_ms', 'sim_stamp', 'frame_age'])
        self.get_logger().info(
            f'Jetson (D 同學程式 SIL)：{D_DIR.name}，COURT_EDGE_Y={d["COURT_EDGE_Y"]}，'
            f'BALL_INFER_PERIOD={d["BALL_INFER_PERIOD_SEC"]}s，V4L2 buffers={V4L2_BUFFERS}')

    # ---------------- 相機 ----------------
    def _image_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # 健康檢查：Gazebo 相機斷掉 (例如筆電闔上、螢幕關掉時算圖被降頻) 會讓模擬失真，記下來
        last = getattr(self, '_last_img_stamp', None)
        if last is not None and stamp - last > 1.0:
            self.camera_stalls = getattr(self, 'camera_stalls', 0) + 1
            self.get_logger().warning(f'相機畫面中斷 {stamp - last:.1f}s (模擬時間 {last:.1f} -> {stamp:.1f})，這段模擬不可信')
        self._last_img_stamp = stamp
        free = V4L2_BUFFERS - len(self.done_queue) - (1 if self.held else 0)
        self._dbg['img'] += 1
        if free > 0:
            self.done_queue.append((stamp, self.bridge.imgmsg_to_cv2(msg, 'bgr8')))
            if self.waiting_frame:
                self.waiting_frame = False
                self._iteration(stamp)
        else:
            self._dbg['drop'] += 1   # 所有 buffer 都滿了：驅動丟掉這張 (實體相機也是)
        # 相機時間戳也當時鐘 (而且不管這張有沒有被丟掉都要推進)。run179 黑盒子：程序卡 1.5 s 之後
        # /joint_states 在佇列裡排隊 (比最新畫面舊 0.5 s)，如果被丟掉的畫面不推進時鐘，兩個時鐘都動不了 -> 停 ~10 s
        self._advance(stamp, 'img')

    # ---------------- 模擬時鐘 ----------------
    def _tick_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        last = getattr(self, '_last_tick_stamp', None)
        if last is not None and stamp - last > 1.0:
            self.get_logger().warning(f'/joint_states 中斷 {stamp - last:.1f}s (模擬時間 {last:.1f} -> {stamp:.1f})')
        self._last_tick_stamp = stamp
        self._advance(stamp, 'tick')

    def _advance(self, stamp, src='?'):
        # 黑盒子：最近 400 次呼叫，第一次落後 > 1 s 時寫檔 (追 run169/175 的 ~10 s 停頓)
        self._trace.append((time.time(), src, stamp, self.now, self.next_iter_t, self.waiting_frame,
                            len(self.done_queue), self.held))
        if (not self._trace_dumped and self.now is not None and self.next_iter_t is not None
                and self.now - self.next_iter_t > 1.0 and LOG_DIR):
            self._trace_dumped = True
            with open(Path(LOG_DIR) / 'jetson_stall_trace.csv', 'w') as f:
                f.write('wall,src,stamp,now,next_iter_t,waiting,queue,held\n')
                for row in self._trace:
                    f.write(','.join(str(v) for v in row) + '\n')
            self.get_logger().warning(f'Jetson 迴圈落後 {self.now - self.next_iter_t:.1f}s，已寫 jetson_stall_trace.csv')
        if self.now is not None and stamp <= self.now:
            return
        self.now = stamp
        if os.environ.get('TENNISBOT_JETSON_DEBUG') and stamp - self._dbg['t'] >= 1.0:
            self.get_logger().info(f"DBG now={stamp:.2f} next={self.next_iter_t} wait={self.waiting_frame} "
                                   f"held={self.held} q={[round(x[0], 2) for x in self.done_queue]} "
                                   f"img={self._dbg['img']} drop={self._dbg['drop']} it={self.frame_count}")
            self._dbg['t'] = stamp
        if self.next_iter_t is None:
            # 開機：先送這兩行 (D 程式 main() 一開始就送)
            self._send('BALL_OFF', self.now)
            self._send('COURT,SAFE,NONE,UNKNOWN', self.now)
            self.next_iter_t = self.now
        self._flush_outbox()
        n = 0
        while not self.waiting_frame and self.now >= self.next_iter_t and n < 10:
            self._iteration(self.next_iter_t)   # 落後時一次補跑 (每輪消耗一張緩衝區的畫面)
            n += 1

    def _send(self, line, t):
        self.outbox.append((t, line))

    def _flush_outbox(self):
        if not self.outbox:
            return
        self.outbox.sort(key=lambda x: x[0])
        while self.outbox and self.outbox[0][0] <= self.now + 1e-9:
            _, line = self.outbox.pop(0)
            self.uart_pub.publish(String(data=line))

    # ---------------- D 主迴圈的一輪 ----------------
    def _iteration(self, t_start):
        # cap.read()：先把手上的 buffer 還回去，再拿最舊的一張
        self.held = False
        if not self.done_queue:
            self.waiting_frame = True
            return
        if self.done_queue[0][0] > t_start:
            t_start = self.done_queue[0][0]   # 那時候緩衝區其實是空的：read() 等到這張拍好才回來
        cap_stamp, frame = self.done_queue.popleft()
        self.held = True
        t_now = t_start
        if self.total_t0 is None:
            self.total_t0 = t_now
        self.frame_count += 1
        d = self.d
        age = t_now - cap_stamp
        self.frame_ages.append(age)

        court_ms = 0.0
        if (t_now - self.court_last_process_time) >= d['COURT_PERIOD_SEC']:
            court_ms = sample_q(self.rng, COURT_PROC_Q)
            _w0 = time.time()
            res = self.court_runtime.observe(frame)
            _wc = time.time() - _w0
            if _wc > 0.3:
                self.get_logger().warning(f'黃線偵測花了 {_wc:.2f}s (真實時間)，模擬時間 {t_now:.2f}')
            self.court_last_process_time = t_now
            cmd = res.packet()
            allowed = self.court_gate.allow(res.observation.state, t_now)
            if allowed and (cmd != self.court_last_packet or res.observation.state == 'EDGE'
                            or (t_now - self.court_last_uart_time) >= d['COURT_UART_HEARTBEAT_SEC']):
                self._send(cmd, t_now + court_ms)
                self.court_last_packet = cmd
                self.court_last_uart_time = t_now
            if self.court_csv:
                o = res.observation
                self.court_csv.writerow([
                    self.frame_count, f'{t_now - self.total_t0:.3f}', cmd, o.state, o.line_side, o.color,
                    '' if o.near_y_norm is None else f'{o.near_y_norm:.5f}', o.near_confirm_count,
                    res.position_state, res.color_status, res.raw_label, f'{res.raw_confidence:.4f}',
                    f'{court_ms * 1000:.1f}', f'{t_now:.3f}', f'{age:.3f}'])

        yolo_ran = False
        if (t_now - self.last_ball_infer_time) >= d['BALL_INFER_PERIOD_SEC']:
            yolo_ran = True
            self.last_ball_infer_time = t_now
            _w0 = time.time()
            input_image, _ = d['preprocess'](frame)
            dets = d['parse_yolo_output'](self.yolo.infer(input_image))
            _wy = time.time() - _w0
            if _wy > 0.3:
                self.get_logger().warning(f'YOLO 花了 {_wy:.2f}s (真實時間)，偵測 {len(dets)} 個，模擬時間 {t_now:.2f}')
            # 迴圈時間的實測分組是「同一輪有 ball 列 (YOLO 有偵測到)」vs 其他，所以沒偵測到的 YOLO 輪用其他那組
            dur = sample_q(self.rng, ITER_Q_YOLO if dets else ITER_Q_OTHER)
            t_ball = t_now + max(court_ms, 0.9 * dur)
            if dets:
                self.detected_streak += 1
                self.missing_streak = 0
                cands = [(det, d['estimate_ball_position'](det, d['FOCAL_LENGTH_PX'])) for det in dets]
                det, pos = min(cands, key=lambda item: item[1]['dist_cm'])
                self.smooth_pos = d['ema_update'](self.smooth_pos, pos)
                if self.detected_streak >= d['BALL_ON_CONFIRM_FRAMES']:
                    self.motor_on = True
                    self._send('BALL,%.1f,%.1f' % (self.smooth_pos['dist_cm'], self.smooth_pos['bearing_deg']),
                               t_ball)
                if self.ball_csv:
                    sp = self.smooth_pos
                    self.ball_csv.writerow([
                        self.frame_count, f'{t_now - self.total_t0:.3f}', f'{det["conf"]:.4f}',
                        f'{pos["cx"]:.2f}', f'{pos["cy"]:.2f}', f'{pos["box_w"]:.2f}', f'{pos["box_h"]:.2f}',
                        f'{pos["ball_pixel"]:.2f}', f'{pos["z_cm"]:.2f}', f'{pos["x_cm"]:.2f}', f'{pos["y_cm"]:.2f}',
                        f'{pos["dist_cm"]:.2f}', f'{pos["bearing_deg"]:.2f}', f'{sp["z_cm"]:.2f}',
                        f'{sp["x_cm"]:.2f}', f'{sp["dist_cm"]:.2f}', f'{sp["bearing_deg"]:.2f}',
                        int(self.motor_on), f'{t_now:.3f}', f'{age:.3f}'])
            else:
                self.detected_streak = 0
                self.missing_streak += 1
                if self.motor_on and self.missing_streak >= d['BALL_OFF_CONFIRM_FRAMES']:
                    self._send('BALL_OFF', t_ball)
                    self.motor_on = False
        else:
            dur = sample_q(self.rng, ITER_Q_OTHER)
        dur = max(dur, court_ms + 0.002)
        self.next_iter_t = t_now + dur
        if self.frame_count % 10 == 0 and self.court_csv:
            for f in self._files:
                f.flush()   # 節點被 kill 時才不會掉最後幾秒
        if self.frame_count % 200 == 0:
            ages = sorted(self.frame_ages[-200:])
            self.get_logger().info(f'畫面年齡 (最近 200 輪) 中位數 {ages[100]:.3f}s，p90 {ages[180]:.3f}s'
                                   f'{"" if yolo_ran else ""}')


def main(args=None):
    rclpy.init(args=args)
    node = JetsonDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
