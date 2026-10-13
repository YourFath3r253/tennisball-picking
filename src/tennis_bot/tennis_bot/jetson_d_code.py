"""把 D 同學 Jetson 上的程式「原封不動」載進模擬 (Software-in-the-loop)。

D 同學的檔案逐字複製在 src/tennis_bot/jetson_d/ (2026-10-12 從 Jetson /home/hp/models 抓回來，md5 跟 Jetson 相同)：
  realtime_camera_trt_distance_uart_court_line.py  主程式 (YOLO + 黃線邊界 + UART)
  yellow_runtime.py / geometry_features.py / edge_uart_gate.py / yellow_model.json  黃線邊界 (純 OpenCV)
  best_fold1_opset9_sim_512x384.onnx  YOLO 模型 (Jetson 上的 .engine 就是由這個 ONNX 轉的)

主程式 import 了 tensorrt / pycuda / serial (只有 Jetson 有)，不能直接 import。這裡用 ast 把主程式裡
「不碰硬體」的常數和函式原文取出來執行 (preprocess、parse_yolo_output、nms、estimate_ball_position、
ema_update…)，所以模擬用的是跟實體車同一份程式碼，不是重寫的版本。YOLO 推論用 onnxruntime 跑同一個 ONNX
(取代 TensorRT 的 infer())；黃線偵測直接 import D 同學的 yellow_runtime。

更新 D 同學的程式：把新檔案覆蓋到 jetson_d/ 就好 (主程式的函式名稱沒變就不用改這裡)。
"""
import ast
import os
import sys
from pathlib import Path

import numpy as np

D_DIR = Path(os.environ.get('TENNISBOT_JETSON_D_DIR', '/home/sean/ros2_ws/src/tennis_bot/jetson_d'))
D_MAIN = D_DIR / 'realtime_camera_trt_distance_uart_court_line.py'
D_ONNX = D_DIR / 'best_fold1_opset9_sim_512x384.onnx'

# 主程式裡要取出來用的函式 (都不碰硬體)
D_FUNCTIONS = ('preprocess', 'xywh_to_xyxy', 'iou', 'nms', 'parse_yolo_output',
               'estimate_ball_position', 'ema_update')
_HW_MODULES = ('tensorrt', 'pycuda', 'serial')


def load_d_main():
    """回傳一個 dict：D 同學主程式的常數 + D_FUNCTIONS 裡的函式 (原文執行)。"""
    if str(D_DIR) not in sys.path:
        sys.path.insert(0, str(D_DIR))
    tree = ast.parse(D_MAIN.read_text(encoding='utf-8'), filename=str(D_MAIN))
    keep = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            node.names = [a for a in node.names if a.name.split('.')[0] not in _HW_MODULES]
            if node.names:
                keep.append(node)
        elif isinstance(node, ast.ImportFrom):
            keep.append(node)           # yellow_runtime / edge_uart_gate (純 Python)
        elif isinstance(node, (ast.Assign, ast.AnnAssign)):
            keep.append(node)           # 常數 (COURT_EDGE_Y、BALL_INFER_PERIOD_SEC…)
        elif isinstance(node, ast.FunctionDef) and node.name in D_FUNCTIONS:
            keep.append(node)
    module = ast.Module(body=keep, type_ignores=[])
    ns = {'__file__': str(D_MAIN), '__name__': 'jetson_d_main'}
    exec(compile(module, str(D_MAIN), 'exec'), ns)
    missing = [f for f in D_FUNCTIONS if f not in ns]
    if missing:
        raise RuntimeError(f'D 同學主程式裡找不到這些函式：{missing}')
    return ns


class OnnxYolo:
    """取代 D 同學主程式的 TensorRT infer()：同一個 ONNX、同樣的輸入 (preprocess 的輸出)，輸出格式一樣。"""

    def __init__(self, path=D_ONNX):
        import onnxruntime as ort
        # 限制執行緒、關掉空轉：預設 16 條執行緒推論完還會空轉，跟同一個程序裡 ROS 收訊息的執行緒搶 CPU
        # (2026-10-12 實測：完整模擬時 Jetson 節點的訊息會間歇性卡住 ~10 s)
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4
        opts.inter_op_num_threads = 1
        opts.add_session_config_entry('session.intra_op.allow_spinning', '0')
        self.sess = ort.InferenceSession(str(path), sess_options=opts, providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name

    def infer(self, input_image):
        # D 同學的 infer() 回傳 [每個輸出 reshape 成 engine 的 shape]，這裡是 [(1, 5, 4032)]
        out = self.sess.run(None, {self.input_name: np.ascontiguousarray(input_image, dtype=np.float32)})
        return [out[0]]
