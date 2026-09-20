import os
import cv2
import time
import math
import csv
import serial
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit


ENGINE_PATH = "best_fold1_opset9_sim_512x384.engine"

INPUT_W = 512
INPUT_H = 384

CAM_W = 640
CAM_H = 480

CONF_THRES = 0.50
NMS_THRES = 0.45

BALL_DIAMETER_CM = 6.7
FOCAL_LENGTH_PX = 390.0
EMA_ALPHA = 0.35

# USB UART
UART_PORT = "/dev/ttyACM0"
UART_BAUD = 115200
UART_TIMEOUT = 0.2

# 偵測/漏偵測確認幀數；UART 指令格式不得更動
BALL_ON_CONFIRM_FRAMES = 2
BALL_OFF_CONFIRM_FRAMES = 5

# Court boundary V6
COURT_MODEL_PATH = "/home/hp/models/court_color_model_v6.npz"
COURT_EVERY_N_FRAMES = 3

# Hysteresis thresholds
COURT_EDGE_ENTER = 0.20
COURT_EDGE_EXIT = 0.10
COURT_OUT_ENTER = 0.55
COURT_OUT_EXIT = 0.40
COURT_CONFIRM_UPDATES = 5
COURT_WARMUP_SEC = 2.0

# Keep False until STM32 firmware can parse:
# COURT,<SAFE|EDGE|OUT>,<LEFT|CENTER|RIGHT|NONE>,<outside_ratio>
COURT_UART_ENABLE = False
COURT_UART_PERIOD_SEC = 0.5

TEST_NAME = os.environ.get("TEST_NAME", "distance_512x384_uart")
CSV_PATH = TEST_NAME + ".csv"
SAVE_LAST_IMAGE = TEST_NAME + ".jpg"
SAVE_VIDEO = TEST_NAME + ".mp4"

# 按下 s 時，儲存尚未畫框的相機原始畫面
FEEDBACK_DIR = "dataset_feedback"


def preprocess(frame):
    resized = cv2.resize(frame, (INPUT_W, INPUT_H))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    img = rgb.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    return np.ascontiguousarray(img), resized


def load_engine(engine_path):
    logger = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f:
        runtime = trt.Runtime(logger)
        return runtime.deserialize_cuda_engine(f.read())


def allocate_buffers(engine):
    inputs = []
    outputs = []
    bindings = []
    stream = cuda.Stream()

    for binding in engine:
        shape = tuple(engine.get_binding_shape(binding))
        dtype = trt.nptype(engine.get_binding_dtype(binding))
        size = int(np.prod(shape))

        host_mem = cuda.pagelocked_empty(size, dtype)
        device_mem = cuda.mem_alloc(host_mem.nbytes)
        bindings.append(int(device_mem))

        item = {
            "name": binding,
            "host": host_mem,
            "device": device_mem,
            "shape": shape,
            "dtype": dtype,
        }

        if engine.binding_is_input(binding):
            inputs.append(item)
        else:
            outputs.append(item)

    return inputs, outputs, bindings, stream


def infer(context, bindings, inputs, outputs, stream, input_image):
    np.copyto(inputs[0]["host"], input_image.ravel())
    cuda.memcpy_htod_async(inputs[0]["device"], inputs[0]["host"], stream)
    context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)

    for out in outputs:
        cuda.memcpy_dtoh_async(out["host"], out["device"], stream)

    stream.synchronize()
    return [out["host"].copy().reshape(out["shape"]) for out in outputs]


def xywh_to_xyxy(cx, cy, w, h):
    return (
        cx - w / 2.0,
        cy - h / 2.0,
        cx + w / 2.0,
        cy + h / 2.0,
    )


def iou(box1, box2):
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    inter = inter_w * inter_h

    area1 = max(0.0, box1[2] - box1[0]) * max(0.0, box1[3] - box1[1])
    area2 = max(0.0, box2[2] - box2[0]) * max(0.0, box2[3] - box2[1])

    union = area1 + area2 - inter
    return 0.0 if union <= 0 else inter / union


def nms(dets, nms_thres):
    if not dets:
        return []

    dets = sorted(dets, key=lambda x: x["conf"], reverse=True)
    keep = []

    while dets:
        best = dets.pop(0)
        keep.append(best)
        dets = [d for d in dets if iou(best["box"], d["box"]) < nms_thres]

    return keep


def parse_yolo_output(outputs_data):
    out = outputs_data[0]
    arr = np.squeeze(out)

    if arr.ndim != 2:
        arr = arr.reshape(-1, arr.shape[-1])

    if arr.shape[0] == 5:
        arr = arr.T

    dets = []

    for row in arr:
        if len(row) < 5:
            continue

        cx, cy, w, h, conf = map(float, row[:5])

        if conf < CONF_THRES:
            continue
        if w <= 2 or h <= 2:
            continue
        if w > INPUT_W or h > INPUT_H:
            continue

        x1, y1, x2, y2 = xywh_to_xyxy(cx, cy, w, h)

        x1 = max(0.0, min(INPUT_W - 1.0, x1))
        y1 = max(0.0, min(INPUT_H - 1.0, y1))
        x2 = max(0.0, min(INPUT_W - 1.0, x2))
        y2 = max(0.0, min(INPUT_H - 1.0, y2))

        dets.append({"box": [x1, y1, x2, y2], "conf": conf})

    return nms(dets, NMS_THRES)


def estimate_ball_position(det, focal_px):
    x1, y1, x2, y2 = det["box"]

    box_w = x2 - x1
    box_h = y2 - y1
    ball_pixel = math.sqrt(max(1.0, box_w * box_h))

    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0

    image_cx = 283.0
    image_cy = INPUT_H / 2.0

    z_cm = BALL_DIAMETER_CM * focal_px / ball_pixel
    x_cm = (cx - image_cx) * z_cm / focal_px
    y_cm = (cy - image_cy) * z_cm / focal_px

    dist_cm = math.sqrt(x_cm * x_cm + z_cm * z_cm)
    bearing_deg = math.degrees(math.atan2(x_cm, z_cm))

    return {
        "cx": cx,
        "cy": cy,
        "box_w": box_w,
        "box_h": box_h,
        "ball_pixel": ball_pixel,
        "z_cm": z_cm,
        "x_cm": x_cm,
        "y_cm": y_cm,
        "dist_cm": dist_cm,
        "bearing_deg": bearing_deg,
    }


def draw_result(frame512, det, pos, smooth_pos, motor_on):
    x1, y1, x2, y2 = det["box"]
    conf = det["conf"]

    x1i, y1i, x2i, y2i = map(int, (x1, y1, x2, y2))
    cv2.rectangle(frame512, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)

    cx = int(pos["cx"])
    cy = int(pos["cy"])
    cv2.circle(frame512, (cx, cy), 4, (0, 0, 255), -1)
    cv2.line(frame512, (INPUT_W // 2, INPUT_H), (cx, cy), (255, 0, 0), 2)

    text1 = "conf %.2f" % conf
    text2 = "Z %.1f cm  X %.1f cm" % (
        smooth_pos["z_cm"],
        smooth_pos["x_cm"],
    )
    text3 = "Dist %.1f cm  Angle %.1f deg" % (
        smooth_pos["dist_cm"],
        smooth_pos["bearing_deg"],
    )
    text4 = "px %.1f" % pos["ball_pixel"]
    text5 = "MOTOR: ON" if motor_on else "MOTOR: OFF"

    cv2.putText(frame512, text1, (x1i, max(20, y1i - 45)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame512, text2, (x1i, max(40, y1i - 25)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(frame512, text3, (x1i, max(60, y1i - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(frame512, text4, (10, INPUT_H - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame512, text5, (350, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0) if motor_on else (0, 0, 255), 2)

    return frame512


def ema_update(prev, current):
    if prev is None:
        return current.copy()

    out = {}
    for key, value in current.items():
        if isinstance(value, (int, float)):
            out[key] = EMA_ALPHA * value + (1.0 - EMA_ALPHA) * prev[key]
        else:
            out[key] = value
    return out


def open_uart():
    print("Opening UART:", UART_PORT)
    ser = serial.Serial(
        port=UART_PORT,
        baudrate=UART_BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=UART_TIMEOUT,
    )
    time.sleep(2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print("UART ready:", UART_PORT, UART_BAUD)
    return ser


def send_command(ser, command):
    message = (command + "\n").encode("utf-8")
    ser.write(message)
    ser.flush()
    print("UART ->", command)

    response = ser.readline()
    if response:
        print("STM32 ->", response.decode("utf-8", errors="replace").strip())
    else:
        print("STM32 -> no response")



def court_patch_feature(patch):
    """Feature vector used by the trained V6 court model."""
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB).astype(np.float32)
    bgr = patch.astype(np.float32)

    b = bgr[:, :, 0]
    g = bgr[:, :, 1]
    r = bgr[:, :, 2]

    s = hsv[:, :, 1]
    v = hsv[:, :, 2]

    white = (s < 35) & (v > 180)
    reflection = v > 245
    dark = v < 25
    valid = ~(white | reflection | dark)

    if np.count_nonzero(valid) < valid.size * 0.35:
        return None

    denom = r + g + b + 1.0

    return np.array([
        np.median(b[valid] / denom[valid]),
        np.median(g[valid] / denom[valid]),
        np.median(r[valid] / denom[valid]),
        np.median(s[valid]) / 255.0,
        np.median((lab[:, :, 1][valid] - 128.0) / 127.0),
        np.median((lab[:, :, 2][valid] - 128.0) / 127.0),
    ], dtype=np.float32)


def load_court_model(path):
    d = np.load(path)

    return {
        "weights": d["weights"],
        "th_inside": d["th_inside"],
        "th_outside": d["th_outside"],
        "valid_cell": d["valid_cell"],
        "separation": d["separation"],
        "h": int(d["h"][0]),
        "w": int(d["w"][0]),
        "y0": int(d["y0"][0]),
        "y1": int(d["y1"][0]),
        "patch": int(d["patch"][0]),
    }


def remove_isolated_court_outside(class_grid, min_size=3):
    binary = (class_grid == 1).astype(np.uint8)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        binary,
        connectivity=8,
    )

    clean = np.zeros_like(binary)

    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]

        if area >= min_size:
            clean[labels == i] = 1

    return clean


def classify_court_frame(frame, model):
    """Return grid, cleaned OUT mask, OUT ratio, known ratio, direction."""
    h, w = frame.shape[:2]

    if h != model["h"] or w != model["w"]:
        raise RuntimeError(
            "Court model expects %dx%d, camera returned %dx%d"
            % (model["w"], model["h"], w, h)
        )

    patch = model["patch"]
    y0 = model["y0"]
    y1 = model["y1"]

    xs = list(range(0, w - patch + 1, patch))
    ys = list(range(y0, y1 - patch + 1, patch))

    rows = len(ys)
    cols = len(xs)

    # -1 UNKNOWN, 0 INSIDE, 1 OUTSIDE
    grid = np.full((rows, cols), -1, dtype=np.int8)

    for ry, y in enumerate(ys):
        for cx, x in enumerate(xs):
            if model["valid_cell"][ry, cx] == 0:
                continue

            p = frame[y:y + patch, x:x + patch]
            f = court_patch_feature(p)

            if f is None:
                continue

            score = float(np.dot(f, model["weights"][ry, cx]))

            if score >= model["th_outside"][ry, cx]:
                grid[ry, cx] = 1
            elif score <= model["th_inside"][ry, cx]:
                grid[ry, cx] = 0

    clean_out = remove_isolated_court_outside(grid, min_size=3)

    inside = (grid == 0)
    known = inside | (clean_out == 1)

    known_count = int(np.count_nonzero(known))
    out_count = int(np.count_nonzero(clean_out))

    if known_count > 0:
        outside_ratio = out_count / float(known_count)
    else:
        outside_ratio = 0.0

    known_ratio = known_count / float(rows * cols)

    direction = "NONE"
    pos = np.argwhere(clean_out == 1)

    if len(pos) > 0:
        mean_col = float(np.mean(pos[:, 1]))

        if mean_col < cols / 3.0:
            direction = "LEFT"
        elif mean_col > 2.0 * cols / 3.0:
            direction = "RIGHT"
        else:
            direction = "CENTER"

    return grid, clean_out, outside_ratio, known_ratio, direction


def update_court_state(
    state,
    pending_target,
    pending_count,
    smooth_ratio,
    known_ratio,
):
    """Update SAFE / EDGE / OUT with hysteresis and confirmation."""
    if known_ratio < 0.25:
        return state, pending_target, pending_count, False

    if state == "SAFE":
        target = "EDGE" if smooth_ratio >= COURT_EDGE_ENTER else "SAFE"

    elif state == "EDGE":
        if smooth_ratio >= COURT_OUT_ENTER:
            target = "OUT"
        elif smooth_ratio <= COURT_EDGE_EXIT:
            target = "SAFE"
        else:
            target = "EDGE"

    else:  # OUT
        target = "EDGE" if smooth_ratio <= COURT_OUT_EXIT else "OUT"

    changed = False

    if target != state:
        if pending_target == target:
            pending_count += 1
        else:
            pending_target = target
            pending_count = 1

        if pending_count >= COURT_CONFIRM_UPDATES:
            state = target
            pending_target = None
            pending_count = 0
            changed = True
    else:
        pending_target = None
        pending_count = 0

    return state, pending_target, pending_count, changed


def draw_court_status(
    frame512,
    state,
    direction,
    smooth_ratio,
    raw_ratio,
    known_ratio,
):
    if state == "SAFE":
        color = (0, 255, 0)
    elif state == "EDGE":
        color = (0, 255, 255)
    else:
        color = (0, 0, 255)

    cv2.putText(
        frame512,
        "COURT %s  %s" % (state, direction),
        (10, 325),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
    )
    cv2.putText(
        frame512,
        "OUT %.1f%% raw %.1f%% known %.1f%%"
        % (smooth_ratio * 100.0, raw_ratio * 100.0, known_ratio * 100.0),
        (10, 350),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.47,
        (255, 255, 255),
        1,
    )

    return frame512

def main():
    ser = None
    cap = None
    video_writer = None
    csv_file = None

    try:
        ser = open_uart()

        # 啟動時先確保馬達停止
        send_command(ser, "BALL_OFF")

        print("Loading TensorRT engine...")
        engine = load_engine(ENGINE_PATH)
        context = engine.create_execution_context()
        inputs, outputs, bindings, stream = allocate_buffers(engine)

        print("Engine:", ENGINE_PATH)
        print("Input:", INPUT_W, "x", INPUT_H)
        print("Focal length px:", FOCAL_LENGTH_PX)
        print("Ball diameter cm:", BALL_DIAMETER_CM)

        print("Loading court V6 model...")
        court_model = load_court_model(COURT_MODEL_PATH)
        print("Court model:", COURT_MODEL_PATH)

        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
        cap.set(cv2.CAP_PROP_FPS, 30)

        if not cap.isOpened():
            raise RuntimeError("Cannot open camera")

        print("Camera width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        print("Camera height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print("Camera FPS setting:", cap.get(cv2.CAP_PROP_FPS))

        smooth_pos = None
        frame_count = 0
        detect_count = 0
        last_output_frame = None

        motor_on = False
        detected_streak = 0
        missing_streak = 0

        # Court runtime state
        court_state = "SAFE"
        court_direction = "NONE"
        court_raw_ratio = 0.0
        court_smooth_ratio = 0.0
        court_known_ratio = 0.0
        court_smooth_started = False
        court_pending_target = None
        court_pending_count = 0
        court_last_uart_time = 0.0
        court_alpha = 0.20

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(
            SAVE_VIDEO,
            fourcc,
            20.0,
            (INPUT_W, INPUT_H),
        )

        csv_file = open(CSV_PATH, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow([
            "frame",
            "time",
            "conf",
            "cx",
            "cy",
            "box_w",
            "box_h",
            "ball_pixel",
            "z_cm",
            "x_cm",
            "y_cm",
            "dist_cm",
            "bearing_deg",
            "smooth_z_cm",
            "smooth_x_cm",
            "smooth_dist_cm",
            "smooth_bearing_deg",
            "motor_on",
        ])

        total_t0 = time.time()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Cannot read frame")
                break

            # 保留尚未 resize、尚未畫框的原始相機畫面
            raw_frame = frame.copy()

            frame_count += 1
            t_now = time.time()

            # ---------------------------------------------------------
            # Court V6 boundary detection on original 640x480 frame
            # ---------------------------------------------------------
            court_changed = False

            if (
                (t_now - total_t0) >= COURT_WARMUP_SEC
                and frame_count % COURT_EVERY_N_FRAMES == 0
            ):
                (
                    _court_grid,
                    _court_clean_out,
                    court_raw_ratio,
                    court_known_ratio,
                    court_direction,
                ) = classify_court_frame(raw_frame, court_model)

                if not court_smooth_started:
                    court_smooth_ratio = court_raw_ratio
                    court_smooth_started = True
                else:
                    court_smooth_ratio = (
                        court_alpha * court_raw_ratio
                        + (1.0 - court_alpha) * court_smooth_ratio
                    )

                (
                    court_state,
                    court_pending_target,
                    court_pending_count,
                    court_changed,
                ) = update_court_state(
                    court_state,
                    court_pending_target,
                    court_pending_count,
                    court_smooth_ratio,
                    court_known_ratio,
                )

                court_command = "COURT,%s,%s,%.2f" % (
                    court_state,
                    court_direction,
                    court_smooth_ratio,
                )

                if COURT_UART_ENABLE:
                    if (
                        court_changed
                        or (t_now - court_last_uart_time) >= COURT_UART_PERIOD_SEC
                    ):
                        send_command(ser, court_command)
                        court_last_uart_time = t_now

                if court_changed or frame_count % (COURT_EVERY_N_FRAMES * 10) == 0:
                    print(
                        court_command,
                        "| raw %.2f" % court_raw_ratio,
                        "| known %.2f" % court_known_ratio,
                    )

            input_image, frame512 = preprocess(frame)
            outputs_data = infer(
                context,
                bindings,
                inputs,
                outputs,
                stream,
                input_image,
            )
            dets = parse_yolo_output(outputs_data)
            print("Detected balls:", len(dets))

            if dets:
                detected_streak += 1
                missing_streak = 0

                candidates = []

                for d in dets:
                    p = estimate_ball_position(d, FOCAL_LENGTH_PX)
                    candidates.append((d, p))

                # 選擇距離最近的球
                det, pos = min(
                    candidates,
                    key=lambda item: item[1]["dist_cm"]
                )

                print(
                    "Nearest ball: %.1f cm, angle %.1f deg"
                    % (pos["dist_cm"], pos["bearing_deg"])
                )

                smooth_pos = ema_update(smooth_pos, pos)
                detect_count += 1

                # UART 格式固定維持：
                # BALL,<distance>,<angle>
                # BALL_OFF
                if detected_streak >= BALL_ON_CONFIRM_FRAMES:
                    motor_on = True
                    command = "BALL,%.1f,%.1f" % (
                        smooth_pos["dist_cm"],
                        smooth_pos["bearing_deg"],
                    )
                    send_command(ser, command)

                frame512 = draw_result(
                    frame512,
                    det,
                    pos,
                    smooth_pos,
                    motor_on,
                )

                writer.writerow([
                    frame_count,
                    "%.3f" % (t_now - total_t0),
                    "%.4f" % det["conf"],
                    "%.2f" % pos["cx"],
                    "%.2f" % pos["cy"],
                    "%.2f" % pos["box_w"],
                    "%.2f" % pos["box_h"],
                    "%.2f" % pos["ball_pixel"],
                    "%.2f" % pos["z_cm"],
                    "%.2f" % pos["x_cm"],
                    "%.2f" % pos["y_cm"],
                    "%.2f" % pos["dist_cm"],
                    "%.2f" % pos["bearing_deg"],
                    "%.2f" % smooth_pos["z_cm"],
                    "%.2f" % smooth_pos["x_cm"],
                    "%.2f" % smooth_pos["dist_cm"],
                    "%.2f" % smooth_pos["bearing_deg"],
                    int(motor_on),
                ])

                if frame_count % 10 == 0:
                    print(
                        "frame", frame_count,
                        "| conf %.2f" % det["conf"],
                        "| Z %.1f cm" % smooth_pos["z_cm"],
                        "| X %.1f cm" % smooth_pos["x_cm"],
                        "| Dist %.1f cm" % smooth_pos["dist_cm"],
                        "| Angle %.1f deg" % smooth_pos["bearing_deg"],
                        "| Motor", "ON" if motor_on else "OFF",
                    )

            else:
                detected_streak = 0
                missing_streak += 1

                if motor_on and missing_streak >= BALL_OFF_CONFIRM_FRAMES:
                    send_command(ser, "BALL_OFF")
                    motor_on = False

                cv2.putText(
                    frame512,
                    "No ball detected",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                cv2.putText(
                    frame512,
                    "MOTOR: ON" if motor_on else "MOTOR: OFF",
                    (350, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0) if motor_on else (0, 0, 255),
                    2,
                )

            frame512 = draw_court_status(
                frame512,
                court_state,
                court_direction,
                court_smooth_ratio,
                court_raw_ratio,
                court_known_ratio,
            )

            last_output_frame = frame512
            video_writer.write(frame512)



    except KeyboardInterrupt:
        print("Stopped by user")
    except serial.SerialException as error:
        print("UART ERROR:", error)
    except Exception as error:
        print("ERROR:", error)

    finally:
        # 無論如何結束，都嘗試關閉馬達
        if ser is not None and ser.is_open:
            try:
                send_command(ser, "BALL_OFF")
            except Exception:
                pass

        if cap is not None:
            cap.release()

        if video_writer is not None:
            video_writer.release()

        if csv_file is not None:
            csv_file.close()

        if "last_output_frame" in locals() and last_output_frame is not None:
            cv2.imwrite(SAVE_LAST_IMAGE, last_output_frame)
            print("Saved last image:", SAVE_LAST_IMAGE)

        if "frame_count" in locals():
            total_t1 = time.time()
            print("")
            print("Total frames:", frame_count)
            print("Detection count:", detect_count, "/", frame_count)

            if frame_count > 0:
                print(
                    "Detection rate: %.2f%%"
                    % (100.0 * detect_count / frame_count)
                )
                print(
                    "End-to-end FPS: %.2f"
                    % (frame_count / (total_t1 - total_t0))
                )

            print("CSV saved:", CSV_PATH)
            print("Video saved:", SAVE_VIDEO)

        if ser is not None and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()