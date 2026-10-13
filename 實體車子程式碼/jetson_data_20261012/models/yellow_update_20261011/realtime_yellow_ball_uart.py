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
from yellow_runtime import YellowRuntime as CourtLineRuntime


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

# Yellow-only marker detection, using a binary model at 320x240.
# EDGE threshold remains provisional until measured on the actual vehicle.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
COURT_YELLOW_MODEL_PATH = os.environ.get(
    "COURT_YELLOW_MODEL_PATH",
    os.path.join(SCRIPT_DIR, "yellow_model.json"),
)

# Image-space trigger inherited from the installed program, not centimeters.
# Must be revalidated with the yellow-only detector and actual driving speed.
COURT_EDGE_Y = 0.48
COURT_EDGE_CONFIRM_FRAMES = 2
COURT_PERIOD_SEC = 0.05
BALL_INFER_PERIOD_SEC = 0.20
COURT_UART_HEARTBEAT_SEC = 0.25

TEST_NAME = os.environ.get("TEST_NAME", "distance_512x384_uart")
CSV_PATH = TEST_NAME + ".csv"
SAVE_LAST_IMAGE = TEST_NAME + ".jpg"
SAVE_VIDEO = TEST_NAME + ".mp4"
# [Sean 2026-09-20] STM32每50ms送一次的實際/目標輪子轉速debug telemetry
RPM_CSV_PATH = TEST_NAME + "_rpm.csv"
COURT_CSV_PATH = TEST_NAME + "_court.csv"

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
    """Write one UART line without blocking the vision loop for an ACK."""
    message = (command + "\n").encode("utf-8")
    ser.write(message)
    ser.flush()
    print("UART ->", command)


def drain_rpm_telemetry(ser, rpm_writer, t0):
    """[Sean 2026-09-20] 把STM32每50ms主動送來的RPM,...telemetry行全部讀出來寫進CSV。
    每個frame呼叫一次，讀光目前緩衝區裡累積的所有行，避免累積延遲。"""
    while ser.in_waiting > 0:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if not line.startswith("RPM,"):
            continue
        parts = line.split(",")
        if len(parts) != 6:
            continue
        rpm_writer.writerow([time.time() - t0] + parts[1:])



def draw_court_line_status(frame512, court_result):
    """Draw the new yarn-boundary status on the saved 512x384 debug video."""
    if court_result is None:
        cv2.putText(
            frame512,
            "COURT waiting for first frame",
            (10, 325),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            1,
        )
        return frame512

    observation = court_result.observation
    packet = observation.packet()
    color = (0, 0, 255) if observation.state == "EDGE" else (0, 200, 0)

    if court_result.line is not None:
        x1, y1, x2, y2 = court_result.line
        cv2.line(
            frame512,
            (int(x1 * INPUT_W / float(CAM_W)), int(y1 * INPUT_H / float(CAM_H))),
            (int(x2 * INPUT_W / float(CAM_W)), int(y2 * INPUT_H / float(CAM_H))),
            color,
            3,
        )

    cv2.putText(
        frame512,
        packet,
        (10, 325),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        color,
        2,
    )

    if observation.near_y_norm is None:
        near_text = "near_y=-- confirm=0/%d" % COURT_EDGE_CONFIRM_FRAMES
    else:
        near_text = "near_y=%.2f confirm=%d/%d" % (
            observation.near_y_norm,
            observation.near_confirm_count,
            COURT_EDGE_CONFIRM_FRAMES,
        )

    cv2.putText(
        frame512,
        near_text,
        (10, 350),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        1,
    )
    return frame512

def main():
    ser = None
    cap = None
    video_writer = None
    csv_file = None
    rpm_csv_file = None
    court_csv_file = None

    try:
        ser = open_uart()

        # 啟動時先確保馬達停止
        send_command(ser, "BALL_OFF")
        send_command(ser, "COURT,SAFE,NONE,UNKNOWN")

        print("Loading TensorRT engine...")
        engine = load_engine(ENGINE_PATH)
        context = engine.create_execution_context()
        inputs, outputs, bindings, stream = allocate_buffers(engine)

        print("Engine:", ENGINE_PATH)
        print("Input:", INPUT_W, "x", INPUT_H)
        print("Focal length px:", FOCAL_LENGTH_PX)
        print("Ball diameter cm:", BALL_DIAMETER_CM)

        court_runtime = CourtLineRuntime(
            COURT_YELLOW_MODEL_PATH,
            edge_y=COURT_EDGE_Y,
            edge_confirm_frames=COURT_EDGE_CONFIRM_FRAMES,
        )
        if court_runtime.color_model_loaded:
            print("Yellow-only model:", COURT_YELLOW_MODEL_PATH)
        else:
            print("WARNING: yellow model unavailable")

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

        # Court runtime state.  Do not call observe() on skipped frames: only
        # real camera observations are allowed to count toward EDGE.
        court_result = None
        court_last_process_time = 0.0
        court_last_uart_time = 0.0
        court_last_packet = None
        last_ball_infer_time = 0.0

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

        rpm_csv_file = open(RPM_CSV_PATH, "w", newline="")
        rpm_writer = csv.writer(rpm_csv_file)
        rpm_writer.writerow([
            "jetson_time_s",
            "stm32_ms",
            "target_left_rpm",
            "actual_left_rpm",
            "target_right_rpm",
            "actual_right_rpm",
        ])

        court_csv_file = open(COURT_CSV_PATH, "w", newline="")
        court_writer = csv.writer(court_csv_file)
        court_writer.writerow([
            "frame",
            "time_s",
            "packet",
            "state",
            "line_side",
            "color",
            "near_y_norm",
            "confirm_count",
            "position_state",
            "color_status",
            "raw_label",
            "raw_confidence",
            "process_ms",
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

            drain_rpm_telemetry(ser, rpm_writer, total_t0)

            # ---------------------------------------------------------
            # Four-color yarn boundary detection on original 640x480 frame.
            # Its result shares the UART with BALL messages.
            # ---------------------------------------------------------
            if (t_now - court_last_process_time) >= COURT_PERIOD_SEC:
                court_t0 = time.time()
                court_result = court_runtime.observe(raw_frame)
                court_ms = (time.time() - court_t0) * 1000.0
                court_last_process_time = t_now
                court_command = court_result.packet()

                # EDGE is re-sent on every fresh court observation.  SAFE is
                # sent on packet changes and as a heartbeat, so STM32 always
                # has current camera state without blocking the vision loop.
                if (
                    court_command != court_last_packet
                    or court_result.observation.state == "EDGE"
                    or (t_now - court_last_uart_time) >= COURT_UART_HEARTBEAT_SEC
                ):
                    send_command(ser, court_command)
                    court_last_packet = court_command
                    court_last_uart_time = t_now

                court_writer.writerow([
                    frame_count,
                    "%.3f" % (t_now - total_t0),
                    court_command,
                    court_result.observation.state,
                    court_result.observation.line_side,
                    court_result.observation.color,
                    "" if court_result.observation.near_y_norm is None
                    else "%.5f" % court_result.observation.near_y_norm,
                    court_result.observation.near_confirm_count,
                    court_result.position_state,
                    court_result.color_status,
                    court_result.raw_label,
                    "%.4f" % court_result.raw_confidence,
                    "%.1f" % court_ms,
                ])

                print(
                    court_command,
                    "| near_y",
                    "--" if court_result.observation.near_y_norm is None
                    else "%.2f" % court_result.observation.near_y_norm,
                    "| confirm %d/%d" % (
                        court_result.observation.near_confirm_count,
                        COURT_EDGE_CONFIRM_FRAMES,
                    ),
                    "| %.1f ms" % court_ms,
                )

            # Boundary safety gets every camera frame.  The slower ball model
            # runs at a fixed rate, so it cannot starve COURT updates.
            frame512 = cv2.resize(frame, (INPUT_W, INPUT_H))
            if (t_now - last_ball_infer_time) >= BALL_INFER_PERIOD_SEC:
                last_ball_infer_time = t_now
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

            frame512 = draw_court_line_status(frame512, court_result)

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

        if rpm_csv_file is not None:
            rpm_csv_file.close()

        if court_csv_file is not None:
            court_csv_file.close()

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
            print("RPM CSV saved:", RPM_CSV_PATH)
            print("Court CSV saved:", COURT_CSV_PATH)
            print("Video saved:", SAVE_VIDEO)

        if ser is not None and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
