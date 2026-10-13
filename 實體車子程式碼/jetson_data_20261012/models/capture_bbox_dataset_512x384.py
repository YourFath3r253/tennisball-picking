import os
import cv2
import time
import math
import csv
import argparse
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit


ENGINE_PATH = "best_fold1_opset9_sim_512x384.engine"

INPUT_W = 512
INPUT_H = 384

CAM_W = 640
CAM_H = 480

CONF_THRES = 0.35
NMS_THRES = 0.45

BALL_DIAMETER_CM = 6.7


def preprocess(frame):
    """
    640x480 -> 512x384
    因為兩者都是 4:3，所以這裡只 resize，沒有 padding。
    """
    resized = cv2.resize(frame, (INPUT_W, INPUT_H))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    img = rgb.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    img = np.ascontiguousarray(img)
    return img, resized


def load_engine(engine_path):
    logger = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f:
        runtime = trt.Runtime(logger)
        engine = runtime.deserialize_cuda_engine(f.read())
    return engine


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
            "dtype": dtype
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
    x1 = cx - w / 2.0
    y1 = cy - h / 2.0
    x2 = cx + w / 2.0
    y2 = cy + h / 2.0
    return x1, y1, x2, y2


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
    if union <= 0:
        return 0.0
    return inter / union


def nms(dets, nms_thres):
    if len(dets) == 0:
        return []

    dets = sorted(dets, key=lambda x: x["conf"], reverse=True)
    keep = []

    while dets:
        best = dets.pop(0)
        keep.append(best)
        dets = [d for d in dets if iou(best["box"], d["box"]) < nms_thres]

    return keep


def parse_yolo_output(outputs_data):
    """
    支援常見 YOLOv8 TensorRT output：
    可能是 (1, 5, N) 或 (1, N, 5)
    單類別格式假設為 cx, cy, w, h, conf。
    座標單位是 512x384 模型輸入座標。
    """
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

        cx, cy, w, h, conf = (
            float(row[0]),
            float(row[1]),
            float(row[2]),
            float(row[3]),
            float(row[4])
        )

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

        dets.append({
            "box": [x1, y1, x2, y2],
            "conf": conf
        })

    return nms(dets, NMS_THRES)


def extract_bbox_features(det):
    x1, y1, x2, y2 = det["box"]

    box_w = x2 - x1
    box_h = y2 - y1
    box_area = box_w * box_h
    sqrt_wh = math.sqrt(max(1.0, box_area))
    wh_ratio = box_w / box_h if box_h > 0 else 0.0

    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0

    return {
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "cx": cx,
        "cy": cy,
        "box_w": box_w,
        "box_h": box_h,
        "box_area": box_area,
        "sqrt_wh": sqrt_wh,
        "wh_ratio": wh_ratio,
        "conf": det["conf"]
    }


def draw_annotated(frame512, features, true_z, angle_deg, sample_idx):
    out = frame512.copy()

    if features is not None:
        x1 = int(features["x1"])
        y1 = int(features["y1"])
        x2 = int(features["x2"])
        y2 = int(features["y2"])
        cx = int(features["cx"])
        cy = int(features["cy"])

        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(out, (cx, cy), 4, (0, 0, 255), -1)

        text1 = "conf %.2f" % features["conf"]
        text2 = "w %.1f h %.1f sqrt %.1f" % (
            features["box_w"],
            features["box_h"],
            features["sqrt_wh"]
        )
        text3 = "w/h %.3f" % features["wh_ratio"]

        cv2.putText(out, text1, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(out, text2, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(out, text3, (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    else:
        cv2.putText(out, "No ball detected", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    text4 = "Z %.1f cm  angle %.1f deg  sample %d" % (
        true_z,
        angle_deg,
        sample_idx
    )
    cv2.putText(out, text4, (10, INPUT_H - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    return out


def angle_to_tag(angle_deg):
    if angle_deg > 0:
        return "ap%02d" % abs(int(round(angle_deg)))
    elif angle_deg < 0:
        return "am%02d" % abs(int(round(angle_deg)))
    else:
        return "a000"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--z", type=float, required=True,
                        help="true distance in cm, example: 50")
    parser.add_argument("--angle", type=float, required=True,
                        help="true horizontal angle in degree, example: -15, 0, 15")
    parser.add_argument("--count", type=int, default=3,
                        help="number of images to save")
    parser.add_argument("--out", type=str, default="bbox_dataset_512x384",
                        help="output folder")
    parser.add_argument("--warmup", type=int, default=20,
                        help="camera warmup frames")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="seconds between captures")
    args = parser.parse_args()

    true_z = args.z
    angle_deg = args.angle
    true_x = true_z * math.tan(math.radians(angle_deg))

    os.makedirs(args.out, exist_ok=True)
    raw_dir = os.path.join(args.out, "raw")
    ann_dir = os.path.join(args.out, "annotated")
    os.makedirs(raw_dir, exist_ok=True)
    os.makedirs(ann_dir, exist_ok=True)

    csv_path = os.path.join(args.out, "bbox_dataset.csv")
    csv_exists = os.path.exists(csv_path)

    print("Loading TensorRT engine...")
    engine = load_engine(ENGINE_PATH)
    context = engine.create_execution_context()
    inputs, outputs, bindings, stream = allocate_buffers(engine)

    print("Engine:", ENGINE_PATH)
    print("Input:", INPUT_W, "x", INPUT_H)
    print("True Z cm:", true_z)
    print("Angle deg:", angle_deg)
    print("True X cm:", true_x)
    print("Output folder:", args.out)
    print("CSV:", csv_path)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("ERROR: Cannot open camera")
        return

    print("Camera width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print("Camera height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("Camera FPS setting:", cap.get(cv2.CAP_PROP_FPS))

    print("Warming up camera...")
    for _ in range(args.warmup):
        cap.read()
        time.sleep(0.02)

    z_tag = "z%03d" % int(round(true_z))
    a_tag = angle_to_tag(angle_deg)

    with open(csv_path, "a", newline="") as csv_file:
        writer = csv.writer(csv_file)

        if not csv_exists:
            writer.writerow([
                "image_name",
                "raw_image_path",
                "annotated_image_path",
                "true_z_cm",
                "angle_deg",
                "true_x_cm",
                "detected",
                "conf",
                "cx",
                "cy",
                "x1",
                "y1",
                "x2",
                "y2",
                "box_w",
                "box_h",
                "box_area",
                "sqrt_wh",
                "wh_ratio",
                "input_w",
                "input_h",
                "camera_w",
                "camera_h",
                "timestamp"
            ])

        saved = 0

        for i in range(1, args.count + 1):
            ret, frame = cap.read()
            if not ret:
                print("Cannot read frame at sample", i)
                continue

            input_image, frame512 = preprocess(frame)
            outputs_data = infer(context, bindings, inputs, outputs, stream, input_image)
            dets = parse_yolo_output(outputs_data)

            features = None
            if len(dets) > 0:
                det = dets[0]
                features = extract_bbox_features(det)

            file_base = "%s_%s_%02d" % (z_tag, a_tag, i)
            raw_name = file_base + "_raw.jpg"
            ann_name = file_base + "_ann.jpg"

            raw_path = os.path.join(raw_dir, raw_name)
            ann_path = os.path.join(ann_dir, ann_name)

            annotated = draw_annotated(frame512, features, true_z, angle_deg, i)

            cv2.imwrite(raw_path, frame512)
            cv2.imwrite(ann_path, annotated)

            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

            if features is not None:
                writer.writerow([
                    file_base,
                    raw_path,
                    ann_path,
                    "%.2f" % true_z,
                    "%.2f" % angle_deg,
                    "%.2f" % true_x,
                    1,
                    "%.4f" % features["conf"],
                    "%.2f" % features["cx"],
                    "%.2f" % features["cy"],
                    "%.2f" % features["x1"],
                    "%.2f" % features["y1"],
                    "%.2f" % features["x2"],
                    "%.2f" % features["y2"],
                    "%.2f" % features["box_w"],
                    "%.2f" % features["box_h"],
                    "%.2f" % features["box_area"],
                    "%.2f" % features["sqrt_wh"],
                    "%.4f" % features["wh_ratio"],
                    INPUT_W,
                    INPUT_H,
                    CAM_W,
                    CAM_H,
                    timestamp
                ])

                print(
                    "[%d/%d] saved %s | conf %.2f | w %.1f h %.1f sqrt %.1f w/h %.3f"
                    % (
                        i,
                        args.count,
                        file_base,
                        features["conf"],
                        features["box_w"],
                        features["box_h"],
                        features["sqrt_wh"],
                        features["wh_ratio"]
                    )
                )
            else:
                writer.writerow([
                    file_base,
                    raw_path,
                    ann_path,
                    "%.2f" % true_z,
                    "%.2f" % angle_deg,
                    "%.2f" % true_x,
                    0,
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    INPUT_W,
                    INPUT_H,
                    CAM_W,
                    CAM_H,
                    timestamp
                ])

                print("[%d/%d] saved %s | No ball detected" % (i, args.count, file_base))

            csv_file.flush()
            saved += 1

            if i < args.count:
                time.sleep(args.interval)

    cap.release()

    print("")
    print("Done.")
    print("Saved images:", saved)
    print("CSV saved:", csv_path)
    print("Raw images folder:", raw_dir)
    print("Annotated images folder:", ann_dir)


if __name__ == "__main__":
    main()
