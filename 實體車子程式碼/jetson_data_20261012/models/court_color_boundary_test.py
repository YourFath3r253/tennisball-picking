#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import argparse
import os


# 每個 patch 使用：
# normalized B/G/R + Saturation + LAB a/b
FEATURE_NUM = 6


def patch_feature(patch):
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB).astype(np.float32)
    bgr = patch.astype(np.float32)

    b = bgr[:, :, 0]
    g = bgr[:, :, 1]
    r = bgr[:, :, 2]

    s = hsv[:, :, 1]
    v = hsv[:, :, 2]

    # 不拿白線、強反光、極暗區域來學顏色
    white = (s < 35) & (v > 180)
    reflection = v > 245
    dark = v < 25

    valid = ~(white | reflection | dark)

    if np.count_nonzero(valid) < valid.size * 0.35:
        return None

    denom = r + g + b + 1.0

    feat = np.array([
        np.median(b[valid] / denom[valid]),
        np.median(g[valid] / denom[valid]),
        np.median(r[valid] / denom[valid]),
        np.median(s[valid]) / 255.0,
        np.median((lab[:, :, 1][valid] - 128.0) / 127.0),
        np.median((lab[:, :, 2][valid] - 128.0) / 127.0),
    ], dtype=np.float32)

    return feat


def get_geometry(path, patch, roi_y0, roi_y1):
    cap = cv2.VideoCapture(path)
    ok, frame = cap.read()
    cap.release()

    if not ok:
        raise RuntimeError("Cannot read: " + path)

    h, w = frame.shape[:2]

    y0 = int(h * roi_y0)
    y1 = int(h * roi_y1)

    xs = list(range(0, w - patch + 1, patch))
    ys = list(range(y0, y1 - patch + 1, patch))

    return h, w, y0, y1, xs, ys


def collect_samples(path, patch, roi_y0, roi_y1, frame_step):
    h, w, y0, y1, xs, ys = get_geometry(
        path, patch, roi_y0, roi_y1
    )

    samples = [
        [[] for _ in xs]
        for _ in ys
    ]

    cap = cv2.VideoCapture(path)
    frame_id = 0

    while True:
        ok, frame = cap.read()

        if not ok:
            break

        if frame_id % frame_step == 0:

            for ry, y in enumerate(ys):

                for cx, x in enumerate(xs):

                    p = frame[
                        y:y + patch,
                        x:x + patch
                    ]

                    f = patch_feature(p)

                    if f is not None:
                        samples[ry][cx].append(f)

        frame_id += 1

    cap.release()

    return samples, (h, w, y0, y1, xs, ys)


def train_model(
    inside_path,
    outside_path,
    model_path,
    patch,
    roi_y0,
    roi_y1,
    frame_step
):

    print("Collecting INSIDE samples...")

    inside, geo_in = collect_samples(
        inside_path,
        patch,
        roi_y0,
        roi_y1,
        frame_step
    )

    print("Collecting OUTSIDE samples...")

    outside, geo_out = collect_samples(
        outside_path,
        patch,
        roi_y0,
        roi_y1,
        frame_step
    )

    if geo_in[:4] != geo_out[:4]:
        raise RuntimeError(
            "INSIDE / OUTSIDE video geometry differs"
        )

    h, w, y0, y1, xs, ys = geo_in

    rows = len(ys)
    cols = len(xs)

    # 每個格子各自有自己的分類方向與 threshold
    weights = np.zeros(
        (rows, cols, FEATURE_NUM),
        dtype=np.float32
    )

    th_inside = np.zeros(
        (rows, cols),
        dtype=np.float32
    )

    th_outside = np.zeros(
        (rows, cols),
        dtype=np.float32
    )

    valid_cell = np.zeros(
        (rows, cols),
        dtype=np.uint8
    )

    separation = np.zeros(
        (rows, cols),
        dtype=np.float32
    )

    learned = 0

    for ry in range(rows):

        for cx in range(cols):

            if (
                len(inside[ry][cx]) < 10
                or
                len(outside[ry][cx]) < 10
            ):
                continue

            A = np.asarray(
                inside[ry][cx],
                dtype=np.float32
            )

            B = np.asarray(
                outside[ry][cx],
                dtype=np.float32
            )

            med_in = np.median(
                A,
                axis=0
            )

            med_out = np.median(
                B,
                axis=0
            )

            mad_in = np.median(
                np.abs(A - med_in),
                axis=0
            )

            mad_out = np.median(
                np.abs(B - med_out),
                axis=0
            )

            scale_in = (
                1.4826 * mad_in
                + 1e-4
            )

            scale_out = (
                1.4826 * mad_out
                + 1e-4
            )

            # 找這個位置最能區分 IN / OUT 的顏色方向
            w_vec = (
                med_out - med_in
            ) / (
                scale_in * scale_in
                +
                scale_out * scale_out
                +
                1e-6
            )

            norm = float(
                np.linalg.norm(w_vec)
            )

            if norm < 1e-6:
                continue

            w_vec = (
                w_vec / norm
            )

            score_in = A.dot(
                w_vec
            )

            score_out = B.dot(
                w_vec
            )

            # 統一：
            # score 越大 = 越像 OUTSIDE
            if (
                np.median(score_out)
                <
                np.median(score_in)
            ):

                w_vec = -w_vec
                score_in = -score_in
                score_out = -score_out

            med_score_in = float(
                np.median(score_in)
            )

            med_score_out = float(
                np.median(score_out)
            )

            # OUTSIDE 判斷採保守策略：
            # 要高過 98% 的場內樣本
            # 避免像之前右側場內亂變紅
            out_threshold = max(
                float(
                    np.percentile(score_in, 98.0)
                ),
                (
                    med_score_in
                    +
                    med_score_out
                ) / 2.0
            )

            # 明顯屬於正常場內
            in_threshold = float(
                np.percentile(score_in, 80.0)
            )

            if (
                out_threshold
                <=
                in_threshold
            ):
                out_threshold = (
                    in_threshold
                    + 1e-4
                )

            spread = (
                np.median(
                    np.abs(
                        score_in
                        -
                        med_score_in
                    )
                )
                * 1.4826
                +
                np.median(
                    np.abs(
                        score_out
                        -
                        med_score_out
                    )
                )
                * 1.4826
                +
                1e-4
            )

            sep = (
                med_score_out
                -
                med_score_in
            ) / spread

            weights[
                ry,
                cx
            ] = w_vec

            th_inside[
                ry,
                cx
            ] = in_threshold

            th_outside[
                ry,
                cx
            ] = out_threshold

            separation[
                ry,
                cx
            ] = sep

            valid_cell[
                ry,
                cx
            ] = 1

            learned += 1

    np.savez(
        model_path,
        weights=weights,
        th_inside=th_inside,
        th_outside=th_outside,
        valid_cell=valid_cell,
        separation=separation,
        h=np.array([h]),
        w=np.array([w]),
        y0=np.array([y0]),
        y1=np.array([y1]),
        patch=np.array([patch])
    )

    print()
    print(
        "Saved model:",
        model_path
    )

    print(
        "Learned cells:",
        learned,
        "/",
        rows * cols
    )

    if learned > 0:

        good_sep = separation[
            valid_cell == 1
        ]

        print(
            "Median separation:",
            "%.2f"
            %
            float(
                np.median(good_sep)
            )
        )


def load_model(path):

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


def remove_isolated_outside(
    class_grid,
    min_size=3
):

    binary = (
        class_grid == 1
    ).astype(np.uint8)

    num_labels, labels, stats, _ = \
        cv2.connectedComponentsWithStats(
            binary,
            connectivity=8
        )

    clean = np.zeros_like(
        binary
    )

    for i in range(
        1,
        num_labels
    ):

        area = stats[
            i,
            cv2.CC_STAT_AREA
        ]

        if area >= min_size:

            clean[
                labels == i
            ] = 1

    return clean


def classify_frame(
    frame,
    model
):

    h, w = frame.shape[:2]

    if (
        h != model["h"]
        or
        w != model["w"]
    ):
        raise RuntimeError(
            "Test video resolution differs from training videos"
        )

    patch = model["patch"]

    y0 = model["y0"]
    y1 = model["y1"]

    xs = list(
        range(
            0,
            w - patch + 1,
            patch
        )
    )

    ys = list(
        range(
            y0,
            y1 - patch + 1,
            patch
        )
    )

    rows = len(ys)
    cols = len(xs)

    # -1 = UNKNOWN
    #  0 = INSIDE
    #  1 = OUTSIDE
    grid = np.full(
        (rows, cols),
        -1,
        dtype=np.int8
    )

    for ry, y in enumerate(ys):

        for cx, x in enumerate(xs):

            if (
                model["valid_cell"][
                    ry,
                    cx
                ]
                == 0
            ):
                continue

            p = frame[
                y:y + patch,
                x:x + patch
            ]

            f = patch_feature(p)

            if f is None:
                continue

            score = float(
                np.dot(
                    f,
                    model["weights"][
                        ry,
                        cx
                    ]
                )
            )

            if (
                score
                >=
                model["th_outside"][
                    ry,
                    cx
                ]
            ):

                grid[
                    ry,
                    cx
                ] = 1

            elif (
                score
                <=
                model["th_inside"][
                    ry,
                    cx
                ]
            ):

                grid[
                    ry,
                    cx
                ] = 0

            else:

                grid[
                    ry,
                    cx
                ] = -1

    # 零星紅格先去掉
    clean_out = remove_isolated_outside(
        grid,
        min_size=3
    )

    inside = (
        grid == 0
    )

    # 被去除的小紅格不改成綠色
    # 而是視為 UNKNOWN
    known = (
        inside
        |
        (clean_out == 1)
    )

    known_count = int(
        np.count_nonzero(
            known
        )
    )

    out_count = int(
        np.count_nonzero(
            clean_out
        )
    )

    if known_count > 0:

        outside_ratio = (
            out_count
            /
            float(known_count)
        )

    else:

        outside_ratio = 0.0

    known_ratio = (
        known_count
        /
        float(
            rows * cols
        )
    )

    direction = "NONE"

    pos = np.argwhere(
        clean_out == 1
    )

    if len(pos) > 0:

        mean_col = float(
            np.mean(
                pos[:, 1]
            )
        )

        if mean_col < cols / 3.0:

            direction = "LEFT"

        elif mean_col > (
            2.0 * cols / 3.0
        ):

            direction = "RIGHT"

        else:

            direction = "CENTER"

    return (
        grid,
        clean_out,
        outside_ratio,
        known_ratio,
        direction
    )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--inside",
        default="/home/hp/models/inside_only.avi"
    )

    parser.add_argument(
        "--outside",
        default="/home/hp/models/outside_only.avi"
    )

    parser.add_argument(
        "--input",
        default="/home/hp/models/court_20260919_160502.avi"
    )

    parser.add_argument(
        "--output",
        default="/home/hp/models/court_boundary_result_v6.mp4"
    )

    parser.add_argument(
        "--model",
        default="/home/hp/models/court_color_model_v6.npz"
    )

    parser.add_argument(
        "--patch",
        type=int,
        default=24
    )

    parser.add_argument(
        "--roi-y0",
        type=float,
        default=0.32
    )

    parser.add_argument(
        "--roi-y1",
        type=float,
        default=0.96
    )

    parser.add_argument(
        "--frame-step",
        type=int,
        default=4
    )

    parser.add_argument(
        "--retrain",
        action="store_true"
    )

    # hysteresis
    parser.add_argument(
        "--edge-enter",
        type=float,
        default=0.20
    )

    parser.add_argument(
        "--edge-exit",
        type=float,
        default=0.10
    )

    parser.add_argument(
        "--out-enter",
        type=float,
        default=0.55
    )

    parser.add_argument(
        "--out-exit",
        type=float,
        default=0.40
    )

    parser.add_argument(
        "--confirm",
        type=int,
        default=5
    )

    # 相機剛開始幾秒容易自動曝光/白平衡變化
    parser.add_argument(
        "--warmup",
        type=float,
        default=2.0
    )

    args = parser.parse_args()

    # 第一次要 train
    if (
        args.retrain
        or
        not os.path.exists(
            args.model
        )
    ):

        train_model(
            args.inside,
            args.outside,
            args.model,
            args.patch,
            args.roi_y0,
            args.roi_y1,
            args.frame_step
        )

    model = load_model(
        args.model
    )

    cap = cv2.VideoCapture(
        args.input
    )

    if not cap.isOpened():

        raise SystemExit(
            "Cannot open: "
            + args.input
        )

    fps = cap.get(
        cv2.CAP_PROP_FPS
    )

    ok, first = cap.read()

    if not ok:
        raise SystemExit(
            "Cannot read video"
        )

    h, w = first.shape[:2]

    writer = cv2.VideoWriter(
        args.output,
        cv2.VideoWriter_fourcc(
            *"mp4v"
        ),
        fps,
        (w, h)
    )

    cap.set(
        cv2.CAP_PROP_POS_FRAMES,
        0
    )

    patch = model["patch"]

    y0 = model["y0"]
    y1 = model["y1"]

    xs = list(
        range(
            0,
            w - patch + 1,
            patch
        )
    )

    ys = list(
        range(
            y0,
            y1 - patch + 1,
            patch
        )
    )

    smooth = 0.0
    smooth_started = False

    alpha = 0.20

    state = "SAFE"

    pending_target = None
    pending_count = 0

    frame_id = 0

    while True:

        ok, frame = cap.read()

        if not ok:
            break

        (
            grid,
            clean_out,
            raw_ratio,
            known_ratio,
            direction
        ) = classify_frame(
            frame,
            model
        )

        elapsed = (
            frame_id / fps
            if fps > 0
            else 0
        )

        # 前兩秒先讓曝光/白平衡穩定
        if elapsed >= args.warmup:

            if not smooth_started:

                smooth = raw_ratio
                smooth_started = True

            else:

                smooth = (
                    alpha * raw_ratio
                    +
                    (1 - alpha)
                    * smooth
                )

            if known_ratio >= 0.25:

                # ============
                # HYSTERESIS
                # ============

                if state == "SAFE":

                    if (
                        smooth
                        >=
                        args.edge_enter
                    ):
                        target = "EDGE"
                    else:
                        target = "SAFE"

                elif state == "EDGE":

                    if (
                        smooth
                        >=
                        args.out_enter
                    ):
                        target = "OUT"

                    elif (
                        smooth
                        <=
                        args.edge_exit
                    ):
                        target = "SAFE"

                    else:
                        target = "EDGE"

                else:
                    # OUT 狀態
                    # 必須真的下降到 40% 以下才退回 EDGE

                    if (
                        smooth
                        <=
                        args.out_exit
                    ):
                        target = "EDGE"
                    else:
                        target = "OUT"

                if target != state:

                    if (
                        pending_target
                        ==
                        target
                    ):

                        pending_count += 1

                    else:

                        pending_target = target
                        pending_count = 1

                    if (
                        pending_count
                        >=
                        args.confirm
                    ):

                        state = target

                        pending_target = None
                        pending_count = 0

                else:

                    pending_target = None
                    pending_count = 0

        # ============
        # 顯示格子
        # 綠 = IN
        # 紅 = OUT
        # 黃 = UNKNOWN
        # ============

        for ry, y in enumerate(ys):

            for cx, x in enumerate(xs):

                if (
                    clean_out[
                        ry,
                        cx
                    ]
                    == 1
                ):

                    color = (
                        0,
                        0,
                        255
                    )

                elif (
                    grid[
                        ry,
                        cx
                    ]
                    == 0
                ):

                    color = (
                        0,
                        255,
                        0
                    )

                else:

                    color = (
                        0,
                        255,
                        255
                    )

                cv2.rectangle(
                    frame,
                    (x, y),
                    (
                        x + patch - 1,
                        y + patch - 1
                    ),
                    color,
                    1
                )

        cv2.rectangle(
            frame,
            (0, y0),
            (w - 1, y1),
            (255, 255, 255),
            2
        )

        if elapsed < args.warmup:

            show_state = "WARMUP"

            state_color = (
                255,
                255,
                255
            )

        else:

            show_state = state

            if state == "SAFE":

                state_color = (
                    0,
                    255,
                    0
                )

            elif state == "EDGE":

                state_color = (
                    0,
                    255,
                    255
                )

            else:

                state_color = (
                    0,
                    0,
                    255
                )

        cv2.putText(
            frame,
            show_state,
            (20, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            state_color,
            3
        )

        cv2.putText(
            frame,
            "OUTSIDE: %.1f%%  raw: %.1f%%"
            %
            (
                smooth * 100.0,
                raw_ratio * 100.0
            ),
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            "KNOWN: %.1f%%  DIR: %s"
            %
            (
                known_ratio * 100.0,
                direction
            ),
            (20, 104),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (255, 255, 255),
            2
        )

        writer.write(
            frame
        )

        frame_id += 1

    cap.release()
    writer.release()

    print()
    print(
        "Saved:",
        args.output
    )


if __name__ == "__main__":
    main()
