#!/usr/bin/env python3
import cv2
import argparse
import time
import os


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--name",
        required=True,
        help="Output name, e.g. inside_only or outside_only"
    )

    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera index, default 0"
    )

    parser.add_argument(
        "--seconds",
        type=int,
        default=20,
        help="Recording duration"
    )

    parser.add_argument(
        "--width",
        type=int,
        default=640
    )

    parser.add_argument(
        "--height",
        type=int,
        default=480
    )

    parser.add_argument(
        "--fps",
        type=int,
        default=30
    )

    args = parser.parse_args()

    output_path = f"/home/hp/models/{args.name}.avi"

    cap = cv2.VideoCapture(args.camera)

    # Astra Pro 之前用 MJPG 比較穩
    cap.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG")
    )

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        args.width
    )

    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        args.height
    )

    cap.set(
        cv2.CAP_PROP_FPS,
        args.fps
    )

    if not cap.isOpened():
        print("ERROR: Cannot open camera")
        return

    real_width = int(
        cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    )

    real_height = int(
        cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    )

    real_fps = cap.get(
        cv2.CAP_PROP_FPS
    )

    print(
        f"Camera opened: "
        f"{real_width}x{real_height} "
        f"{real_fps:.1f} FPS"
    )

    fourcc = cv2.VideoWriter_fourcc(
        *"MJPG"
    )

    writer = cv2.VideoWriter(
        output_path,
        fourcc,
        args.fps,
        (
            real_width,
            real_height
        )
    )

    if not writer.isOpened():
        print("ERROR: Cannot create output video")
        cap.release()
        return

    print()
    print("================================")
    print("Recording:", args.name)
    print("Duration :", args.seconds, "seconds")
    print("Output   :", output_path)
    print("================================")
    print()

    start = time.time()
    frame_count = 0
    last_print = -1

    while True:

        elapsed = time.time() - start

        if elapsed >= args.seconds:
            break

        ok, frame = cap.read()

        if not ok:
            print("WARNING: Camera frame failed")
            continue

        writer.write(frame)

        frame_count += 1

        sec = int(elapsed)

        if sec != last_print:
            print(
                f"Recording... "
                f"{sec}/{args.seconds} sec"
            )
            last_print = sec

    cap.release()
    writer.release()

    size_mb = (
        os.path.getsize(output_path)
        / 1024
        / 1024
    )

    print()
    print("Done!")
    print("Saved:", output_path)
    print(
        f"Frames: {frame_count}, "
        f"Size: {size_mb:.1f} MB"
    )


if __name__ == "__main__":
    main()
