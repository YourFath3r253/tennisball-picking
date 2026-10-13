import cv2
import numpy as np

VIDEO = "court_20260916_201113.avi"

cap = cv2.VideoCapture(VIDEO)

if not cap.isOpened():
    print("ERROR: Cannot open video")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]

    # 1. 灰階
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2. 稍微模糊，減少地板紋路造成的雜訊
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # 3. 找邊緣
    edges = cv2.Canny(blur, 60, 150)

    # 4. Hough Transform 找線段
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=50,
        minLineLength=100,
        maxLineGap=30
    )

    detected_y = None

    if lines is not None:
        candidates = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            dx = x2 - x1
            dy = y2 - y1

            # 避免除以 0
            if abs(dx) < 1:
                continue

            slope = abs(dy / dx)
            length = np.sqrt(dx * dx + dy * dy)

            # 第一版：只找「夠長、接近水平」的線
            if slope < 0.25 and length > 100:
                y_avg = int((y1 + y2) / 2)
                candidates.append(y_avg)

                cv2.line(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    3
                )

        if candidates:
            # 最靠近畫面底部的候選線
            detected_y = max(candidates)

    # -------------------------
    # Boundary 判斷
    # -------------------------

    # 先暫定畫面高度 70% 以下為危險區
    danger_y = int(h * 0.70)

    cv2.line(
        frame,
        (0, danger_y),
        (w, danger_y),
        (0, 0, 255),
        2
    )

    if detected_y is not None:
        cv2.putText(
            frame,
            "LINE y=%d" % detected_y,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        if detected_y > danger_y:
            cv2.putText(
                frame,
                "BOUNDARY!",
                (20, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 0, 255),
                3
            )

    cv2.imshow("Court Boundary Test", frame)
    cv2.imshow("Edges", edges)

    key = cv2.waitKey(30) & 0xFF

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
