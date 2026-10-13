import cv2
import time

CAMERA_INDEX = 0
WIDTH = 640
HEIGHT = 480
FPS = 30

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

if not cap.isOpened():
    print("ERROR: Cannot open camera")
    exit()

filename = time.strftime("court_%Y%m%d_%H%M%S.avi")

fourcc = cv2.VideoWriter_fourcc(*"MJPG")
out = cv2.VideoWriter(filename, fourcc, FPS, (WIDTH, HEIGHT))

print("Recording:", filename)
print("Press Ctrl+C to stop.")

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to read frame")
            break

        out.write(frame)

except KeyboardInterrupt:
    print("\nRecording stopped.")

finally:
    cap.release()
    out.release()

print("Saved:", filename)
