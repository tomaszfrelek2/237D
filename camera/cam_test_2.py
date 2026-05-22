import cv2
import threading
import requests
import time

def keep_alive():
    while True:
        try:
            requests.get("http://172.28.114.51:8080/gopro/camera/keep_alive", timeout=1)
        except:
            pass
        time.sleep(2.5)

threading.Thread(target=keep_alive, daemon=True).start()

STREAM_URLS = [
    "udp://172.28.114.51:8554",
    "rtsp://172.28.114.51:8554/live",
    "udp://@:8554",
]

def open_stream(urls):
    for url in urls:
        print(f"Trying: {url}")
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            print(f"Connected: {url}")
            return cap
    return None

cap = open_stream(STREAM_URLS)
if not cap:
    raise RuntimeError("Could not connect to GoPro stream")

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (1408, 704))

frame_count = 0
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Could not read frame")
            continue

        out.write(frame)

        frame_count += 1
        if frame_count % 30 == 0:
            print(f"Captured {frame_count} frames")

        if frame_count >= 300:
            break
finally:
    cap.release()
    out.release()
    requests.get("http://172.28.114.51:8080/gopro/camera/stream/stop")
    print(f"Done. Stream stopped. Saved {frame_count} frames to output.mp4")