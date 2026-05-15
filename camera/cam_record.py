import cv2
import threading
import requests
import time
import os

os.makedirs("recordings", exist_ok=True)

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

def new_writer(chunk_num):
    filename = f"recordings/output_{chunk_num:04d}.mp4"
    print(f"Starting new file: {filename}")
    return cv2.VideoWriter(filename, fourcc, 30.0, (1408, 704)), filename

# out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (1408, 704))

chunk = 0
out, current_file = new_writer(chunk)

frame_count = 0
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Could not read frame")
            continue

        out.write(frame)

        frame_count += 1
        # if frame_count % 30 == 0:
        #     print(f"Captured {frame_count} frames")

        if frame_count >= 300:
            frame_count = 0
            out.release()
            chunk += 1
            out, current_file = new_writer(chunk)
finally:
    cap.release()
    requests.get("http://172.28.114.51:8080/gopro/camera/stream/stop")
    print(f"Done. Stream stopped. Saved {frame_count} frames to {current_file}")