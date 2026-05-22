import cv2

cap = cv2.VideoCapture("udp://172.28.114.51:8554", cv2.CAP_FFMPEG)
ret, frame = cap.read()
if ret:
    print(f"Frame shape: {frame.shape}")  # (height, width, channels)
    print(f"Width: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
    print(f"Height: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
cap.release()