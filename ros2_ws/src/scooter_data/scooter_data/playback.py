# Copyright 2024
# Licensed under the Apache License, Version 2.0 (the "License");
import sys
import os
import argparse
import numpy as np
import cv2

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from rclpy.serialization import deserialize_message
import rosbag2_py

from sensor_msgs.msg import CompressedImage, PointCloud2, JointState
from radar_msgs.msg import RadarScan
from sensor_msgs_py import point_cloud2 as pc2
from vision_msgs.msg import Detection2DArray

TOPIC_TYPE_MAP = {
    '/video/compressed': CompressedImage,
    '/radar/points': PointCloud2,
    '/servo/joint_states': JointState,
    '/video/detections': Detection2DArray,
    '/video/yolov26_image/compressed': CompressedImage
}

# --- Plot Settings ---
SIDE_LIMIT_M = 10.0      
FORWARD_MIN_M = 0.0
FORWARD_MAX_M = 25.0     
HEIGHT_MIN_M = -3.0
HEIGHT_MAX_M = 5.0
FIXED_ELEVATION = 20
FIXED_AZIMUTH = 0
TARGET_FPS = 10.0

#Detection Overlay Settings
BOX_COLOR = (0, 255, 0)         # Green bounding boxes
BOX_THICKNESS = 2
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.55
FONT_THICKNESS = 1
LABEL_BG_COLOR = (0, 255, 0)
LABEL_TEXT_COLOR = (0, 0, 0)
CONF_THRESHOLD = 0.25           # Only draw detections above this confidence

# Standard YOLO/COCO class names mapping
COCO_CLASSES = {
    0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 
    6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 10: 'fire hydrant', 
    11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat', 
    16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear', 
    22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 
    27: 'tie', 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard', 
    32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 
    36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 
    40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 
    46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 
    51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair', 
    57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table', 61: 'toilet', 
    62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 
    68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 
    73: 'book', 74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 
    78: 'hair drier', 79: 'toothbrush'
}

def draw_detections(img, detections):
    """
    Overlays YOLO bounding boxes, class labels, and confidence scores
    from a vision_msgs/Detection2DArray message onto a BGR image.
 
    Detection2D structure:
      - bbox: BoundingBox2D  (center.x, center.y, size_x, size_y)  — all in pixels
      - results[0].hypothesis.class_id  (string label)
      - results[0].hypothesis.score     (float confidence)
    """
    if detections is None:
        return img
 
    img_h, img_w = img.shape[:2]
 
    for det in detections.detections:
        if not det.results:
            continue
 
        hyp = det.results[0].hypothesis
        class_id = hyp.class_id
        score = hyp.score
 
        if score < CONF_THRESHOLD:
            continue
 
        # BoundingBox2D uses center + size (pixels)
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        bw = det.bbox.size_x
        bh = det.bbox.size_y
 
        x1 = int(cx - bw / 2)
        y1 = int(cy - bh / 2)
        x2 = int(cx + bw / 2)
        y2 = int(cy + bh / 2)
 
        # Clamp to image bounds
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(img_w - 1, x2), min(img_h - 1, y2)
 
        # Draw bounding box
        cv2.rectangle(img, (x1, y1), (x2, y2), BOX_COLOR, BOX_THICKNESS)
 
        # Build label string: "person 0.91"
        label = f"{COCO_CLASSES[int(class_id)]} {score:.2f}"
 
        (lw, lh), baseline = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICKNESS)
        label_y = max(y1, lh + 4)
 
        # Draw filled label background
        cv2.rectangle(img,
                      (x1, label_y - lh - baseline - 2),
                      (x1 + lw, label_y),
                      LABEL_BG_COLOR, cv2.FILLED)
 
        # Draw label text
        cv2.putText(img, label,
                    (x1, label_y - baseline),
                    FONT, FONT_SCALE, LABEL_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)
 
    return img

def fig_to_img(fig):
    """Converts a Matplotlib figure into a raw OpenCV BGR image array."""
    fig.canvas.draw()
    w, h = fig.canvas.get_width_height()
    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8).reshape((h, w, 3))
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def stitch_videos(output_dir):
    """Post-processes the synchronized MP4s into a single dashboard view."""
    cam_path = os.path.join(output_dir, "camera_export.mp4")
    radar_path = os.path.join(output_dir, "radar_export.mp4")
    servo_path = os.path.join(output_dir, "servo_export.mp4")
    out_path = os.path.join(output_dir, "stitched_export.mp4")

    print("\nStitching final dashboard video... Please wait.")

    # Open the synchronized videos
    cam_cap = cv2.VideoCapture(cam_path)
    radar_cap = cv2.VideoCapture(radar_path)
    servo_cap = cv2.VideoCapture(servo_path)

    if not cam_cap.isOpened():
        print("Error: Could not open camera video for stitching.")
        return

    # Use the camera's resolution as the baseline reference
    w = int(cam_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cam_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cam_cap.get(cv2.CAP_PROP_FPS)

    # Master Canvas Size: 2x Width (Left/Right), 1x Height
    canvas_w = w * 2
    canvas_h = h
    
    # Calculate quadrant heights
    half_h = h // 2

    writer = cv2.VideoWriter(
        out_path, 
        cv2.VideoWriter_fourcc(*'mp4v'), 
        fps, 
        (canvas_w, canvas_h)
    )

    while True:
        ret_c, frame_c = cam_cap.read()
        ret_r, frame_r = radar_cap.read()
        ret_s, frame_s = servo_cap.read()

        # Camera is our master timeline
        if not ret_c:
            break

        # Fallbacks in case of fractional frame differences
        if not ret_r: frame_r = np.zeros((half_h, w, 3), dtype=np.uint8)
        else: frame_r = cv2.resize(frame_r, (w, h - half_h))

        if not ret_s: frame_s = np.zeros((half_h, w, 3), dtype=np.uint8)
        else: frame_s = cv2.resize(frame_s, (w, half_h))

        # 1. Create a blank black canvas
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        
        # 2. Left Half (Full Height): Camera
        canvas[:, :w] = frame_c
        
        # 3. Top Right Quarter: Servo Graph
        canvas[:half_h, w:] = frame_s
        
        # 4. Bottom Right Quarter: Radar Point Cloud
        canvas[half_h:, w:] = frame_r

        writer.write(canvas)

    cam_cap.release()
    radar_cap.release()
    servo_cap.release()
    writer.release()
    
    print(f"Stitching complete! Final video saved to: {out_path}")

def main(args=None):
    # Apply runtime args
    global CONF_THRESHOLD
    parser = argparse.ArgumentParser(description="Generate synchronized animated videos from MCAP data.")
    parser.add_argument("bag_path", type=str, help="Path to the bag directory.")
    parser.add_argument("-o", "--output", type=str, default="exports", 
                        help="Directory to save the mp4s (default: ./exports)")
    parser.add_argument("--yolo-topic", type=str, default="/video/detections",
                        help="ROS topic name for YOLO Detection2DArray")
    parser.add_argument("--conf", type=float, default=CONF_THRESHOLD,
                        help=f"Minimum confidence threshold for drawing detections (default: {CONF_THRESHOLD})")
    
    parsed_args, _ = parser.parse_known_args(args=sys.argv[1:])
    os.makedirs(parsed_args.output, exist_ok=True)
    
    
    CONF_THRESHOLD = parsed_args.conf
    TOPIC_TYPE_MAP[parsed_args.yolo_topic] = Detection2DArray

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=parsed_args.bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    # Telemetry Tracking
    start_time_ns = None
    servo_times, servo_positions = [], []
    
    # Sync Engine Variables
    cam_frame_count, radar_frame_count, servo_frame_count = 0, 0, 0
    last_cam_img, last_radar_img, last_servo_img = None, None, None
    
    # Video Writers
    cam_writer, radar_writer, servo_writer = None, None, None
    
    # Setup Matplotlib Figures
    radar_fig = plt.figure(figsize=(9, 8))
    radar_ax = radar_fig.add_subplot(111, projection="3d")
    plt.subplots_adjust(bottom=0.1)

    servo_fig = plt.figure(figsize=(8, 4))
    servo_ax = servo_fig.add_subplot(111)

    print(f"Starting synchronized video generation from MCAP stream...")
    print(f"Exports will be saved to: {parsed_args.output}/")
    print(f"YOLO detections topic: {parsed_args.yolo_topic}")
    print(f"Confidence threshold:   {CONF_THRESHOLD}")

    
    msg_count = 0
    det_count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_count += 1
        
        if msg_count % 500 == 0:
            print(f"Processed {msg_count} messages...")

        if start_time_ns is None:
            start_time_ns = timestamp
            
        relative_time_sec = (timestamp - start_time_ns) / 1e9
        expected_frame_idx = int(relative_time_sec * TARGET_FPS)
        
        if topic not in TOPIC_TYPE_MAP:
            continue
            
        msg = deserialize_message(data, TOPIC_TYPE_MAP[topic])

        # latest_detections = None
        if topic == parsed_args.yolo_topic:
            latest_detections = msg
            det_count += 1
            continue
        
        # --- 1. GoPro Video Processing ---
        if topic == '/video/yolov26_image/compressed':
            if expected_frame_idx < cam_frame_count:
                continue # Skip processing if sensor is running ahead of the clock
                
            np_arr = np.array(msg.data, dtype=np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is not None:
                if latest_detections is not None:
                    img = draw_detections(img, latest_detections)
                if cam_writer is None:
                    h, w = img.shape[:2]
                    cam_writer = cv2.VideoWriter(
                        os.path.join(parsed_args.output, "camera_export.mp4"), 
                        cv2.VideoWriter_fourcc(*'mp4v'), TARGET_FPS, (w, h))
                
                # Padding Engine: Fill gap if packets were dropped
                while cam_frame_count < expected_frame_idx:
                    if last_cam_img is not None:
                        cam_writer.write(last_cam_img)
                    cam_frame_count += 1
                    
                cam_writer.write(img)
                last_cam_img = img
                cam_frame_count += 1
                
        # --- 2. 3D Radar Processing ---
        elif topic == '/radar/points':
            if expected_frame_idx < radar_frame_count:
                continue
                
            points_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            frame_points = np.array([[p[0], p[1], p[2]] for p in points_iter], dtype=np.float32)
            
            radar_ax.cla()
            if len(frame_points) > 0:
                xs = frame_points[:, 1]  # Plot X-Axis = Side
                ys = frame_points[:, 0]  # Plot Y-Axis = Forward
                zs = frame_points[:, 2]  # Plot Z-Axis = Height
                radar_ax.scatter(xs, ys, zs, s=25, alpha=0.75, color='royalblue')

            radar_ax.scatter([0], [0], [0], s=100, marker="^", color='red')
            radar_ax.text(0, 0, 0, " Radar", color='black')

            radar_ax.set_title("3D Radar Point Cloud")
            radar_ax.set_xlabel("Side (m)")
            radar_ax.set_ylabel("Forward (m)")
            radar_ax.set_zlabel("Height (m)")
            radar_ax.set_xlim(-SIDE_LIMIT_M, SIDE_LIMIT_M)
            radar_ax.set_ylim(FORWARD_MIN_M, FORWARD_MAX_M)
            radar_ax.set_zlim(HEIGHT_MIN_M, HEIGHT_MAX_M)
            radar_ax.set_box_aspect((SIDE_LIMIT_M * 2, FORWARD_MAX_M - FORWARD_MIN_M, HEIGHT_MAX_M - HEIGHT_MIN_M))
            radar_ax.view_init(elev=FIXED_ELEVATION, azim=FIXED_AZIMUTH)

            r_img = fig_to_img(radar_fig)
            if radar_writer is None:
                rh, rw = r_img.shape[:2]
                radar_writer = cv2.VideoWriter(
                    os.path.join(parsed_args.output, "radar_export.mp4"), 
                    cv2.VideoWriter_fourcc(*'mp4v'), TARGET_FPS, (rw, rh))
            
            # Padding Engine
            while radar_frame_count < expected_frame_idx:
                if last_radar_img is not None:
                    radar_writer.write(last_radar_img)
                radar_frame_count += 1
                
            radar_writer.write(r_img)
            last_radar_img = r_img
            radar_frame_count += 1

        elif topic == '/servo/joint_states':
            if len(msg.position) > 0:
                # Apply modulo 2*pi (approx 6.283) to keep values in [0, 2pi)
                pos_mod = msg.position[0] % (2.0 * np.pi)
                
                servo_times.append(relative_time_sec)
                servo_positions.append(pos_mod) # Appending the wrapped value
                
                if expected_frame_idx < servo_frame_count:
                    continue 
                
                servo_ax.cla()
                servo_ax.plot(servo_times, servo_positions, linewidth=3, color='crimson')
                servo_ax.set_title("Servo Position (Mod 2pi)")
                servo_ax.set_xlabel("Time (s)")
                servo_ax.set_ylabel("Angle (rad)")
                
                servo_ax.set_xlim(0, max(5.0, relative_time_sec + 1.0))
                
                # Modulo ensures range is always [0, 6.28]
                servo_ax.set_ylim(-0.5, 7.0) 
                servo_ax.grid(True)
                
                s_img = fig_to_img(servo_fig)
                if servo_writer is None:
                    sh, sw = s_img.shape[:2]
                    servo_writer = cv2.VideoWriter(
                        os.path.join(parsed_args.output, "servo_export.mp4"), 
                        cv2.VideoWriter_fourcc(*'mp4v'), TARGET_FPS, (sw, sh))
                        
                # Padding Engine
                while servo_frame_count < expected_frame_idx:
                    if last_servo_img is not None:
                        servo_writer.write(last_servo_img)
                    servo_frame_count += 1
                    
                servo_writer.write(s_img)
                last_servo_img = s_img
                servo_frame_count += 1

    # Clean up
    if cam_writer: cam_writer.release()
    if radar_writer: radar_writer.release()
    if servo_writer: servo_writer.release()
    
    plt.close(radar_fig)
    plt.close(servo_fig)
    
    print("\nVideo generation complete!")
    print(f"Final Synchronized Frame Counts:")
    print(f"  Camera: {cam_frame_count} frames")
    print(f"  Radar:  {radar_frame_count} frames")
    print(f"  Servo:  {servo_frame_count} frames")

    stitch_videos(parsed_args.output)
if __name__ == '__main__':
    main()