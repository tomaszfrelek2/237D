#!/usr/bin/env python3
import os
import json
import cv2
import numpy as np
import argparse
from pathlib import Path
from rosbags.highlevel import AnyReader

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

def visualize_bag(bag_path, json_path, topic, output_video):
    print(f"Loading detections from: {json_path}")
    with open(json_path, 'r') as f:
        detections = json.load(f)
    
    print(f"Reading ROS bag: {bag_path}")
    print(f"Target topic: {topic}")
    
    video_writer = None
    frame_count = 0

    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic == topic]
        
        if not connections:
            print(f"Error: Topic '{topic}' not found in the bag.")
            return

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            timestamp_string = "{:.9f}".format(timestamp / 1e9)
            
            # Decode image (handles both CompressedImage and raw Image)
            np_arr = np.frombuffer(msg.data, np.uint8)
            if hasattr(msg, 'format'): 
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else: 
                img = np_arr.reshape((msg.height, msg.width, -1))
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
            # FIX: Ensure the image is writable before drawing on it
            img = img.copy()
            
            # Initialize VideoWriter on the first frame once we know the image dimensions
            if video_writer is None:
                height, width, _ = img.shape
                # mp4v is a standard codec that works on almost all systems
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                # Assuming ~30 FPS for the output video playback speed
                video_writer = cv2.VideoWriter(output_video, fourcc, 30.0, (width, height))
                print(f"Initializing video writer: {width}x{height} resolution.")

            # Draw bounding boxes if we have detections for this exact timestamp
            if timestamp_string in detections:
                for det in detections[timestamp_string]:
                    x1, y1, x2, y2, cls_id, conf = det
                    
                    # Draw rectangle
                    cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    
                    # Fetch class name, default to "Unknown" if not in dictionary
                    class_name = COCO_CLASSES.get(int(cls_id), f"Class {int(cls_id)}")
                    label = f"{class_name}: {conf:.2f}"
                    
                    # Draw label background for better readability
                    (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(img, (int(x1), int(y1) - label_h - 5), (int(x1) + label_w, int(y1)), (0, 255, 0), -1)
                    
                    # Draw text
                    cv2.putText(img, label, (int(x1), int(y1) - 5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            
            video_writer.write(img)
            frame_count += 1
            
            if frame_count % 100 == 0:
                print(f"Processed {frame_count} frames...")

    if video_writer:
        video_writer.release()
        print(f"Success! Video saved to: {output_video}")
    else:
        print("No frames were processed.")

def main():
    parser = argparse.ArgumentParser(description="Overlay YOLO JSON detections onto a rosbag video.")
    parser.add_argument('--bag_path', help='Path to the ROS bag directory')
    parser.add_argument('--json_path', help='Path to the generated JSON detections file')
    parser.add_argument('--topic', default='/oak_d_pro/rgb/image_raw', help='Image topic to visualize')
    parser.add_argument('--output', default='output_visualization.mp4', help='Name of the output MP4 file')
    
    args = parser.parse_args()
    
    visualize_bag(args.bag_path, args.json_path, args.topic, args.output)

if __name__ == '__main__':
    main()