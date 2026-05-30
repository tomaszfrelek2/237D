#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/ubuntu/237D/camera/cam_env/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from functools import partial
import json
from rclpy.executors import ExternalShutdownException
from scooter_data.camera_yolo26 import YOLODetector
import os
from datetime import datetime

class CameraYOLO26Node(Node):
    def __init__(self):
        super().__init__('camera_yolo26_node')
        
        self.model_name = "yolo26l.pt"
        self.output_image = True
        self.color = (0, 255, 0)
        self.thickness = 2
        self.show_label = True

        self.topic_list = ['/video/compressed']
        
        self.json_dir = "json_dir/"
        os.makedirs(self.json_dir, exist_ok=True)
        session = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.json_path = os.path.join(self.json_dir, f'detections_{session}.json')
        self.json_file = open(self.json_path, 'w')
        self.json_file.write('[\n')
        self.first_entry = True
        self.get_logger().info(f"Saving detections to {self.json_path}")
        
        self.sub_image = {}
        self.pub_detection = {}
        self.pub_image = {}
        self.bridge = CvBridge()
        self.detector = YOLODetector(model_name=self.model_name)

        for topic in self.topic_list:
            topic_clean = topic.replace('/compressed', '')
            
            # ROS 2 uses 'partial' to pass extra arguments to callbacks
            self.sub_image[topic] = self.create_subscription(
                CompressedImage,
                topic,
                partial(self.image_callback, topic=topic),
                10)
            
            self.pub_detection[topic] = self.create_publisher(
                Detection2DArray, 
                f"{topic_clean}/detections", 
                10)
            
            if self.output_image:
                self.pub_image[topic] = self.create_publisher(
                    CompressedImage, 
                    f"{topic_clean}/yolov26_image/compressed", 
                    10)

    def image_callback(self, msg, topic):
        self.get_logger().info(f"Received image from {topic}")

        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_in = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        outputs = self.detector.detect(image_in, show=False)
        
        image_out = np.array(image_in) if self.output_image else None

        det2d_array = Detection2DArray()
        det2d_array.header = msg.header 
        
        # JSON record for this frame
        frame_record = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'topic': topic,
            'detections': []
        }

        for boxes in outputs:
            for i in range(boxes.xyxy.shape[0]):
                x1, y1, x2, y2 = boxes.xyxy[i]
                
                cls_idx = int(boxes.cls[i])
                conf = float(boxes.conf[i])
                
                det2d = Detection2D()
                # ROS 2 BoundingBox2D uses center and size
                det2d.bbox.center.position.x = float((x1 + x2) / 2.0)
                det2d.bbox.center.position.y = float((y1 + y2) / 2.0)
                det2d.bbox.size_x = float(x2 - x1)
                det2d.bbox.size_y = float(y2 - y1)

                obj = ObjectHypothesisWithPose()
                obj.hypothesis.class_id = str(int(boxes.cls[i]))
                obj.hypothesis.score = float(boxes.conf[i])
                
                det2d.results.append(obj)
                det2d_array.detections.append(det2d)
                
                # Append to JSON record
                frame_record['detections'].append({
                    'class_id': cls_idx,
                    'class_name': self.detector.model.names[cls_idx],
                    'confidence': round(conf, 4),
                    'bbox': {
                        'x1': float(x1), 'y1': float(y1),
                        'x2': float(x2), 'y2': float(y2),
                        'cx': float((x1 + x2) / 2),
                        'cy': float((y1 + y2) / 2),
                        'w':  float(x2 - x1),
                        'h':  float(y2 - y1),
                    }
                })

                if self.output_image:
                    self.draw_detection(image_out, x1, y1, x2, y2, boxes.cls[i], boxes.conf[i])


        # Write frame record to JSON file
        if not self.first_entry:
            self.json_file.write(',\n')
        self.json_file.write(json.dumps(frame_record, indent=2))
        self.json_file.flush()
        self.first_entry = False
        
        self.pub_detection[topic].publish(det2d_array)
        
        if self.output_image:
            img_msg = self.bridge.cv2_to_compressed_imgmsg(image_out, "jpg")
            img_msg.header = msg.header
            self.pub_image[topic].publish(img_msg)

    def draw_detection(self, img, x1, y1, x2, y2, cls_idx, conf):
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), self.color, self.thickness)
        if self.show_label:
            label = f"{self.detector.model.names[int(cls_idx)]}: {conf:.2f}"
            cv2.putText(img, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color, 1)
            
    def destroy_node(self):
        # Close JSON file properly on shutdown
        self.json_file.write('\n]')
        self.json_file.close()
        self.get_logger().info(f"Detections saved to {self.json_path}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraYOLO26Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == "__main__":
    main()