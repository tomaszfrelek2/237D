import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import threading
import requests
import time
import os

# from bagpy import bagreader
# import pandas as pd

# b = bagreader('test.bag')

def keep_alive():
    while True:
        try:
            requests.get("http://172.28.114.51:8080/gopro/camera/keep_alive", timeout=1)
        except:
            pass
        time.sleep(2.5)

threading.Thread(target=keep_alive, daemon=True).start()


fourcc = cv2.VideoWriter_fourcc(*'mp4v')


class VideoPublisher(Node):
    def __init__(self):
        super().__init__(node_name="video_publisher")
        
        self.declare_parameter("otopic_video", "/video")
        self.otopic_video = self.get_parameter("otopic_video").value

        #stream URLs
        self.STREAM_URLS = [
            "udp://172.28.114.51:8554",
            "rtsp://172.28.114.51:8554/live",
            "udp://@:8554",
        ]
        self.cv_bridge = CvBridge()

        self.frame_publisher = self.create_publisher(
            Image,
            self.otopic_video,
            10
        )
        time_period = 1/30
        
        self.timer = self.create_timer(time_period, self.load_video)
        self.img = Image()
        self.get_logger().info("VideoPublisherNode has started")
        

    def publish_frames(self, opencv_img):
        img_msg: Image = self.cv_bridge.cv2_to_imgmsg(opencv_img, encoding="passthrough")

        self.frame_publisher.publish(img_msg)
        
    def open_stream(self, urls):
        for url in urls:
            print(f"Trying: {url}")
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            claimed_fps = cap.get(cv2.CAP_PROP_FPS)
            if cap.isOpened():
                print(f"Connected: {url}")
                print(f"Claimed FPS: {claimed_fps}")
                return cap
        return None


    def load_image(self):
        img = cv2.imread(self.ifile_video)

        sleep_time = (1.0/60.0)

        while True:
            self.publish_frames(img)
            time.sleep(sleep_time)


    def load_video(self):
        cap = self.open_stream(self.STREAM_URLS)
        if not cap:
            raise RuntimeError("Could not connect to GoPro stream")

        try:
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error("Could not read frame.")

            self.publish_frames(frame)

        finally:
            cap.release()
            self.get_logger().info("Video finished.")

def main(args=None):
    rclpy.init(args=args)

    video_publisher = VideoPublisher()

    rclpy.spin(video_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_publisher.destroy_node()
    
    
    requests.get("http://172.28.114.51:8080/gopro/camera/stream/stop")
    rclpy.shutdown()


if __name__ == '__main__':
    main()