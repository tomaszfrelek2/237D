# Copyright 2024
# Licensed under the Apache License, Version 2.0 (the "License");
import sys
import threading
import requests
import time
import cv2
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def keep_alive():
    """
    Sends a heartbeat to the GoPro every 2.5 seconds so it doesn't close the stream.
    """
    while True:
        try:
            requests.get("http://172.28.114.51:8080/gopro/camera/keep_alive", timeout=1)
        except:
            pass
        time.sleep(2.5)

threading.Thread(target=keep_alive, daemon=True).start()

class VideoPublisher(Node):
    def __init__(self):
        super().__init__(node_name="video_publisher")
        
        self.declare_parameter("otopic_video", "/video")
        self.otopic_video = self.get_parameter("otopic_video").value

        # Frames are always read at camera_fps (keeps the UDP buffer empty);
        # publish_fps sets how many of them are published per second (e.g. 10, 20 or 30).
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("publish_fps", 10.0)
        self.camera_fps = float(self.get_parameter("camera_fps").value)
        self.publish_fps = float(self.get_parameter("publish_fps").value)
        if not 0 < self.publish_fps <= self.camera_fps:
            raise ValueError(f"publish_fps must be between 0 and camera_fps ({self.camera_fps}), "
                             f"got {self.publish_fps}")

        self.STREAM_URLS = [
            "udp://172.28.114.51:8554?fifo_size=50000000&overrun_nonfatal=1",
            "rtsp://172.28.114.51:8554/live",
            "udp://@:8554",
        ]
        
        self.cv_bridge = CvBridge()

        self.frame_publisher = self.create_publisher(
            CompressedImage,
            self.otopic_video + '/compressed',
            10
        )

        self.get_logger().info("Enabling GoPro wired USB control...")
        try:
            requests.get("http://172.28.114.51:8080/gopro/camera/control/wired_usb?p=1", timeout=3)
            time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"Failed to enable wired USB: {e}")

        self.get_logger().info("Sending start stream command to GoPro...")
        try:
            requests.get("http://172.28.114.51:8080/gopro/camera/stream/start", timeout=3)
            time.sleep(2)
        except Exception as e:
            self.get_logger().error(f"Failed to send start command: {e}")

        self.get_logger().info("Opening video stream connection...")
        self.cap = self.open_stream(self.STREAM_URLS)
        
        if not self.cap:
            self.get_logger().error("Could not connect to GoPro stream")
            sys.exit(1)

        self.get_logger().info("VideoPublisher node has successfully started reading frames.")

        # Keep polling the buffer rapidly to prevent OpenCV lag
        time_period = 1.0 / self.camera_fps
        self.publish_credit = 0.0
        self.timer = self.create_timer(time_period, self.load_video)
        self.get_logger().info(f"Reading at {self.camera_fps:g} FPS, publishing at {self.publish_fps:g} FPS")

    def publish_frames(self, opencv_img):
        """Compresses the raw image into a JPEG and publishes it."""
        img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(opencv_img, dst_format='jpg')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.frame_publisher.publish(img_msg)

    def open_stream(self, urls):
        """Attempts to connect to the GoPro UDP stream with zero buffering."""
        # Force FFMPEG to drop the UDP buffer and prioritize live latency
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay"
        
        for url in urls:
            self.get_logger().info(f"Trying: {url}")
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            if cap.isOpened():
                self.get_logger().info(f"Connected to: {url}")
                return cap
        return None

    def load_video(self):
        """Reads frames rapidly to prevent lag, but throttles publishing to publish_fps."""
        if not self.cap or not self.cap.isOpened():
            return

        # Always read the frame to keep the UDP buffer completely empty
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Dropped frame or stream paused.")
            return

        # Throttle: each read frame earns publish_fps "credit"; publish whenever a full
        # camera_fps worth has built up. 10 of 30 -> every 3rd frame, 20 of 30 -> 2 of every 3,
        # 30 of 30 -> every frame.
        self.publish_credit += self.publish_fps
        if self.publish_credit >= self.camera_fps:
            self.publish_credit -= self.camera_fps
            self.publish_frames(frame)

    def destroy_node(self):
        """Cleans up the OpenCV object on shutdown."""
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()

    try:
        rclpy.spin(video_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        video_publisher.destroy_node()
        
        print("\nShutting down GoPro stream...")
        try:
            requests.get("http://172.28.114.51:8080/gopro/camera/stream/stop", timeout=2)
        except:
            pass
            
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()