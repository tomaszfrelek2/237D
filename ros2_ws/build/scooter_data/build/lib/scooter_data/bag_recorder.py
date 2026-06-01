# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py
import argparse
import signal
import sys

# Image Imports
from sensor_msgs.msg import CompressedImage

# Radar Imports
from sensor_msgs.msg import PointCloud2
from radar_msgs.msg import RadarScan

# Servo Imports
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

#YOLO Imports
from vision_msgs.msg import Detection2DArray


class BagRecorder(Node):

    def __init__(self, bag_name):
        super().__init__('simple_bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()
        
        self.get_logger().info("Bag Recorder initialized. Opening bag file for writing...")

        # Upgraded to MCAP storage plugin
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_name,
            storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        
        self.get_logger().info("Writer opened successfully. Registering topics and creating publishers...")
        
        # 1. Register Topics
        self._register_topic(0, '/video/compressed', 'sensor_msgs/msg/CompressedImage')
        self._register_topic(1, '/radar/scan', 'radar_msgs/msg/RadarScan')
        self._register_topic(2, '/radar/points', 'sensor_msgs/msg/PointCloud2')
        
        self._register_topic(3, '/servo/joint_states', 'sensor_msgs/msg/JointState')
        self._register_topic(4, '/servo/temperature', 'std_msgs/msg/Float32')
        self._register_topic(5, '/servo/voltage', 'std_msgs/msg/Float32')
        self._register_topic(6, '/servo/current', 'std_msgs/msg/Float32')
        
        self._register_topic(7, '/video/detections', 'vision_msgs/msg/Detection2DArray')
        self._register_topic(8, '/video/yolov26_image/compressed', 'sensor_msgs/msg/CompressedImage')

        # 2. Create Subscriptions
        self.create_subscription(CompressedImage, '/video/compressed', self._make_callback('/video/compressed'), 10)
        self.create_subscription(RadarScan, '/radar/scan', self._make_callback('/radar/scan'), 10)
        self.create_subscription(PointCloud2, '/radar/points', self._make_callback('/radar/points'), 10)
        
        self.create_subscription(JointState, '/servo/joint_states', self._make_callback('/servo/joint_states'), 10)
        self.create_subscription(Float32, '/servo/temperature', self._make_callback('/servo/temperature'), 10)
        self.create_subscription(Float32, '/servo/voltage', self._make_callback('/servo/voltage'), 10)
        self.create_subscription(Float32, '/servo/current', self._make_callback('/servo/current'), 10)
        
        self.create_subscription(Detection2DArray,'/video/detections', self._make_callback('/video/detections'), 10)
        self.create_subscription(CompressedImage,'/video/yolov26_image/compressed', self._make_callback('/video/yolov26_image/compressed'), 10)
        
        self.get_logger().info("All topics registered and subscriptions created. Bag Recorder is now recording...")
        
    def _register_topic(self, id, name, type):
        topic_info = rosbag2_py.TopicMetadata(
            id=id,
            name=name,
            type=type,
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

    def _make_callback(self, topic_name):
        def callback(msg):
            self.writer.write(
                topic_name,
                serialize_message(msg),
                self.get_clock().now().nanoseconds)
        return callback

    # def destroy_node(self):
    #     """Forces the MCAP writer to close the file handles properly before exiting."""
    #     print("\n[FLUSH] Closing MCAP bag file gracefully...")
    #     if hasattr(self, 'writer'):
    #         del self.writer
    #     super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description="Record a ROS2 bag.")
    parser.add_argument(
        "-n", "--name", 
        type=str, 
        default="my_default_bag", 
        help="The name/URI of the output bag directory."
    )
    
    parsed_args, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    sbr = BagRecorder(parsed_args.name)

    # Intercept system termination signals to guarantee destroy_node runs 
    def signal_handler(sig, frame):
        sbr.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(sbr)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if 'sbr' in locals():
            try:
                sbr.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()