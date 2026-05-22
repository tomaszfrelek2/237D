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

# Existing Imports
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from radar_msgs.msg import RadarScan, RadarReturn
from std_msgs.msg import Header

# New Imports for Servo Data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class BagRecorder(Node):

    def __init__(self):
        super().__init__('simple_bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py.StorageOptions(
            uri='bag_2',
            storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        
        # 1. Register Topics
        self._register_topic(0, 'video', 'sensor_msgs/msg/Image')
        self._register_topic(1, '/radar/scan', 'radar_msgs/msg/RadarScan')
        self._register_topic(2, '/radar/points', 'sensor_msgs/msg/PointCloud2')
        
        self._register_topic(3, '/servo/joint_states', 'sensor_msgs/msg/JointState')
        self._register_topic(4, '/servo/temperature', 'std_msgs/msg/Float32')
        self._register_topic(5, '/servo/voltage', 'std_msgs/msg/Float32')
        self._register_topic(6, '/servo/current', 'std_msgs/msg/Float32')

        # 2. Create Subscriptions
        self.create_subscription(
            Image,
            'video',
            self._make_callback('/video'),
            10)
        self.create_subscription(RadarScan, '/radar/scan', self._make_callback('/radar/scan'), 10)
        self.create_subscription(PointCloud2, '/radar/points', self._make_callback('/radar/points'), 10)
        
        self.create_subscription(JointState, '/servo/joint_states', self._make_callback('/servo/joint_states'), 10)
        self.create_subscription(Float32, '/servo/temperature', self._make_callback('/servo/temperature'), 10)
        self.create_subscription(Float32, '/servo/voltage', self._make_callback('/servo/voltage'), 10)
        self.create_subscription(Float32, '/servo/current', self._make_callback('/servo/current'), 10)
        
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


def main(args=None):
    try:
        rclpy.init(args=args)
        sbr = BagRecorder()
        rclpy.spin(sbr)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()