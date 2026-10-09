# Copyright 2024
# Licensed under the Apache License, Version 2.0 (the "License");
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import rosbag2_py

# Import all message types needed for deserialization and publishing
from sensor_msgs.msg import Image, PointCloud2, JointState
from radar_msgs.msg import RadarScan
from std_msgs.msg import Float32

# Map topics to their respective message types
TOPIC_TYPE_MAP = {
    'video': Image,
    '/radar/scan': RadarScan,
    '/radar/points': PointCloud2,
    '/servo/joint_states': JointState,
    '/servo/temperature': Float32,
    '/servo/voltage': Float32,
    '/servo/current': Float32,
}

# Map topics to logical sensor names for console output
SENSOR_MAP = {
    'video': 'GoPro Camera',
    '/radar/scan': 'TI mmWave Radar',
    '/radar/points': 'TI mmWave Radar',
    '/servo/joint_states': 'ST3025 Servo',
    '/servo/temperature': 'ST3025 Servo',
    '/servo/voltage': 'ST3025 Servo',
    '/servo/current': 'ST3025 Servo',
}

class BagReaderPublisher(Node):

    def __init__(self, bag_filename):
        super().__init__('bag_reader_publisher')
        
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_filename,
            storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)

        # Dynamically create a publisher for every topic in our map
        self.publishers_dict = {}
        for topic, msg_type in TOPIC_TYPE_MAP.items():
            self.publishers_dict[topic] = self.create_publisher(msg_type, topic, 10)
            self.get_logger().info(f"Created publisher for {topic}")

        # Timer to read and publish messages at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def format_value(self, topic, msg):
        """Formats the deserialized message to prevent console flooding."""
        if topic == 'video':
            return f"Frame ({msg.width}x{msg.height}, {msg.encoding})"
        elif topic == '/radar/points':
            return f"Cloud (width: {msg.width}, points: {msg.width * msg.height})"
        elif topic == '/radar/scan':
            return f"Scan ({len(msg.returns)} returns)"
        elif topic == '/servo/joint_states':
            return f"Pos: {msg.position[0]:.2f} rad | Vel: {msg.velocity[0]:.2f} rad/s | Load: {msg.effort[0]:.2f} Nm"
        elif topic.startswith('/servo/'):
            return f"{msg.data:.2f}"
        return str(msg)

    def timer_callback(self):
        if self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()
            
            # Only process topics we explicitly registered
            if topic in TOPIC_TYPE_MAP:
                msg_type = TOPIC_TYPE_MAP[topic]
                sensor = SENSOR_MAP[topic]
                
                # 1. Convert binary data back to a Python ROS message object
                deserialized_msg = deserialize_message(data, msg_type)
                
                # 2. Print the formatted value to the terminal
                val_str = self.format_value(topic, deserialized_msg)
                self.get_logger().info(f"[{sensor}] {topic} | {val_str}")
                
                # 3. Publish the valid Python object back to the ROS 2 network
                self.publishers_dict[topic].publish(deserialized_msg)
        else:
            self.get_logger().info('End of bag file reached.')
            self.timer.cancel()


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run scooter_data bag_reader <path_to_bag_file>")
        return

    try:
        rclpy.init(args=args)
        sbr = BagReaderPublisher(sys.argv[1])
        rclpy.spin(sbr)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()