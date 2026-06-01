# Copyright 2024
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

# Import the class you provided
from scooter_data.controller import ST3025_Controller

class ServoStatePublisher(Node):
    def __init__(self):
        super().__init__('servo_state_publisher')
        
        # 1. Connection Parameters
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14110532-if00')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('servo_id', 1)
        self.declare_parameter('enable_torque', False)
        self.declare_parameter('poll_interval', 0.1)

        # 2. Hardware Math Parameters
        self.declare_parameter('encoder_resolution', 4096.0) # 12-bit encoder
        self.declare_parameter('max_torque_nm', 3.92)        # 40 kg-cm converted to Nm
        self.declare_parameter('load_scale_max', 1000.0)     # Load reports 0-1000 for 0-100% effort

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.servo_id = self.get_parameter('servo_id').value
        enable_torque = self.get_parameter('enable_torque').value
        poll_interval = self.get_parameter('poll_interval').value

        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.max_torque_nm = self.get_parameter('max_torque_nm').value
        self.load_scale_max = self.get_parameter('load_scale_max').value

        # Setup Publishers
        self.joint_pub = self.create_publisher(JointState, '/servo/joint_states', 10)
        self.temp_pub = self.create_publisher(Float32, '/servo/temperature', 10)
        self.volt_pub = self.create_publisher(Float32, '/servo/voltage', 10)
        self.curr_pub = self.create_publisher(Float32, '/servo/current', 10)

        # Initialize the Hardware Controller
        try:
            self.servo = ST3025_Controller(port=port, baudrate=baudrate)
        except Exception as e:
            self.get_logger().fatal(f"Failed to open port: {e}")
            raise SystemExit

        if not self.servo.ping(self.servo_id):
            self.get_logger().fatal("Servo not responding to ping. Check wiring and ID.")
            self.servo.close()
            raise SystemExit

        # Configure Initial State
        self.servo.set_torque(self.servo_id, enable=enable_torque)
        self.get_logger().info(f"Connected to Servo {self.servo_id}. Torque enabled: {enable_torque}")

        # Initialize the standard ROS Timer 
        self.timer = self.create_timer(poll_interval, self.timer_callback)

    def timer_callback(self):
        data = self.servo.get_feedback(self.servo_id)
        
        if data:
            now = self.get_clock().now().to_msg()

            # --- Apply SI Unit Conversions ---
            raw_pos = float(data['Position'])
            raw_vel = float(data['Speed'])
            raw_load = float(data['Load'])

            # 4096 steps per revolution -> radians
            pos_rad = (raw_pos / self.encoder_resolution) * (2.0 * math.pi)
            
            # Assuming speed is measured in the same steps per second
            vel_rad_s = (raw_vel / self.encoder_resolution) * (2.0 * math.pi)
            
            # Raw load (0-1000) -> Percentage -> Newton-meters
            effort_nm = (raw_load / self.load_scale_max) * self.max_torque_nm

            # Publish Kinematics and Dynamics (JointState)
            joint_msg = JointState()
            joint_msg.header.stamp = now
            joint_msg.name = [f'servo_{self.servo_id}']
            joint_msg.position = [pos_rad]
            joint_msg.velocity = [vel_rad_s]
            joint_msg.effort = [effort_nm]
            self.joint_pub.publish(joint_msg)

            # Publish System Health Metrics
            temp_msg = Float32()
            temp_msg.data = float(data['Temperature'])
            self.temp_pub.publish(temp_msg)

            volt_msg = Float32()
            volt_msg.data = float(data['Voltage'])
            self.volt_pub.publish(volt_msg)

            curr_msg = Float32()
            curr_msg.data = float(data['Current'])
            self.curr_pub.publish(curr_msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down servo connection...")
        if hasattr(self, 'servo'):
            self.servo.set_torque(self.servo_id, enable=False)
            self.servo.close()
        super().destroy_node()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ServoStatePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 1. Safely destroy the node structure first
        if 'node' in locals():
            try:
                node.destroy_node()
            except Exception:
                pass # Prevent logging failures from crashing the shutdown sequence
                
        # 2. ONLY call shutdown if ROS2 is still actively running
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()