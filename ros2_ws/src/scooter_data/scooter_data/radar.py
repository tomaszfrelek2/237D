import os
import time
import struct
import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from radar_msgs.msg import RadarScan, RadarReturn
from std_msgs.msg import Header

# Packet settings
MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_SIZE = 40

TLV_DETECTED_POINTS = 1
TLV_SIDE_INFO = 7

def parse_packet(packet):
    """
    Parse one complete TI mmWave UART packet.

    Returns:
        frame_number, radar_points

    Each radar point contains:
        raw_x, raw_y, raw_z, velocity
        forward, side, height
        range, snr, noise
    """
    if len(packet) < HEADER_SIZE:
        return None, []

    if packet[0:8] != MAGIC_WORD:
        return None, []

    try:
        (
            version,
            total_packet_len,
            platform,
            frame_number,
            time_cpu_cycles,
            num_points,
            num_tlv,
            subframe
        ) = struct.unpack_from("<IIIIIIII", packet, 8)
    except struct.error:
        return None, []

    radar_points = []
    side_info = []

    offset = HEADER_SIZE

    for _ in range(num_tlv):
        if offset + 8 > len(packet):
            break

        tlv_type, tlv_length = struct.unpack_from("<II", packet, offset)
        offset += 8

        payload_start = offset
        payload_end = payload_start + tlv_length

        if payload_end > len(packet):
            break

        payload = packet[payload_start:payload_end]

        if tlv_type == TLV_DETECTED_POINTS:
            for i in range(num_points):
                point_offset = i * 16

                if point_offset + 16 > len(payload):
                    break

                raw_x, raw_y, raw_z, velocity = struct.unpack_from(
                    "<ffff",
                    payload,
                    point_offset
                )

                point_range = math.sqrt(
                    raw_x * raw_x +
                    raw_y * raw_y +
                    raw_z * raw_z
                )

                # Scooter-friendly coordinate conversion
                forward = raw_y
                side = -raw_x
                height = raw_z

                radar_points.append({
                    "id": i,
                    "raw_x": raw_x,
                    "raw_y": raw_y,
                    "raw_z": raw_z,
                    "forward": forward,
                    "side": side,
                    "height": height,
                    "velocity": velocity,
                    "range": point_range,
                    "snr": None,
                    "noise": None,
                })

        elif tlv_type == TLV_SIDE_INFO:
            for i in range(num_points):
                info_offset = i * 4

                if info_offset + 4 > len(payload):
                    break

                snr, noise = struct.unpack_from("<HH", payload, info_offset)
                side_info.append((snr, noise))

        offset = payload_end

    # Attach SNR/noise to matching radar points
    for i in range(min(len(radar_points), len(side_info))):
        radar_points[i]["snr"] = side_info[i][0]
        radar_points[i]["noise"] = side_info[i][1]

    return frame_number, radar_points


class MmWaveRadarNode(Node):
    def __init__(self):
        super().__init__("mmwave_radar")

        # 1. Declare parameters using the custom udev symlinks
        self.declare_parameter("config_port", "/dev/radar_config")
        self.declare_parameter("data_port", "/dev/radar_data")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("frame_id", "radar_link")
        self.declare_parameter("cfg_file", "/home/ubuntu/237D/radar/radar_driver/2000039 (TI) xwr1843aop firmware-v03_06_01_00-LTS-1/profile_3d_aop.cfg")

        config_port = self.get_parameter("config_port").value
        data_port = self.get_parameter("data_port").value
        baud = self.get_parameter("baud").value
        self.frame_id = self.get_parameter("frame_id").value
        cfg_file = self.get_parameter("cfg_file").value

        # 2. Configure the radar FIRST
        self.configure_radar(config_port, cfg_file)

        # 3. Open the high-speed data port AFTER configuration is complete
        self.get_logger().info(f"Opening data port: {data_port} at {baud} baud")
        self.pub_scan  = self.create_publisher(RadarScan, "/radar/scan", 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/radar/points", 10)

        self.ser = serial.Serial(data_port, baud, timeout=0.05)
        self.buffer = bytearray()

        self.create_timer(0.005, self.read_serial)
        
    def configure_radar(self, config_port, cfg_file):
        """Replaces the external bash script by sending the .cfg file over the config port."""
        if not os.path.isfile(cfg_file):
            self.get_logger().error(f"Config file not found: {cfg_file}")
            raise FileNotFoundError(f"Missing config file at {cfg_file}")

        self.get_logger().info(f"Opening config port: {config_port}")
        
        try:
            # TI Config ports always operate at 115200
            cfg_serial = serial.Serial(config_port, 115200, timeout=1)
            
            with open(cfg_file, 'r') as file:
                lines = file.readlines()

            for line in lines:
                line = line.strip()
                
                if not line or line.startswith('%'):
                    continue

                self.get_logger().info(f"Sending config: {line}")
                cfg_serial.write((line + '\r\n').encode('utf-8'))
                time.sleep(0.05)
                
                while cfg_serial.in_waiting > 0:
                    response = cfg_serial.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        self.get_logger().debug(f"Radar reply: {response}")

            cfg_serial.close()
            self.get_logger().info("Radar configuration sent successfully.")
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to communicate with config port: {e}")
            raise

    def read_serial(self):
        data = self.ser.read(4096)
        if data:
            self.buffer.extend(data)

        magic_index = self.buffer.find(MAGIC_WORD)
        if magic_index == -1:
            if len(self.buffer) > 8:
                self.buffer = self.buffer[-8:]
            return

        if magic_index > 0:
            del self.buffer[:magic_index]

        if len(self.buffer) < HEADER_SIZE:
            return

        total_packet_len = struct.unpack_from("<I", self.buffer, 12)[0]

        if total_packet_len < HEADER_SIZE or total_packet_len > 65535:
            del self.buffer[0]
            return

        if len(self.buffer) < total_packet_len:
            return

        packet = bytes(self.buffer[:total_packet_len])
        del self.buffer[:total_packet_len]

        frame_number, radar_points = parse_packet(packet)
        if frame_number is None or not radar_points:
            return

        stamp = self.get_clock().now().to_msg()
        self.publish_scan(stamp, radar_points)
        self.publish_pointcloud(stamp, radar_points)

        self.get_logger().debug(
            f"Frame {frame_number} | {len(radar_points)} points"
        )
        
    def publish_scan(self, stamp, radar_points):
        msg = RadarScan()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)

        for p in radar_points:
            ret = RadarReturn()
            ret.range = float(p["range"])
            ret.azimuth = math.atan2(p["side"], p["forward"])
            ret.elevation = math.atan2(p["height"], p["range"]) if p["range"] > 0 else 0.0
            ret.doppler_velocity = float(p["velocity"])
            msg.returns.append(ret)

        self.pub_scan.publish(msg)
        
    def publish_pointcloud(self, stamp, radar_points):
        fields = [
            PointField(name="x",        offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",        offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",        offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="velocity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="range",    offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="snr",      offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name="noise",    offset=24, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 28

        buffer = bytearray()
        for p in radar_points:
            buffer += struct.pack(
                "<fffffff",
                p["forward"],
                p["side"],
                p["height"],
                p["velocity"],
                p["range"],
                float(p["snr"])   if p["snr"]   is not None else 0.0,
                float(p["noise"]) if p["noise"] is not None else 0.0,
            )

        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        msg.height = 1
        msg.width = len(radar_points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * len(radar_points)
        msg.data = bytes(buffer)
        msg.is_dense = True

        self.pub_cloud.publish(msg)
        
    def destroy_node(self):
        self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = MmWaveRadarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()