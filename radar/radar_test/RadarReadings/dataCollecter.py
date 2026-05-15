import serial
import struct
import math
import time
import csv

# =========================
# Serial settings
# =========================
PORT = "/dev/ttyXRUSB1"      # Radar DATA port
BAUD = 921600               # Default TI mmWave data baud rate

# =========================
# Packet settings
# =========================
MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_SIZE = 40

TLV_DETECTED_POINTS = 1
TLV_SIDE_INFO = 7

# =========================
# Logging settings
# =========================
SAVE_TO_CSV = True
CSV_FILENAME = "radar_points.csv"


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
            # Each detected point is 16 bytes:
            # float32 x, float32 y, float32 z, float32 velocity
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
                # raw_y = forward
                # -raw_x = side
                # raw_z = height
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
            # Each side-info entry is 4 bytes:
            # uint16 snr, uint16 noise
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


def open_csv():
    csv_file = open(CSV_FILENAME, "w", newline="")
    writer = csv.writer(csv_file)

    writer.writerow([
        "timestamp",
        "frame",
        "point_id",
        "raw_x_m",
        "raw_y_m",
        "raw_z_m",
        "forward_m",
        "side_m",
        "height_m",
        "range_m",
        "velocity_mps",
        "snr",
        "noise"
    ])

    return csv_file, writer


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    buffer = bytearray()

    csv_file = None
    writer = None

    if SAVE_TO_CSV:
        csv_file, writer = open_csv()
        print(f"Saving radar points to {CSV_FILENAME}")

    print(f"Listening on {PORT} at {BAUD} baud...")
    print("Collecting all detected radar points.")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            data = ser.read(4096)

            if data:
                buffer.extend(data)

            # Find start of packet
            magic_index = buffer.find(MAGIC_WORD)

            if magic_index == -1:
                # Keep last few bytes in case magic word is split across reads
                if len(buffer) > 8:
                    buffer = buffer[-8:]
                continue

            # Remove garbage before magic word
            if magic_index > 0:
                del buffer[:magic_index]

            # Need full header before packet length can be read
            if len(buffer) < HEADER_SIZE:
                continue

            # totalPacketLen is bytes 12-15 from packet start
            total_packet_len = struct.unpack_from("<I", buffer, 12)[0]

            # Basic sanity check
            if total_packet_len < HEADER_SIZE or total_packet_len > 65535:
                del buffer[0]
                continue

            # Wait until full packet arrives
            if len(buffer) < total_packet_len:
                continue

            # Extract one full packet
            packet = bytes(buffer[:total_packet_len])
            del buffer[:total_packet_len]

            timestamp = time.time()
            frame_number, radar_points = parse_packet(packet)

            if frame_number is None:
                continue

            print(f"Frame {frame_number} | points={len(radar_points)}")

            for point in radar_points:
                print(
                    f"  Point {point['id']} | "
                    f"raw=({point['raw_x']:.2f}, {point['raw_y']:.2f}, {point['raw_z']:.2f}) m | "
                    f"forward={point['forward']:.2f} m, "
                    f"side={point['side']:.2f} m, "
                    f"height={point['height']:.2f} m, "
                    f"range={point['range']:.2f} m, "
                    f"v={point['velocity']:.2f} m/s, "
                    f"snr={point['snr']}, "
                    f"noise={point['noise']}"
                )

                if SAVE_TO_CSV:
                    writer.writerow([
                        timestamp,
                        frame_number,
                        point["id"],
                        point["raw_x"],
                        point["raw_y"],
                        point["raw_z"],
                        point["forward"],
                        point["side"],
                        point["height"],
                        point["range"],
                        point["velocity"],
                        point["snr"],
                        point["noise"]
                    ])

            if SAVE_TO_CSV:
                csv_file.flush()

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        ser.close()

        if csv_file is not None:
            csv_file.close()


if __name__ == "__main__":
    main()