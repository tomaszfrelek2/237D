import serial
import struct
import math
import time
from sklearn.cluster import DBSCAN

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
# Front-zone filter settings
# =========================
MIN_FORWARD_M = 0.10
MAX_FORWARD_M = 1
MAX_SIDE_M = 0.4
MIN_HEIGHT_M = -1
MAX_HEIGHT_M = 0.50

# =========================
# Obstacle logic
# =========================
DANGER_FORWARD_M = 0.50          # warning distance
REQUIRED_HIT_FRAMES = 3          # obstacle must appear this many times
REQUIRED_CLEAR_FRAMES = 3        # clear must appear this many times

PRINT_DELAY_SEC = 0.5            # faster for testing

# =========================
# DBSCAN settings
# =========================
DBSCAN_EPS_M = 0.2            # x cm grouping distance
DBSCAN_MIN_SAMPLES = 2           # x nearby points form a cluster


def parse_packet(packet):
    """
    Parse one complete TI mmWave UART packet.

    Returns:
        frame_number, radar_points

    Each radar point has:
        forward, side, height, velocity, range, snr, noise
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

                # Scooter-friendly coordinates
                # forward = in front of scooter
                # side    = left/right
                # height  = vertical
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
                    "noise": None
                })

        elif tlv_type == TLV_SIDE_INFO:
            for i in range(num_points):
                info_offset = i * 4

                if info_offset + 4 > len(payload):
                    break

                snr, noise = struct.unpack_from("<HH", payload, info_offset)
                side_info.append((snr, noise))

        offset = payload_end

    for i in range(min(len(radar_points), len(side_info))):
        radar_points[i]["snr"] = side_info[i][0]
        radar_points[i]["noise"] = side_info[i][1]

    return frame_number, radar_points


def get_front_points(radar_points):
    """
    Keep only radar points inside the front detection zone.
    """

    front_points = []

    for point in radar_points:
        if (
            MIN_FORWARD_M <= point["forward"] <= MAX_FORWARD_M
            and abs(point["side"]) <= MAX_SIDE_M
            and MIN_HEIGHT_M <= point["height"] <= MAX_HEIGHT_M
        ):
            front_points.append(point)

    return front_points


def cluster_front_points(front_points):
    """
    Group nearby front-zone radar points using DBSCAN.

    Returns:
        clusters, noise_points
    """

    if not front_points:
        return [], []

    points_2d = [
        [point["forward"], point["side"]]
        for point in front_points
    ]

    db = DBSCAN(
        eps=DBSCAN_EPS_M,
        min_samples=DBSCAN_MIN_SAMPLES
    ).fit(points_2d)

    labels = db.labels_

    clusters_dict = {}
    noise_points = []

    for point, label in zip(front_points, labels):
        if label == -1:
            noise_points.append(point)
            continue

        if label not in clusters_dict:
            clusters_dict[label] = []

        clusters_dict[label].append(point)

    return list(clusters_dict.values()), noise_points


def get_closest_front_distance(front_points):
    """
    Return closest forward distance from raw front-zone radar points.
    """

    if not front_points:
        return None

    closest_point = min(front_points, key=lambda point: point["forward"])
    return closest_point["forward"]


def get_closest_cluster_distance(clusters):
    """
    Return closest forward distance from clustered points.
    """

    if not clusters:
        return None

    closest_distance = None

    for cluster in clusters:
        cluster_closest_forward = min(point["forward"] for point in cluster)

        if closest_distance is None or cluster_closest_forward < closest_distance:
            closest_distance = cluster_closest_forward

    return closest_distance


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    buffer = bytearray()

    print(f"Listening on {PORT} at {BAUD} baud...")
    print("Front-zone:")
    print(f"  forward: {MIN_FORWARD_M:.2f} m to {MAX_FORWARD_M:.2f} m")
    print(f"  side:    +/- {MAX_SIDE_M:.2f} m")
    print(f"  height:  {MIN_HEIGHT_M:.2f} m to {MAX_HEIGHT_M:.2f} m")
    print()
    print("Obstacle logic:")
    print(f"  danger distance: {DANGER_FORWARD_M:.2f} m")
    print(f"  required hit frames: {REQUIRED_HIT_FRAMES}")
    print(f"  required clear frames: {REQUIRED_CLEAR_FRAMES}")
    print()


    last_print_time = 0

    hit_count = 0
    clear_count = 0
    obstacle_active = False

    while True:
        data = ser.read(4096)

        if data:
            buffer.extend(data)

        # Find packet start
        magic_index = buffer.find(MAGIC_WORD)

        if magic_index == -1:
            if len(buffer) > 8:
                buffer = buffer[-8:]
            continue

        if magic_index > 0:
            del buffer[:magic_index]

        if len(buffer) < HEADER_SIZE:
            continue

        total_packet_len = struct.unpack_from("<I", buffer, 12)[0]

        if total_packet_len < HEADER_SIZE or total_packet_len > 65535:
            del buffer[0]
            continue

        if len(buffer) < total_packet_len:
            continue

        packet = bytes(buffer[:total_packet_len])
        del buffer[:total_packet_len]

        frame_number, radar_points = parse_packet(packet)

        if frame_number is None:
            continue

        front_points = get_front_points(radar_points)
        clusters, noise_points = cluster_front_points(front_points)

        closest_raw_front = get_closest_front_distance(front_points)
        closest_cluster_front = get_closest_cluster_distance(clusters)

        # Main safety signal:
        # use raw points so sparse detections are not hidden by DBSCAN
        danger_detected = (
            closest_raw_front is not None
            and closest_raw_front <= DANGER_FORWARD_M
        )

        if danger_detected:
            hit_count += 1
            clear_count = 0
        else:
            clear_count += 1
            hit_count = 0

        if hit_count >= REQUIRED_HIT_FRAMES:
            obstacle_active = True

        if clear_count >= REQUIRED_CLEAR_FRAMES:
            obstacle_active = False

        now = time.time()
        should_print = (now - last_print_time) >= PRINT_DELAY_SEC

        if not should_print:
            continue

        last_print_time = now

        if obstacle_active:
            status = "obstacle"
        elif danger_detected:
            status = "maybe"
        else:
            status = "clear"

        raw_dist_text = (
            f"{closest_raw_front:.2f} m"
            if closest_raw_front is not None
            else "none"
        )

        cluster_dist_text = (
            f"{closest_cluster_front:.2f} m"
            if closest_cluster_front is not None
            else "none"
        )

        print(

            f"clusters={len(clusters)} | "
            f"closest_raw={raw_dist_text} | "
            f"closest_cluster={cluster_dist_text} | "
            f"status={status}"
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")