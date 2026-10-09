#!/usr/bin/env python3
# Copyright 2024
# Licensed under the Apache License, Version 2.0 (the "License");
"""
Export the camera and semantic segmentation from a recorded bag (MCAP or sqlite3) to video.

Outputs (in --output, default ./seg_exports):
  raw_camera_export.mp4     the camera stream
  segmentation_export.mp4   segmentation overlay on the camera image
  side_by_side_export.mp4   raw | segmentation | class legend

The overlay comes from /video/segmentation_image/compressed if it was recorded. Otherwise
it is drawn here from the class map (/video/segmentation) on top of the camera frame with
the same timestamp (the segmentation node copies the camera frame's header).
All videos run at --fps on the bag's time axis (frames are repeated to fill gaps), so they
stay in sync with each other and with real time.

Usage:
  ros2 run scooter_data segmentation_playback <bag_dir> [-o seg_exports] [--labels-file config_v1.2.json]
"""
import argparse
import json
import os
import sys
from collections import OrderedDict

import cv2
import numpy as np

SEG_IGNORE = 255  # class-map value for classes filtered out by the node (class_ids)

# Mapillary Vistas v1.2 order, same as scooter_data/semantic_segmenter.py
VISTAS_V12_NAMES = [
    "Bird", "Ground Animal", "Curb", "Fence", "Guard Rail", "Barrier", "Wall", "Bike Lane",
    "Crosswalk - Plain", "Curb Cut", "Parking", "Pedestrian Area", "Rail Track", "Road",
    "Service Lane", "Sidewalk", "Bridge", "Building", "Tunnel", "Person", "Bicyclist",
    "Motorcyclist", "Other Rider", "Lane Marking - Crosswalk", "Lane Marking - General",
    "Mountain", "Sand", "Sky", "Snow", "Terrain", "Vegetation", "Water", "Banner", "Bench",
    "Bike Rack", "Billboard", "Catch Basin", "CCTV Camera", "Fire Hydrant", "Junction Box",
    "Mailbox", "Manhole", "Phone Booth", "Pothole", "Street Light", "Pole",
    "Traffic Sign Frame", "Utility Pole", "Traffic Light", "Traffic Sign (Back)",
    "Traffic Sign (Front)", "Trash Can", "Bicycle", "Boat", "Bus", "Car", "Caravan",
    "Motorcycle", "On Rails", "Other Vehicle", "Trailer", "Truck", "Wheeled Slow",
    "Car Mount", "Ego Vehicle",
]
FONT = cv2.FONT_HERSHEY_SIMPLEX


# --------------------------------------------------------------------------
# Segmentation helpers
# --------------------------------------------------------------------------
def load_seg_labels(labels_file):
    """Class names and BGR colours matching the segmentation node's overlay."""
    names = list(VISTAS_V12_NAMES)
    palette = np.random.RandomState(12345).randint(40, 256, (256, 3)).astype(np.uint8)
    if labels_file:
        with open(labels_file) as f:
            cfg = json.load(f)
        labels = cfg["labels"] if isinstance(cfg, dict) else cfg
        for i, lab in enumerate(labels[:256]):
            if isinstance(lab, dict):
                if i < len(names):
                    names[i] = lab.get("readable", lab.get("name", names[i]))
                if "color" in lab:
                    palette[i] = lab["color"]
    return names, palette[:, ::-1].copy()


def class_map_from_msg(msg):
    """sensor_msgs/Image mono8 -> (H, W) uint8 array (handles row padding)."""
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    return arr[:, :msg.width]


def seg_overlay(bgr, class_map, palette, alpha):
    """Colour the class map onto the camera frame; filtered-out pixels keep the image."""
    h, w = class_map.shape
    if bgr.shape[:2] != (h, w):
        bgr = cv2.resize(bgr, (w, h))
    out = cv2.addWeighted(bgr, 1.0 - alpha, palette[class_map], alpha, 0)
    keep = class_map == SEG_IGNORE
    out[keep] = bgr[keep]
    return out


def make_legend(class_pixels, names, palette, w, h, max_rows=20):
    """Legend panel: classes seen in the bag, most frequent first, with their share of pixels."""
    panel = np.full((h, w, 3), 30, np.uint8)
    scale = max(0.4, min(0.9, w / 500))
    cv2.putText(panel, "Classes", (12, int(35 * scale) + 5), FONT, scale, (255, 255, 255), 2, cv2.LINE_AA)
    total = class_pixels[:SEG_IGNORE].sum()
    if total == 0:
        cv2.putText(panel, "(no class map recorded)", (12, 80), FONT, 0.5 * scale + 0.1,
                    (200, 200, 200), 1, cv2.LINE_AA)
        return panel
    order = [int(c) for c in np.argsort(-class_pixels[:SEG_IGNORE]) if class_pixels[c] > 0][:max_rows]
    top = int(50 * scale) + 10
    row_h = max(16, min(32, (h - top) // max(1, len(order))))
    for r, c in enumerate(order):
        y = top + r * row_h
        cv2.rectangle(panel, (12, y), (12 + row_h - 6, y + row_h - 6),
                      tuple(int(v) for v in palette[c]), cv2.FILLED)
        name = names[c] if c < len(names) else f"class {c}"
        label = f"{c:2d} {name} {100.0 * class_pixels[c] / total:.1f}%"
        cv2.putText(panel, label, (20 + row_h, y + row_h - 9), FONT, 0.45 * scale + 0.1,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return panel


def stamp_ns(msg):
    st = msg.header.stamp
    return st.sec * 1_000_000_000 + st.nanosec


# --------------------------------------------------------------------------
# Synchronised video writer
# --------------------------------------------------------------------------
class SyncedVideo:
    """Writes frames on a fixed-FPS timeline; gaps are filled by repeating the last frame."""

    def __init__(self, path, fps):
        self.path, self.fps = path, fps
        self.writer, self.count, self.last = None, 0, None

    def due(self, frame_idx):
        return frame_idx >= self.count

    def write(self, img, frame_idx):
        if self.writer is None:
            h, w = img.shape[:2]
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (w, h))
        if self.last is not None and img.shape != self.last.shape:
            img = cv2.resize(img, (self.last.shape[1], self.last.shape[0]))
        while self.count < frame_idx:
            if self.last is not None:
                self.writer.write(self.last)
            self.count += 1
        self.writer.write(img)
        self.last = img
        self.count += 1

    def close(self):
        if self.writer is not None:
            self.writer.release()


# --------------------------------------------------------------------------
# Export
# --------------------------------------------------------------------------
def export(messages, args, has_overlay_topic, has_map_topic):
    """messages: iterable of (topic, deserialized_msg, bag_timestamp_ns)."""
    os.makedirs(args.output, exist_ok=True)
    names, palette = load_seg_labels(args.labels_file)
    raw_vid = SyncedVideo(os.path.join(args.output, "raw_camera_export.mp4"), args.fps)
    seg_vid = SyncedVideo(os.path.join(args.output, "segmentation_export.mp4"), args.fps)
    class_pixels = np.zeros(256, np.int64)
    recent_frames = OrderedDict()   # header stamp -> decoded camera frame (for class-map overlays)
    latest_frame = None
    start = None
    n_cam = n_map = n_overlay = n_exact = 0

    for topic, msg, t in messages:
        if start is None:
            start = t
        idx = int((t - start) / 1e9 * args.fps)

        if topic == args.camera_topic:
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            n_cam += 1
            latest_frame = img
            recent_frames[stamp_ns(msg)] = img
            while len(recent_frames) > 90:            # ~3-9 s of frames depending on rate
                recent_frames.popitem(last=False)
            if raw_vid.due(idx):
                raw_vid.write(img, idx)

        elif topic == args.seg_map_topic:
            cm = class_map_from_msg(msg)
            n_map += 1
            class_pixels += np.bincount(cm.ravel(), minlength=256)
            if not has_overlay_topic and seg_vid.due(idx):
                frame = recent_frames.get(stamp_ns(msg))
                if frame is not None:
                    n_exact += 1
                else:
                    frame = latest_frame
                if frame is not None:
                    seg_vid.write(seg_overlay(frame, cm, palette, args.alpha), idx)

        elif topic == args.seg_topic:
            n_overlay += 1
            if seg_vid.due(idx):
                img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    seg_vid.write(img, idx)

    raw_vid.close()
    seg_vid.close()
    print(f"\nCamera frames read:        {n_cam}  -> {raw_vid.count} video frames")
    print(f"Segmentation overlays read: {n_overlay}")
    print(f"Class maps read:            {n_map}"
          + (f" ({n_exact} matched to their exact camera frame)" if not has_overlay_topic and n_map else ""))
    print(f"Segmentation video:         {seg_vid.count} frames")

    if raw_vid.count and seg_vid.count:
        side_by_side(args.output, class_pixels, names, palette, args.fps)
    return class_pixels


def side_by_side(output_dir, class_pixels, names, palette, fps):
    """raw | segmentation | legend."""
    raw = cv2.VideoCapture(os.path.join(output_dir, "raw_camera_export.mp4"))
    seg = cv2.VideoCapture(os.path.join(output_dir, "segmentation_export.mp4"))
    w = int(raw.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(raw.get(cv2.CAP_PROP_FRAME_HEIGHT))
    legend = make_legend(class_pixels, names, palette, max(220, w // 2), h)
    out_path = os.path.join(output_dir, "side_by_side_export.mp4")
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), fps,
                             (2 * w + legend.shape[1], h))
    blank = np.zeros((h, w, 3), np.uint8)
    n = 0
    while True:
        ok_r, fr = raw.read()
        ok_s, fs = seg.read()
        if not ok_r and not ok_s:
            break
        fr = cv2.resize(fr, (w, h)) if ok_r else blank
        fs = cv2.resize(fs, (w, h)) if ok_s else blank
        writer.write(np.hstack([fr, fs, legend]))
        n += 1
    raw.release()
    seg.release()
    writer.release()
    print(f"Side-by-side video:         {n} frames -> {out_path}")


# --------------------------------------------------------------------------
# Bag reading
# --------------------------------------------------------------------------
def detect_storage_id(bag_path):
    if os.path.isfile(bag_path):
        return 'mcap' if bag_path.endswith('.mcap') else 'sqlite3'
    files = os.listdir(bag_path) if os.path.isdir(bag_path) else []
    if any(f.endswith('.db3') for f in files) and not any(f.endswith('.mcap') for f in files):
        return 'sqlite3'
    return 'mcap'


def bag_messages(reader, type_map):
    from rclpy.serialization import deserialize_message
    count = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        count += 1
        if count % 500 == 0:
            print(f"Processed {count} messages...")
        if topic in type_map:
            yield topic, deserialize_message(data, type_map[topic]), t


def main(args=None):
    parser = argparse.ArgumentParser(description="Export camera + semantic segmentation from a bag to video.")
    parser.add_argument("bag_path", help="Path to the bag directory (or .mcap file)")
    parser.add_argument("-o", "--output", default="seg_exports", help="Output folder (default: ./seg_exports)")
    parser.add_argument("--camera-topic", default="/video/compressed")
    parser.add_argument("--seg-topic", default="/video/segmentation_image/compressed",
                        help="Segmentation overlay topic (CompressedImage)")
    parser.add_argument("--seg-map-topic", default="/video/segmentation",
                        help="Segmentation class-map topic (mono8 Image)")
    parser.add_argument("--labels-file", default="",
                        help="Vistas config_v1.2.json for names/colours (same as the segmentation node)")
    parser.add_argument("--alpha", type=float, default=0.5,
                        help="Overlay strength when drawing from the class map (default 0.5)")
    parser.add_argument("--fps", type=float, default=10.0, help="Output video frame rate (default 10)")
    parsed = parser.parse_args(sys.argv[1:] if args is None else args)

    import rosbag2_py
    from sensor_msgs.msg import CompressedImage, Image

    reader = rosbag2_py.SequentialReader()
    storage_id = detect_storage_id(parsed.bag_path)
    reader.open(rosbag2_py.StorageOptions(uri=parsed.bag_path, storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    bag_topics = {t.name for t in reader.get_all_topics_and_types()}

    type_map = {}
    if parsed.camera_topic in bag_topics:
        type_map[parsed.camera_topic] = CompressedImage
    has_overlay = parsed.seg_topic in bag_topics
    has_map = parsed.seg_map_topic in bag_topics
    if has_overlay:
        type_map[parsed.seg_topic] = CompressedImage
    if has_map:
        type_map[parsed.seg_map_topic] = Image
    # Only read the topics we need (much faster on bags with radar etc.)
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(type_map)))

    print(f"Bag: {parsed.bag_path} ({storage_id})")
    print(f"Camera:       {parsed.camera_topic} {'found' if parsed.camera_topic in bag_topics else 'NOT FOUND'}")
    if has_overlay:
        print(f"Segmentation: overlay from {parsed.seg_topic}")
    elif has_map:
        print(f"Segmentation: drawn from class map {parsed.seg_map_topic}")
    else:
        print("Segmentation: no segmentation topics in this bag")
    if not has_map:
        print("              (class map not recorded: the legend will be empty)")
    print(f"Output:       {parsed.output}/")

    export(bag_messages(reader, type_map), parsed, has_overlay, has_map)
    print("\nDone.")


if __name__ == '__main__':
    main()