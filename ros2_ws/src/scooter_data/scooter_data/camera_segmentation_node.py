#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/ubuntu/237D/camera/cam_env/lib/python3.12/site-packages')

import json
import os
import threading
import time
from datetime import datetime
from functools import partial

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from scooter_data.semantic_segmenter import IGNORE, SemanticSegmenter

DECODE_FLAGS = {1: cv2.IMREAD_COLOR, 2: cv2.IMREAD_REDUCED_COLOR_2,
                4: cv2.IMREAD_REDUCED_COLOR_4, 8: cv2.IMREAD_REDUCED_COLOR_8}


class LatestSlot:
    """Hand-off between pipeline stages that keeps only the newest item (stale ones are
    dropped, like a depth-1 queue)."""

    def __init__(self):
        self._cv = threading.Condition()
        self._item = None
        self.dropped = 0

    def put(self, item):
        with self._cv:
            if self._item is not None:
                self.dropped += 1
            self._item = item
            self._cv.notify()

    def get(self, stop):
        with self._cv:
            while self._item is None and not stop.is_set():
                self._cv.wait(0.1)
            item, self._item = self._item, None
            return item


class CameraSegmentationNode(Node):
    """Semantic segmentation (BiSeNetFormer, Mapillary Vistas) on the Rubik Pi NPU.

    For each input topic  <topic>/compressed  it publishes:
      <topic>/segmentation                      sensor_msgs/Image mono8, pixel value = class id
                                                (255 = class not selected by class_ids)
      <topic>/segmentation_image/compressed     colour overlay for viewing (if output_image)
    and logs per-frame class coverage to a JSON file (like the YOLO node's detections).

    class_ids limits the output to chosen classes, e.g. -p class_ids:="55,19" or "Car,Person".

    With pipeline:=true (default) the work runs in three overlapping stages, so the NPU is
    busy while the next frame is decoded and the previous one is drawn/encoded/published:
      1. ROS callback   JPEG decode + resize to 640 (CPU)
      2. NPU thread     inference + class map
      3. output thread  upscale, publish class map, overlay + JPEG encode, JSON
    Throughput is then set by the slowest stage instead of the sum of all of them.
    """

    def __init__(self):
        super().__init__('camera_segmentation_node')

        # ---- parameters (override with --ros-args -p name:=value)
        self.declare_parameter('model_path', '/home/ubuntu/bisenetformer/models/model_opt_int8_htp.dlc')
        self.declare_parameter('runtime', 'dsp')            # dsp (NPU) | cpu | gpu
        self.declare_parameter('perf_profile', 'burst')     # burst | sustained | high | balanced | default
        self.declare_parameter('lib_path', '')              # default: libsnpe_infer.so next to this file
        self.declare_parameter('labels_file', '')           # optional Vistas config_v1.2.json
        self.declare_parameter('num_classes', 65)
        self.declare_parameter('topic_list', ['/video/compressed'])
        self.declare_parameter('output_image', True)
        self.declare_parameter('overlay_alpha', 0.5)
        self.declare_parameter('overlay_every', 1)          # publish the overlay every Nth frame
        self.declare_parameter('overlay_scale', 1.0)        # e.g. 0.5 = overlay at half size
        self.declare_parameter('jpeg_quality', 80)          # overlay JPEG quality
        self.declare_parameter('decode_scale', 1)           # 1|2|4|8: decode the camera JPEG reduced
        self.declare_parameter('pipeline', True)            # overlap decode / NPU / output stages
        self.declare_parameter('save_json', True)
        self.declare_parameter('json_dir', 'json_dir/')
        self.declare_parameter('min_class_fraction', 0.01)  # classes below this share are not logged
        self.declare_parameter('stats_every', 30)           # log timing every N frames
        self.declare_parameter('class_ids', '')             # e.g. "55,19" or "Car,Person"; empty = all

        p = lambda n: self.get_parameter(n).value
        self.output_image = p('output_image')
        self.alpha = float(p('overlay_alpha'))
        self.overlay_every = max(1, int(p('overlay_every')))
        self.overlay_scale = float(p('overlay_scale'))
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, int(p('jpeg_quality'))]
        decode_scale = int(p('decode_scale'))
        if decode_scale not in DECODE_FLAGS:
            raise ValueError("decode_scale must be 1, 2, 4 or 8")
        self.decode_flag = DECODE_FLAGS[decode_scale]
        self.pipeline = bool(p('pipeline'))
        self.min_fraction = float(p('min_class_fraction'))
        self.stats_every = int(p('stats_every'))
        self.topic_list = list(p('topic_list'))

        self.get_logger().info(f"Loading {p('model_path')} on {p('runtime')} ...")
        self.segmenter = SemanticSegmenter(
            p('model_path'), runtime=p('runtime'), perf=p('perf_profile'),
            num_classes=int(p('num_classes')), labels_file=p('labels_file'),
            lib_path=p('lib_path') or None)
        self.get_logger().info(f"Model ready (input {self.segmenter.model.input_shape}, "
                               f"outputs {[n for n, _ in self.segmenter.model.outputs]}, "
                               f"mode {self.segmenter.mode})")
        class_ids = self.segmenter.parse_classes(p('class_ids'))
        self.segmenter.set_class_filter(class_ids)
        if class_ids:
            self.get_logger().info("Outputting only: " + ", ".join(
                f"{c} ({self.segmenter.names[c]})" for c in class_ids)
                + f"; other pixels are {IGNORE} in the class map")
        else:
            self.get_logger().info("Outputting all classes")

        self.json_file = None
        if p('save_json'):
            os.makedirs(p('json_dir'), exist_ok=True)
            session = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.json_path = os.path.join(p('json_dir'), f'segmentation_{session}.json')
            self.json_file = open(self.json_path, 'w')
            self.json_file.write('[\n')
            self.first_entry = True
            self.get_logger().info(f"Saving per-frame class coverage to {self.json_path}")

        # Keep only the newest frame: if inference is slower than the camera, stale
        # frames are dropped instead of queueing up (the YOLO node queues 10).
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)

        self.bridge = CvBridge()
        self.sub_image, self.pub_classmap, self.pub_image = {}, {}, {}
        for topic in self.topic_list:
            topic_clean = topic.replace('/compressed', '')
            self.sub_image[topic] = self.create_subscription(
                CompressedImage, topic, partial(self.image_callback, topic=topic), qos)
            self.pub_classmap[topic] = self.create_publisher(
                Image, f"{topic_clean}/segmentation", 10)
            if self.output_image:
                self.pub_image[topic] = self.create_publisher(
                    CompressedImage, f"{topic_clean}/segmentation_image/compressed", 10)

        self.n_frames = 0
        self.frame_shape_logged = False
        self.stats_lock = threading.Lock()
        self._reset_stats()

        self.stop = threading.Event()
        self.to_npu, self.to_output = LatestSlot(), LatestSlot()
        self.threads = []
        if self.pipeline:
            for target, name in ((self._npu_loop, 'seg_npu'), (self._output_loop, 'seg_output')):
                t = threading.Thread(target=target, name=name, daemon=True)
                t.start()
                self.threads.append(t)
        self.get_logger().info(
            f"pipeline={self.pipeline}, decode_scale={decode_scale}, output_image="
            f"{self.output_image} (every {self.overlay_every}, scale {self.overlay_scale}, "
            f"quality {self.jpeg_params[1]})")

    # ------------------------------------------------------------------ stage 1
    def image_callback(self, msg, topic):
        t0 = time.perf_counter()
        image_in = cv2.imdecode(np.frombuffer(msg.data, np.uint8), self.decode_flag)
        if image_in is None:
            self.get_logger().warn(f"Could not decode image from {topic}")
            return
        t1 = time.perf_counter()
        x = self.segmenter.preprocess(image_in)
        t2 = time.perf_counter()
        if not self.frame_shape_logged:
            self.frame_shape_logged = True
            self.get_logger().info(f"Camera frames: {image_in.shape[1]}x{image_in.shape[0]} "
                                   f"(after decode_scale), JPEG {len(msg.data) / 1024:.0f} KB")
        frame = {'topic': topic, 'header': msg.header, 'image': image_in, 'x': x, 't_recv': t0,
                 'ms': {'decode': 1000 * (t1 - t0), 'pre': 1000 * (t2 - t1)}}
        if self.pipeline:
            self.to_npu.put(frame)
        else:
            self._infer(frame)
            self._output(frame)

    # ------------------------------------------------------------------ stage 2
    def _infer(self, f):
        t0 = time.perf_counter()
        f['cm_small'] = self.segmenter.infer_small(f.pop('x'))
        f['ms']['npu'] = 1000 * (time.perf_counter() - t0)

    def _npu_loop(self):
        while not self.stop.is_set():
            f = self.to_npu.get(self.stop)
            if f is None:
                continue
            try:
                self._infer(f)
            except Exception as e:  # keep the node alive; report and skip the frame
                self.get_logger().error(f"inference failed: {e}")
                continue
            self.to_output.put(f)

    # ------------------------------------------------------------------ stage 3
    def _output(self, f):
        topic, header, image_in, cm_small, ms = f['topic'], f['header'], f['image'], f['cm_small'], f['ms']

        t0 = time.perf_counter()
        class_map = self.segmenter.upscale(cm_small, image_in.shape)
        cm_msg = self.bridge.cv2_to_imgmsg(class_map, encoding='mono8')
        cm_msg.header = header
        self.pub_classmap[topic].publish(cm_msg)
        t1 = time.perf_counter()
        ms['classmap'] = 1000 * (t1 - t0)

        ms['overlay'] = ms['encode'] = 0.0
        if self.output_image and self.n_frames % self.overlay_every == 0:
            img = image_in
            if self.overlay_scale != 1.0:
                img = cv2.resize(img, None, fx=self.overlay_scale, fy=self.overlay_scale,
                                 interpolation=cv2.INTER_LINEAR)
            overlay = self.segmenter.overlay(img, cm_small, self.alpha)
            t2 = time.perf_counter()
            ok, jpg = cv2.imencode('.jpg', overlay, self.jpeg_params)
            if ok:
                img_msg = CompressedImage(format='jpeg', data=jpg.tobytes())
                img_msg.header = header
                self.pub_image[topic].publish(img_msg)
            t3 = time.perf_counter()
            ms['overlay'], ms['encode'] = 1000 * (t2 - t1), 1000 * (t3 - t2)

        t4 = time.perf_counter()
        if self.json_file:
            # coverage from the 640x640 map: same fractions as the upscaled map, far cheaper
            record = {
                'timestamp': header.stamp.sec + header.stamp.nanosec * 1e-9,
                'topic': topic,
                'inference_ms': round(ms['npu'], 2),
                'classes': [{'class_id': c, 'class_name': name, 'fraction': round(frac, 4)}
                            for c, name, frac in self.segmenter.coverage(cm_small, self.min_fraction)],
            }
            if not self.first_entry:
                self.json_file.write(',\n')
            self.json_file.write(json.dumps(record, indent=2))
            self.first_entry = False
        ms['json'] = 1000 * (time.perf_counter() - t4)
        ms['latency'] = 1000 * (time.perf_counter() - f['t_recv'])
        self._update_stats(ms)

    def _output_loop(self):
        while not self.stop.is_set():
            f = self.to_output.get(self.stop)
            if f is None:
                continue
            try:
                self._output(f)
            except Exception as e:
                self.get_logger().error(f"output stage failed: {e}")

    # ------------------------------------------------------------------ stats
    def _reset_stats(self):
        self.t_acc = {}
        self.t_window_start = time.perf_counter()

    def _update_stats(self, ms):
        with self.stats_lock:
            self.n_frames += 1
            for k, v in ms.items():
                self.t_acc[k] = self.t_acc.get(k, 0.0) + v
            if self.n_frames % self.stats_every:
                return
            n = self.stats_every
            elapsed = time.perf_counter() - self.t_window_start
            a = {k: v / n for k, v in self.t_acc.items()}
            drops = (f" | dropped: before NPU {self.to_npu.dropped}, before output "
                     f"{self.to_output.dropped}" if self.pipeline else "")
            self.get_logger().info(
                f"{n / elapsed:.1f} FPS | decode {a['decode']:.1f}, resize {a['pre']:.1f}, "
                f"NPU {a['npu']:.1f}, class map {a['classmap']:.1f}, overlay {a['overlay']:.1f}, "
                f"jpeg {a['encode']:.1f}, json {a['json']:.1f} ms | latency {a['latency']:.0f} ms"
                + drops)
            if self.json_file:
                self.json_file.flush()
            self._reset_stats()

    def destroy_node(self):
        self.stop.set()
        for t in self.threads:
            t.join(timeout=2.0)
        if self.json_file:
            self.json_file.write('\n]')
            self.json_file.close()
            self.get_logger().info(f"Class coverage saved to {self.json_path}")
        self.segmenter.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraSegmentationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()