#!/usr/bin/env python3
"""
gst_segmentation.py - GoPro -> semantic segmentation on the NPU, in one GStreamer process.

    GoPro UDP (MPEG-TS) -> v4l2 hardware decode (DMA-buf, linear NV12) -> tee
        +-> qtimlvconverter (GPU: resize/RGB/normalize) -> qtimlsnpe / qtimlqnn (NPU) -> appsink "tensors"
        +-> qtivtransform (GPU copy into a CPU-mappable buffer)                        -> appsink "frames"
                                                       |
            Python: match class map to its frame by timestamp, blend in NV12, H.264 encode
            -> .mkv/.mp4 file and/or RTP/UDP live stream (+ optional JSON coverage log, ROS 2 topic)

Why it is built this way (Rubik Pi / QCS6490):
  * The decoder is followed directly by "video/x-raw,format=NV12". Without that caps filter the
    v4l2 decoder picks NV12_Q08C (Qualcomm UBWC compressed) in DMA mode, which only Qualcomm
    hardware blocks can read; videoconvert / numpy cannot. This is what Qualcomm's own sample
    pipelines do ("v4l2h264dec capture-io-mode=4 output-io-mode=4 ! video/x-raw,format=NV12").
  * No videoconvert/videoscale between the decoder and qtimlvconverter: those produce
    system-memory buffers, and qtimlvconverter's GPU engine needs DMA-buf (FD) memory.
  * qtivtransform in the frames branch advertises GstVideoMeta upstream, so the decoder can keep
    handing out zero-copy DMA buffers to the tee (a bare appsink would make it fall back to a copy).
  * qtimlvconverter letterboxes by default (image-disposition=top-left). We ask for "stretch";
    whatever disposition is in effect, the overlay maps the class map back to the frame correctly.

Two ways to draw the overlay:
  default        Python blends each class map into its NV12 frame (matched by timestamp) and
                 pushes it to an encoder. Flexible, costs CPU per frame.
  --hw-overlay   the IM SDK deeplab-argmax post-process module turns the class map into a BGRA
                 mask, qtivcomposer blends it on the GPU, v4l2h264enc encodes it, all inside
                 GStreamer with DMA buffers. No per-frame Python unless --json-dir / --ros.

        decode -> tee -+-> queue ---------------------------------------------> qtivcomposer -> encode
                       +-> qtimlvconverter -> qtimlsnpe -> tee -+-> deeplab-argmax mask --^
                                                                +-> appsink (only for --json-dir/--ros)

Typical use on the Pi:
  # live: record the overlay and stream it to the PC
  python3 gst_segmentation.py --record-overlay seg.mkv --stream-host 10.43.0.1

  # same, but overlay + encode in hardware (needs the deeplab-argmax module installed)
  python3 gst_segmentation.py --hw-overlay --stream-host 10.43.0.1 --mask-size 320
  # on the PC:
  gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
      ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false

  # only cars and people, also keep the original camera stream (no re-encode) and a coverage log
  python3 gst_segmentation.py --class-ids "Car,Person" --record-raw raw.ts --json-dir json_dir/

  # QNN engine instead of SNPE (model is a QNN context binary / .so, not a .dlc)
  python3 gst_segmentation.py --engine qnn --model model.bin

  # not sure which codec the camera sends? let decodebin pick (hardware decoder still used)
  python3 gst_segmentation.py --codec auto

  # test the plumbing without the NPU / Qualcomm plugins (fake class map from brightness)
  python3 gst_segmentation.py --fake-npu --source test.ts --record-overlay test.mkv

Needs: python3-gi (gir1.2-gstreamer-1.0, gir1.2-gst-plugins-base-1.0), numpy, opencv,
Qualcomm IM SDK GStreamer plugins (qtimlvconverter, qtimlsnpe / qtimlqnn, qtivtransform).
"""
import argparse
import json
import os
import re
import sys
import tempfile
import threading
import time
import traceback
import urllib.request
from collections import OrderedDict
from datetime import datetime

import cv2
import numpy as np

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import GLib, Gst, GstVideo  # noqa: E402

try:
    from scooter_data.semantic_segmenter import IGNORE, SemanticSegmenter
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from semantic_segmenter import IGNORE, SemanticSegmenter

DEFAULT_MODEL = '/home/ubuntu/bisenetformer/models/model_noeinsum_int8_htp.dlc'
ENCODERS = [  # tried in order by --encoder auto
    'v4l2h264enc',
    'qtic2venc',
    'x264enc tune=zerolatency speed-preset=ultrafast key-int-max=30',
    'openh264enc',
]
HW_ENCODERS = ('v4l2h264enc', 'qtic2venc')


# --------------------------------------------------------------------------- GStreamer introspection
def element_props(factory_name):
    """Property names of an installed element (installed plugin versions differ)."""
    el = Gst.ElementFactory.make(factory_name, None)
    if el is None:
        return set()
    return {p.name for p in el.list_properties()}


def enum_nicks(factory_name, prop):
    """(accepted values, default value) of an enum property; ([], None) if unknown."""
    el = Gst.ElementFactory.make(factory_name, None)
    spec = el.find_property(prop) if el is not None else None
    if spec is None:
        return [], None
    try:
        nicks = [v.value_nick for v in spec.enum_class.__enum_values__.values()]
        default = spec.default_value.value_nick
    except Exception:
        return [], None
    return nicks, default


def have(factory_name):
    return Gst.ElementFactory.find(factory_name) is not None


def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


_reported = set()


def streaming_callback(fn):
    """appsink callbacks run on GStreamer threads: a Python exception there would print a
    traceback per frame. Report each distinct error once, keep the pipeline running."""
    def wrapper(self, sink):
        try:
            return fn(self, sink)
        except Exception as e:
            key = (fn.__name__, type(e).__name__, str(e))
            if key not in _reported:
                _reported.add(key)
                log(f"{fn.__name__} failed (shown once, further identical errors are suppressed):\n"
                    + traceback.format_exc())
            return Gst.FlowReturn.OK
    wrapper.__name__ = fn.__name__
    return wrapper


# --------------------------------------------------------------------------- GoPro control
class GoPro:
    """HTTP control of the GoPro over the USB network link (Open GoPro API, same calls as the ROS node)."""

    def __init__(self, ip):
        self.base = f"http://{ip}:8080/gopro/camera"
        self.stop_event = threading.Event()

    def _get(self, path, timeout=3):
        try:
            urllib.request.urlopen(self.base + path, timeout=timeout).read()
            return True
        except Exception as e:
            log(f"GoPro {path}: {e}")
            return False

    def start(self):
        log("Enabling GoPro wired USB control and starting the preview stream ...")
        self._get("/control/wired_usb?p=1")
        time.sleep(1)
        self._get("/stream/stop", timeout=2)      # a stream left running from a previous run blocks start
        time.sleep(0.5)
        if not self._get("/stream/start"):
            log("GoPro did not accept /stream/start - check the USB network link and --gopro-ip")
        threading.Thread(target=self._keep_alive, daemon=True).start()

    def _keep_alive(self):
        while not self.stop_event.wait(2.5):
            self._get("/keep_alive", timeout=1)

    def stop(self):
        self.stop_event.set()
        self._get("/stream/stop", timeout=2)


# --------------------------------------------------------------------------- helpers
class LatestSlot:
    """Hand-off between threads that keeps only the newest item."""

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

    def get(self, stop, timeout=0.1):
        with self._cv:
            if self._item is None and not stop.is_set():
                self._cv.wait(timeout)
            item, self._item = self._item, None
            return item


class _TensorOnly:
    """Lets SemanticSegmenter be used for labels/palette/class filter without loading SNPE."""

    def __init__(self, size):
        self.outputs = [('class_map', size * size)]


def plane(raw, offset, stride, rows, cols):
    """Copy one image plane out of a (possibly padded) buffer."""
    need = offset + stride * (rows - 1) + cols
    if need > raw.size:
        raise ValueError(f"plane needs {need} bytes, buffer has {raw.size}")
    view = np.lib.stride_tricks.as_strided(raw[offset:], shape=(rows, cols), strides=(stride, 1))
    return np.ascontiguousarray(view)


def bgr_palette_to_yuv(palette_bgr):
    """BT.601 limited-range Y, U, V lookup tables (what H.264 / NV12 from the decoder use)."""
    palette_bgr = np.asarray(palette_bgr)
    lut = np.zeros((256, 3), np.float32)               # cv2.LUT needs exactly 256 entries
    lut[:min(256, len(palette_bgr))] = palette_bgr[:256]
    b, g, r = lut[:, 0], lut[:, 1], lut[:, 2]
    y = 16 + 0.257 * r + 0.504 * g + 0.098 * b
    u = 128 - 0.148 * r - 0.291 * g + 0.439 * b
    v = 128 + 0.439 * r - 0.368 * g - 0.071 * b
    to8 = lambda a: np.clip(np.rint(a), 0, 255).astype(np.uint8)
    return to8(y), to8(u), to8(v)


def model_roi(disposition, size, w, h):
    """Region (x, y, w, h) of the size x size model input that holds the camera image.

    Mirrors qtimlvconverter's gst_ml_video_converter_update_destination()."""
    if disposition in ('stretch', 'centre-crop', None):
        return 0, 0, size, size
    cw = ch = size
    if w * size > h * size:
        ch = size * h // w
    elif w * size < h * size:
        cw = size * w // h
    if disposition == 'centre':
        return (size - cw) // 2, (size - ch) // 2, cw, ch
    return 0, 0, cw, ch                                # top-left


def write_hw_labels(path, fmt, names, palette_bgr, selected):
    """Labels file for the deeplab-argmax post-process module, coloured like the software overlay.

    fmt 'json'      : qtimlpostprocess  -> [{"id": 0, "label": "...", "color": "0xRRGGBBAA"}, ...]
    fmt 'structure' : qtimlvsegmentation -> one (structure)"name,id=(uint)N,color=(uint)RGBA;" per line
    Classes left out by --class-ids get alpha 0 (transparent)."""
    palette_bgr = np.asarray(palette_bgr)
    entries = []
    for cid in range(len(palette_bgr)):
        b, g, r = (int(v) for v in palette_bgr[cid][:3])
        alpha = 0xFF if (selected is None or cid in selected) else 0x00
        rgba = (r << 24) | (g << 16) | (b << 8) | alpha
        name = str(names[cid]) if cid < len(names) else f'class{cid}'
        entries.append((cid, name, rgba))
    with open(path, 'w') as f:
        if fmt == 'json':
            json.dump([{'id': cid, 'label': name, 'color': f'0x{rgba:08X}'}
                       for cid, name, rgba in entries], f, indent=1)
        else:
            for cid, name, rgba in entries:
                # GstStructure names: letters first, no spaces ('-' is shown as a space)
                sname = re.sub(r'[^A-Za-z0-9_-]+', '-', name).strip('-') or f'class{cid}'
                if not sname[0].isalpha():
                    sname = 'c' + sname
                f.write(f'(structure)"{sname},id=(uint){cid},color=(uint){rgba};"\n')
    return path


TENSOR_TYPES = {'FLOAT32': np.float32, 'FLOAT16': np.float16, 'INT32': np.int32,
                'UINT32': np.uint32, 'INT16': np.int16, 'UINT16': np.uint16,
                'INT8': np.int8, 'UINT8': np.uint8, 'INT64': np.int64, 'UINT64': np.uint64}


def parse_tensor_caps(caps):
    """-> (numpy dtype, dims of the first tensor, fake) for neural-network/tensors caps
    (or GRAY8 video in --fake-npu)."""
    s = caps.get_structure(0)
    if s.get_name() == 'video/x-raw':                     # --fake-npu stand-in
        return np.uint8, (s.get_value('height'), s.get_value('width')), True
    text = caps.to_string()
    m = re.search(r'type=\(string\)"?([A-Z0-9]+)', text)
    dtype = TENSOR_TYPES.get(m.group(1) if m else 'FLOAT32', np.float32)
    # first tensor's dims; plugin versions write either "< < 1, 640, 640 > >"
    # or "< < (int)1, (int)640, (int)640 > >"
    m = re.search(r'dimensions=(?:\(int\))?\s*<\s*<([^>]*)>', text)
    dims = None
    if m:
        dims = tuple(int(v) for v in re.findall(r'\d+', m.group(1).replace('(int)', ''))) or None
    return dtype, dims, False


# --------------------------------------------------------------------------- main app
class GstSegmentation:
    def __init__(self, a):
        self.a = a
        self.seg = SemanticSegmenter(None, num_classes=a.num_classes, input_size=a.input_size,
                                     labels_file=a.labels_file, backend=_TensorOnly(a.input_size))
        ids = self.seg.parse_classes(a.class_ids)
        self.seg.set_class_filter(ids)
        log("Classes: " + (", ".join(f"{c} ({self.seg.names[c]})" for c in ids) if ids else "all"))
        self.lut_y, self.lut_u, self.lut_v = bgr_palette_to_yuv(self.seg.palette)
        self.class_lut = getattr(self.seg, '_lut', None)

        self.pipe = self.loop = self.worker = None
        self.stop = threading.Event()
        self.frames = OrderedDict()            # pts -> (nv12, arrival monotonic, arrival wall time)
        self.frames_lock = threading.Lock()
        self.to_output = LatestSlot()
        self.frame_info = {}                   # caps string -> (w, h, offsets, strides)
        self.tensor_info = {}                  # caps string -> (dtype, dims, fake)
        self.disposition = 'stretch'           # what qtimlvconverter actually does (set in infer_chain)
        self.hw = a.hw_overlay                 # overlay drawn by qtimlvsegmentation + qtivcomposer
        self.hw_info = {}                      # element names chosen for --hw-overlay
        self.frame_size = None                 # (w, h) of decoded frames, from the caps event
        self.want_tensors = not self.hw or bool(a.json_dir or a.ros)
        self.got_eos = False
        self.roi_cache = {}
        self.out_pipe = self.appsrc = None
        self.out_lock = threading.Lock()
        self.st_lock = threading.Lock()
        self._reset_stats()
        self.json_file = None
        if a.json_dir:
            os.makedirs(a.json_dir, exist_ok=True)
            self.json_path = os.path.join(a.json_dir, f"segmentation_{datetime.now():%Y%m%d_%H%M%S}.json")
            self.json_file = open(self.json_path, 'w')
            self.json_file.write('[\n')
            self.first_json = True
        self.ros = None
        if a.ros:
            self._init_ros()

    def _reset_stats(self):
        self.st = dict(frames=0, tensors=0, unmatched=0, t_npu=0.0, t_overlay=0.0, t_push=0.0, n=0)
        self.t_window = time.monotonic()

    def _count(self, key, inc=1):
        with self.st_lock:
            self.st[key] += inc

    # ---------------------------------------------------------------- pipeline
    def infer_chain(self):
        a = self.a
        s = a.input_size
        if a.fake_npu:
            self.disposition = 'stretch'
            return (f"videoconvert ! videoscale add-borders=false ! "
                    f"video/x-raw,format=GRAY8,width={s},height={s}")

        conv = "qtimlvconverter"
        nicks, default = enum_nicks('qtimlvconverter', 'image-disposition')
        if a.disposition in nicks:
            conv += f" image-disposition={a.disposition}"
            self.disposition = a.disposition
        else:
            self.disposition = default or 'top-left'
            log(f"qtimlvconverter has no image-disposition '{a.disposition}' "
                f"(has: {', '.join(nicks) or '?'}); using its default '{self.disposition}', "
                f"the overlay compensates for it")
        if a.converter_engine:
            engines, _ = enum_nicks('qtimlvconverter', 'engine')
            if engines and a.converter_engine not in engines:
                sys.exit(f"qtimlvconverter engine '{a.converter_engine}' not available "
                         f"(has: {', '.join(engines)})")
            conv += f" engine={a.converter_engine}"

        if a.engine == 'snpe':
            props = f'model="{a.model}"'
            delegates, _ = enum_nicks('qtimlsnpe', 'delegate')
            if delegates and a.delegate not in delegates:
                sys.exit(f"qtimlsnpe has no delegate '{a.delegate}' (has: {', '.join(delegates)})")
            props += f' delegate={a.delegate}'
            profiles, _ = enum_nicks('qtimlsnpe', 'performance-profile')
            if profiles:
                if a.perf_profile not in profiles:
                    fallback = 'high-performance' if 'high-performance' in profiles else profiles[0]
                    log(f"qtimlsnpe has no performance-profile '{a.perf_profile}' "
                        f"(has: {', '.join(profiles)}); using {fallback}")
                    a.perf_profile = fallback
                props += f' performance-profile={a.perf_profile}'
            # qtimlsnpe always offers FLOAT32 next to the model's native (quantized) output type,
            # so asking for FLOAT32 gets dequantized class ids.
            return f'{conv} ! qtimlsnpe {props} ! neural-network/tensors,type=FLOAT32'

        qprops = element_props('qtimlqnn')
        qnn = f'{conv} ! qtimlqnn model="{a.model}"'
        for name, value in (('backend', a.qnn_backend), ('system', a.qnn_system)):
            if value:
                qnn += f' {name}="{value}"'
        if a.qnn_device_id is not None and 'backend-device-id' in qprops:
            qnn += f' backend-device-id={a.qnn_device_id}'
        return qnn

    def decode_chain(self):
        """Source of decoded linear-NV12 frames in DMA-buf memory."""
        a = self.a
        if a.decoder != 'auto':
            dec = a.decoder
        elif a.codec == 'auto' or (a.source and os.path.isfile(a.source)
                                   and not a.source.endswith('.ts')):
            # decodebin picks the parser/decoder; _on_element_added puts v4l2 decoders in DMA mode
            dec = 'decodebin'
        else:
            # GoPro MPEG-TS: only the video stream, decoded in hardware, DMA-buf in and out
            # (exactly how Qualcomm's sample apps configure the decoder)
            dec = (f'tsdemux ! queue ! {a.codec}parse ! '
                   f'v4l2{a.codec}dec capture-io-mode=dmabuf output-io-mode=dmabuf')
        if a.fake_npu and dec.startswith('tsdemux'):
            dec = dec.split(' ! v4l2')[0] + (f' ! avdec_{a.codec}' if have(f'avdec_{a.codec}')
                                             else ' ! decodebin')
        if a.fake_npu:     # software decoders give I420; videoconvert is fine without the NPU path
            dec += ' ! videoconvert'
        # The caps filter is the important part: it forces linear NV12 out of the decoder
        # instead of NV12_Q08C (UBWC). Nothing else may sit between the decoder and the tee.
        return f'{dec} ! video/x-raw,format={a.decoder_format}'

    def frames_chain(self):
        """Branch that delivers CPU-readable NV12 frames for the overlay."""
        if not self.a.fake_npu and have('qtivtransform'):
            return 'qtivtransform ! video/x-raw,format=NV12 ! '
        if not self.a.fake_npu:
            log("qtivtransform not found: frames go straight to the appsink. If qtimlvconverter "
                "then reports 'does not have FD memory', install the IM SDK video plugins.")
        return ''

    def source_head(self):
        a = self.a
        if a.source and os.path.isfile(a.source):
            src = f'filesrc location="{a.source}"'
        elif a.source:
            src = a.source
        else:
            src = (f'udpsrc port={a.port} buffer-size=8388608 '
                   f'caps="video/mpegts,systemstream=(boolean)true,packetsize=(int)188"')
        head = src
        if a.record_raw:   # copy of the camera's own stream, no re-encode
            head += (f' ! tee name=rawtee rawtee. ! queue ! filesink location="{a.record_raw}" '
                     f'async=false rawtee.')
        return head

    def pace(self):
        return ' ! identity sync=true' if self.a.sync else ''   # play files at real-time speed

    def pipeline_string(self):
        if self.hw:
            return self.hw_pipeline_string()
        return (f'{self.source_head()} ! queue ! {self.decode_chain()}{self.pace()} ! tee name=t '
                f't. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! '
                f'{self.infer_chain()} ! '
                f'appsink name=tensors emit-signals=true sync=false max-buffers=1 drop=true '
                f't. ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! '
                f'{self.frames_chain()}'
                f'appsink name=frames emit-signals=true sync=false max-buffers=2 drop=true')

    # ---------------------------------------------------------------- --hw-overlay
    def _pick_postprocess(self):
        """qtimlvsegmentation (older IM SDK) or qtimlpostprocess (newer), with deeplab-argmax."""
        if 'post' in self.hw_info:
            return self.hw_info['post']
        a = self.a
        cands = (['qtimlvsegmentation', 'qtimlpostprocess'] if a.postprocess == 'auto'
                 else [a.postprocess])
        found = []
        for name in cands:
            modules, _ = enum_nicks(name, 'module')
            found.append(f"{name}: {', '.join(modules) if modules else 'not installed / no modules'}")
            if 'deeplab-argmax' in modules:
                fmt = 'json' if name == 'qtimlpostprocess' else 'structure'
                self.hw_info.update(post=name, labels_fmt=fmt)
                return name
        sys.exit("--hw-overlay needs the deeplab-argmax post-process module, found:\n  "
                 + "\n  ".join(found)
                 + "\nCheck: ls /usr/lib/*/gstreamer-1.0/ml/modules/  (or run without --hw-overlay)")

    def postprocess_chain(self):
        """Class-map tensor -> coloured BGRA mask."""
        a = self.a
        size = a.mask_size or a.input_size
        if a.fake_npu:   # stand-in: grey class map as a mask
            return (f'videoconvert ! videoscale ! video/x-raw,format=BGRA,width={size},height={size},'
                    f'pixel-aspect-ratio=1/1')
        post = self._pick_postprocess()
        labels = a.hw_labels_out or os.path.join(
            tempfile.gettempdir(), f"gst_seg_labels_{os.getpid()}."
            + ('json' if self.hw_info['labels_fmt'] == 'json' else 'txt'))
        write_hw_labels(labels, self.hw_info['labels_fmt'], self.seg.names, self.seg.palette,
                        set(self.seg.selected) if self.seg.selected is not None else None)
        self.hw_info['labels'] = labels
        log(f"HW overlay: {post} module=deeplab-argmax, labels {labels}, mask {size}x{size}")
        # the mask is drawn on the CPU inside the module: a smaller mask is cheaper
        return (f'{post} name=post module=deeplab-argmax labels="{labels}" ! '
                f'video/x-raw,format=BGRA,width={size},height={size}')

    def encoder_chain(self, hw_input):
        """'<format conversion> ! <encoder> ! h264parse ! tee name=o <branches>'."""
        a = self.a
        enc = self._encoder_with_bitrate(self._pick_encoder())
        name = enc.split()[0]
        if name in HW_ENCODERS:
            # hw encoders also accept NV12_Q08C; pin plain NV12 (the composer branch already does)
            to_enc = '' if hw_input else 'video/x-raw,format=NV12 ! '
            if hw_input and name == 'v4l2h264enc' and 'io-mode' not in enc:
                # composer output is DMA-buf: let the encoder import it without a copy
                # (what Qualcomm's gst-ip-camera sample does)
                enc += ' capture-io-mode=dmabuf output-io-mode=dmabuf-import'
        else:
            to_enc = 'videoconvert ! '
        branches = []
        if a.record_overlay:
            mux = 'mp4mux' if a.record_overlay.endswith('.mp4') else 'matroskamux'
            branches.append(f'o. ! queue ! {mux} ! filesink location="{a.record_overlay}"')
        if a.stream_host:
            # not leaky: dropping encoded H.264 frames corrupts the stream until the next keyframe
            branches.append(f'o. ! queue ! rtph264pay config-interval=1 pt=96 ! '
                            f'udpsink host={a.stream_host} port={a.stream_port} sync=false async=false')
        if not branches:
            branches.append('o. ! queue ! fakesink sync=false async=false')
        return name, (f'{to_enc}{enc} ! h264parse config-interval=-1 ! tee name=o '
                      + ' '.join(branches))

    def hw_pipeline_string(self):
        """Decode -> tee -> [frames -> composer sink_0] + [NPU -> deeplab-argmax mask -> composer
        sink_1] -> encode. No per-frame Python; the class map only reaches Python if
        --json-dir / --ros need it."""
        a = self.a
        mixer = 'compositor' if a.fake_npu else 'qtivcomposer'
        latency = int(a.hw_latency_ms * Gst.MSECOND)
        tensor_tap = ''
        if self.want_tensors:
            tensor_tap = (' nt. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 '
                          'max-size-time=0 ! appsink name=tensors emit-signals=true sync=false '
                          'max-buffers=1 drop=true')
        enc_name, enc = self.encoder_chain(hw_input=not a.fake_npu)
        self.hw_info['encoder'] = enc_name
        return (f'{self.source_head()} ! queue ! {self.decode_chain()}{self.pace()} ! tee name=t '
                # camera frames wait in this queue until their mask arrives (or latency expires)
                f't. ! queue max-size-buffers=0 max-size-bytes=0 max-size-time=2000000000 ! mix.sink_0 '
                f't. ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! '
                f'{self.infer_chain()} ! tee name=nt '
                f'nt. ! queue ! {self.postprocess_chain()} ! queue ! mix.sink_1'
                f'{tensor_tap} '
                f'{mixer} name=mix latency={latency} sink_1::alpha={a.alpha} ! '
                # (the stand-in compositor won't take a BGRA input when its output is pinned to NV12)
                f'{"videoconvert ! " if a.fake_npu else ""}video/x-raw,format=NV12 ! queue ! {enc}')

    def _hw_on_caps(self, pad, info):
        """Decoded frame size is known: pin both composer inputs to it, so the 640x640 mask is
        stretched over the frame and never makes the output bigger than the camera image."""
        ev = info.get_event()
        if ev.type != Gst.EventType.CAPS:
            return Gst.PadProbeReturn.OK
        st = ev.parse_caps().get_structure(0)
        ok_w, w = st.get_int('width')
        ok_h, h = st.get_int('height')
        if not (ok_w and ok_h) or self.frame_size == (w, h):
            return Gst.PadProbeReturn.OK
        self.frame_size = (w, h)
        sc = self.a.output_scale
        ow, oh = max(2, int(w * sc) // 2 * 2), max(2, int(h * sc) // 2 * 2)
        mix = self.pipe.get_by_name('mix')
        for pad_name in ('sink_0', 'sink_1'):
            sink = mix.get_static_pad(pad_name)
            if sink is None:
                continue
            if sink.find_property('dimensions') is not None:          # qtivcomposer
                Gst.util_set_object_arg(sink, 'position', '<0, 0>')
                Gst.util_set_object_arg(sink, 'dimensions', f'<{ow}, {oh}>')
            else:                                                     # compositor (--fake-npu)
                sink.set_property('width', ow)
                sink.set_property('height', oh)
        log(f"Camera frames: {w}x{h}; overlay output {ow}x{oh}, encoder {self.hw_info['encoder']}")
        return Gst.PadProbeReturn.OK

    def _hw_on_frame(self, pad, info):
        buf = info.get_buffer()
        with self.frames_lock:
            self.frames[buf.pts] = (None, time.monotonic(), time.time())
            while len(self.frames) > self.a.frame_buffer:
                self.frames.popitem(last=False)
        self._count('frames')
        return Gst.PadProbeReturn.OK

    def _hw_on_mask(self, pad, info):
        """A class map reached the post-process element: count it, measure latency."""
        buf = info.get_buffer()
        with self.frames_lock:
            f = self.frames.get(buf.pts)
        with self.st_lock:
            self.st['tensors'] += 1
            if f is not None:
                self.st['n'] += 1
                self.st['t_npu'] += time.monotonic() - f[1]
        return Gst.PadProbeReturn.OK

    def _hw_stats(self):
        with self.st_lock:
            st, now = self.st, time.monotonic()
            dt = max(now - self.t_window, 1e-6)
            n = max(st['n'], 1)
            self._reset_stats()
        log(f"segmentation {st['tensors'] / dt:.1f} FPS, camera {st['frames'] / dt:.1f} FPS | "
            f"frame->class map {1000 * st['t_npu'] / n:.0f} ms (convert+NPU+queue) | overlay+encode "
            f"on GPU/VPU")
        return GLib.SOURCE_CONTINUE

    def check_elements(self):
        a = self.a
        if a.fake_npu:
            return
        need = ['qtimlvconverter', 'qtimlsnpe' if a.engine == 'snpe' else 'qtimlqnn']
        if a.decoder == 'auto' and a.codec != 'auto' and not (a.source and os.path.isfile(a.source)
                                                               and not a.source.endswith('.ts')):
            need.append(f'v4l2{a.codec}dec')
        if self.hw:
            need.append('qtivcomposer')
            self._pick_postprocess()                     # exits with a clear message if none fits
        missing = [e for e in need if not have(e)]
        if missing:
            sys.exit(f"Missing GStreamer elements: {', '.join(missing)}.\n"
                     f"Check with: gst-inspect-1.0 {missing[0]}  (Qualcomm IM SDK plugins). If the plugin "
                     f"exists but fails to load, run: GST_DEBUG=2 gst-inspect-1.0 {missing[0]}")
        if not os.path.isfile(a.model):
            sys.exit(f"Model not found: {a.model}")
        if a.engine == 'qnn' and a.model.endswith('.dlc'):
            log("warning: qtimlqnn expects a QNN context binary (.bin) or model library (.so), "
                "not an SNPE .dlc")

    def start(self):
        self.check_elements()
        desc = self.pipeline_string()
        log("Pipeline:\n  " + desc.replace(' t. ', '\n  t. '))
        try:
            self.pipe = Gst.parse_launch(desc)
        except GLib.Error as e:
            sys.exit(f"Could not build the pipeline: {e.message}")
        self.pipe.connect('deep-element-added', self._on_element_added)
        frames, tensors = self.pipe.get_by_name('frames'), self.pipe.get_by_name('tensors')
        if frames is not None:
            frames.connect('new-sample', self._on_frame)
        if tensors is not None:
            tensors.connect('new-sample', self._on_tensor)
        if self.hw:
            tee_sink = self.pipe.get_by_name('t').get_static_pad('sink')
            tee_sink.add_probe(Gst.PadProbeType.EVENT_DOWNSTREAM, self._hw_on_caps)
            tee_sink.add_probe(Gst.PadProbeType.BUFFER, self._hw_on_frame)
            nt_sink = self.pipe.get_by_name('nt').get_static_pad('sink')
            nt_sink.add_probe(Gst.PadProbeType.BUFFER, self._hw_on_mask)
            GLib.timeout_add_seconds(max(1, self.a.hw_stats_seconds), self._hw_stats)
            outs = [self.a.record_overlay and f"recording {self.a.record_overlay}",
                    self.a.stream_host and f"streaming to {self.a.stream_host}:{self.a.stream_port}"]
            log("HW overlay output: " + (", ".join(filter(None, outs)) or "none (stats only)"))
        bus = self.pipe.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self._on_bus)
        if tensors is not None:
            self.worker = threading.Thread(target=self._output_loop, daemon=True)
            self.worker.start()
        if self.pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            sys.exit("Pipeline failed to start (run with GST_DEBUG=3 for details)")
        log("Waiting for video ...")

    def _on_element_added(self, _bin, _sub, element):
        f = element.get_factory()
        if not f:
            return
        name, klass = f.get_name(), f.get_metadata('klass') or ''
        # decoders created inside decodebin: same DMA-buf setup as the explicit chain
        if name.startswith('v4l2') and name.endswith('dec'):
            for prop in ('capture-io-mode', 'output-io-mode'):
                if element.find_property(prop) is not None:
                    Gst.util_set_object_arg(element, prop, 'dmabuf')
        if any(k in klass for k in ('Decoder', 'Demux')):
            log(f"Using {name} ({klass})")

    def _on_bus(self, _bus, msg):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            log(f"ERROR from {msg.src.get_name()}: {err.message}\n  {dbg}")
            if "Internal data stream error" in err.message:
                log("  (the element that actually failed is usually named in a WARNING/ERROR just "
                    "above, or rerun with GST_DEBUG=3 to see it)")
            # collect any follow-up errors (they often name the real culprit) before quitting
            GLib.timeout_add(500, self._quit)
        elif t == Gst.MessageType.WARNING:
            w, _ = msg.parse_warning()
            log(f"warning from {msg.src.get_name()}: {w.message}")
        elif t == Gst.MessageType.EOS and msg.src == self.pipe:
            self.got_eos = True
            log("End of stream")
            self._quit()

    def _quit(self, *_):
        if self.loop is not None:
            self.loop.quit()
        return GLib.SOURCE_REMOVE

    # ---------------------------------------------------------------- streaming-thread callbacks
    @streaming_callback
    def _on_frame(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        buf, caps = sample.get_buffer(), sample.get_caps()
        key = caps.to_string()
        info = self.frame_info.get(key)
        if info is None:
            vi = GstVideo.VideoInfo.new_from_caps(caps)
            if vi.finfo.format != GstVideo.VideoFormat.NV12:
                log(f"frames branch delivers {vi.finfo.name}, expected NV12 - check the caps filters")
                return Gst.FlowReturn.ERROR
            info = (vi.width, vi.height, list(vi.offset[:2]), list(vi.stride[:2]))
            self.frame_info[key] = info
            log(f"Camera frames: {vi.width}x{vi.height} NV12")
        w, h, offsets, strides = info
        meta = GstVideo.buffer_get_video_meta(buf)
        if meta is not None:                    # hardware buffers pad rows/planes; trust the meta
            offsets, strides = list(meta.offset[:2]), list(meta.stride[:2])
        ok, m = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        raw = None
        try:
            raw = np.frombuffer(m.data, np.uint8)       # view into mapped memory: copy before unmap
            nv12 = np.empty((h * 3 // 2, w), np.uint8)
            nv12[:h] = plane(raw, offsets[0], strides[0], h, w)
            nv12[h:] = plane(raw, offsets[1], strides[1], h // 2, w)
        except ValueError as e:
            log(f"bad frame layout: {e}")
            return Gst.FlowReturn.OK
        finally:
            raw = None
            buf.unmap(m)
        with self.frames_lock:
            self.frames[buf.pts] = (nv12, time.monotonic(), time.time())
            while len(self.frames) > self.a.frame_buffer:
                self.frames.popitem(last=False)
        self._count('frames')
        return Gst.FlowReturn.OK

    def _tensor_to_class_map(self, arr, dims, fake):
        """Model output (already copied out of the buffer) -> s x s uint8 class map."""
        s = self.a.input_size
        if fake:
            return (arr[: dims[0] * dims[1]].reshape(dims) // 4).astype(np.uint8)
        n = s * s
        nc = self.a.num_classes
        if arr.size >= n * nc and nc > 1 and arr.size < n * nc + n:
            # model without the argmax layer: per-class scores, NHWC or NCHW
            if dims and dims[-1] == nc:
                cm = arr[:n * nc].reshape(s, s, nc).argmax(axis=2)
            else:
                cm = arr[:n * nc].reshape(nc, s, s).argmax(axis=0)
            return cm.astype(np.uint8)
        if arr.size < n:
            raise ValueError(f"unexpected output size {arr.size} (expected a {s}x{s} class map, "
                             f"dims {dims})")
        cm = arr[:n].reshape(s, s)
        if arr.dtype.kind == 'f':
            cm = np.rint(cm)
        return np.clip(cm, 0, 255).astype(np.uint8)

    @streaming_callback
    def _on_tensor(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        buf, caps = sample.get_buffer(), sample.get_caps()
        key = caps.to_string()
        info = self.tensor_info.get(key)
        if info is None:
            log(f"Model output: {key}")
            info = parse_tensor_caps(caps)
            self.tensor_info[key] = info
            log(f"  -> {np.dtype(info[0]).name} tensor, dims {info[1]}")
        dtype, dims, fake = info
        ok, m = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            # m.data is a view of the mapped memory, so everything that reads it happens here
            arr = np.frombuffer(m.data, dtype)
            cm = self._tensor_to_class_map(arr, dims, fake)
        except ValueError as e:
            log(str(e))
            return Gst.FlowReturn.OK
        finally:
            arr = None
            buf.unmap(m)
        if self.class_lut is not None:
            cm = self.class_lut[cm]
        if not self.hw:
            self._count('tensors')
        self.to_output.put((buf.pts, cm, time.monotonic()))
        return Gst.FlowReturn.OK

    # ---------------------------------------------------------------- output thread
    def _match_frame(self, pts):
        with self.frames_lock:
            f = self.frames.get(pts)
            if f is None and self.frames and pts != Gst.CLOCK_TIME_NONE:
                near = min(self.frames, key=lambda p: abs(p - pts))   # shouldn't normally happen
                if abs(near - pts) < 100 * Gst.MSECOND:
                    f = self.frames[near]
            return f

    def _frame_class_map(self, cm, w, h):
        """Cut the part of the model input that holds the image (letterbox modes)."""
        key = (w, h)
        roi = self.roi_cache.get(key)
        if roi is None:
            roi = model_roi(self.disposition, cm.shape[0], w, h)
            self.roi_cache[key] = roi
            if roi[2:] != cm.shape[::-1]:
                log(f"converter disposition '{self.disposition}': image occupies {roi} of the "
                    f"{cm.shape[1]}x{cm.shape[0]} model input")
        x, y, rw, rh = roi
        return cm[y:y + rh, x:x + rw]

    def overlay_nv12(self, nv12, cm):
        """Blend class colours into an NV12 frame directly (no RGB conversion)."""
        a = self.a.alpha
        h = nv12.shape[0] * 2 // 3
        w = nv12.shape[1]
        y, uv = nv12[:h], nv12[h:].reshape(h // 2, w // 2, 2)
        cm_full = cv2.resize(cm, (w, h), interpolation=cv2.INTER_NEAREST)
        cm_half = cv2.resize(cm, (w // 2, h // 2), interpolation=cv2.INTER_NEAREST)
        y_col = cv2.LUT(cm_full, self.lut_y)
        uv_col = cv2.merge([cv2.LUT(cm_half, self.lut_u), cv2.LUT(cm_half, self.lut_v)])
        out_y = cv2.addWeighted(y, 1.0 - a, y_col, a, 0)
        out_uv = cv2.addWeighted(uv, 1.0 - a, uv_col, a, 0)
        if self.seg.selected is not None:                 # filtered-out pixels keep the camera
            cv2.copyTo(y, cv2.compare(cm_full, IGNORE, cv2.CMP_EQ), out_y)
            cv2.copyTo(uv, cv2.compare(cm_half, IGNORE, cv2.CMP_EQ), out_uv)
        return np.vstack([out_y, out_uv.reshape(h // 2, w)])

    def _scale_nv12(self, nv12, scale):
        h = nv12.shape[0] * 2 // 3
        w = nv12.shape[1]
        W, H = max(2, int(w * scale) // 2 * 2), max(2, int(h * scale) // 2 * 2)
        y = cv2.resize(nv12[:h], (W, H), interpolation=cv2.INTER_AREA)
        uv = cv2.resize(nv12[h:].reshape(h // 2, w // 2, 2), (W // 2, H // 2), interpolation=cv2.INTER_AREA)
        return np.vstack([y, uv.reshape(H // 2, W)])

    def _output_loop(self):
        a = self.a
        need_overlay = bool(a.record_overlay or a.stream_host)
        while not self.stop.is_set():
            item = self.to_output.get(self.stop)
            if item is None:
                continue
            pts, cm, t_tensor = item
            f = self._match_frame(pts)
            if f is None:
                self._count('unmatched')
                continue
            nv12, t_frame, wall = f
            try:
                if nv12 is None:     # --hw-overlay: the video is composed on the GPU, only publish
                    if self.frame_size:
                        cm = self._frame_class_map(cm, *self.frame_size)
                    self._publish(cm, pts, wall)
                    continue
                h, w = nv12.shape[0] * 2 // 3, nv12.shape[1]
                cm = self._frame_class_map(cm, w, h)
                t0 = time.monotonic()
                if need_overlay:
                    if a.output_scale != 1.0:
                        nv12 = self._scale_nv12(nv12, a.output_scale)
                    out = self.overlay_nv12(nv12, cm)
                t1 = time.monotonic()
                if need_overlay:
                    self._push(out)
                t2 = time.monotonic()
                self._publish(cm, pts, wall)
            except Exception as e:
                log(f"output failed: {e!r}")
                continue
            with self.st_lock:
                st = self.st
                st['n'] += 1
                st['t_npu'] += t_tensor - t_frame
                st['t_overlay'] += t1 - t0
                st['t_push'] += t2 - t1
                due = st['n'] >= a.stats_every
            if due:
                self._print_stats()

    def _print_stats(self):
        with self.st_lock:
            st, now = self.st, time.monotonic()
            dt = max(now - self.t_window, 1e-6)
            n = max(st['n'], 1)
            dropped, self.to_output.dropped = self.to_output.dropped, 0
            self._reset_stats()
        log(f"segmentation {st['tensors'] / dt:.1f} FPS, camera {st['frames'] / dt:.1f} FPS | "
            f"frame->class map {1000 * st['t_npu'] / n:.0f} ms (convert+NPU+queue), "
            f"overlay {1000 * st['t_overlay'] / n:.1f} ms, encode push {1000 * st['t_push'] / n:.1f} ms"
            + (f" | unmatched {st['unmatched']}" if st['unmatched'] else "")
            + (f" | output drops {dropped}" if dropped else ""))

    # ---------------------------------------------------------------- encode / record / stream
    def _pick_encoder(self):
        if self.a.encoder != 'auto':
            return self.a.encoder
        for cand in ENCODERS:
            if have(cand.split()[0]):
                return cand
        raise RuntimeError("No H.264 encoder found (tried "
                           + ", ".join(e.split()[0] for e in ENCODERS) + ")")

    def _encoder_with_bitrate(self, enc):
        name = enc.split()[0]
        kbps = self.a.bitrate
        if not kbps or 'bitrate' in enc:
            return enc
        props = element_props(name)
        if name.startswith('v4l2') and 'extra-controls' in props:
            return f'{enc} extra-controls="controls,video_bitrate={kbps * 1000}"'
        if 'target-bitrate' in props:                       # qtic2venc
            return f'{enc} target-bitrate={kbps * 1000}'
        if name == 'x264enc':
            return f'{enc} bitrate={kbps}'
        if 'bitrate' in props:                              # openh264enc (bits/s)
            return f'{enc} bitrate={kbps * 1000}'
        return enc

    def _build_output(self, w, h):
        a = self.a
        name, enc = self.encoder_chain(hw_input=False)
        desc = (f'appsrc name=src is-live=true do-timestamp=true format=time '
                f'caps="video/x-raw,format=NV12,width={w},height={h},framerate={a.output_fps}/1" ! '
                f'queue leaky=downstream max-size-buffers=4 max-size-bytes=0 max-size-time=0 ! {enc}')
        log(f"Output ({w}x{h}, encoder {name}): "
            + ", ".join(filter(None, [a.record_overlay and f"recording {a.record_overlay}",
                                      a.stream_host and f"streaming to {a.stream_host}:{a.stream_port}"])))
        pipe = Gst.parse_launch(desc)
        pipe.get_bus().add_signal_watch()
        pipe.get_bus().connect('message', self._on_out_bus)
        appsrc = pipe.get_by_name('src')
        if pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("output pipeline failed to start (try --encoder x264enc)")
        self.out_pipe, self.appsrc = pipe, appsrc

    def _on_out_bus(self, _bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            log(f"OUTPUT ERROR from {msg.src.get_name()}: {err.message}\n  {dbg}\n"
                f"  (try --encoder x264enc or --encoder openh264enc)")

    def _push(self, nv12):
        h, w = nv12.shape[0] * 2 // 3, nv12.shape[1]
        with self.out_lock:
            if self.stop.is_set():
                return
            if self.out_pipe is None:
                self._build_output(w, h)
            self.appsrc.emit('push-buffer', Gst.Buffer.new_wrapped(nv12.tobytes()))

    # ---------------------------------------------------------------- side outputs
    def _init_ros(self):
        import rclpy
        from sensor_msgs.msg import Image
        rclpy.init()
        self.ros = rclpy.create_node('gst_segmentation')
        self.ros_pub = self.ros.create_publisher(Image, self.a.ros_topic, 10)
        self.ros_image = Image
        log(f"Publishing class maps on {self.a.ros_topic}")

    def _publish(self, cm, pts, wall):
        if self.ros is not None:
            cm = np.ascontiguousarray(cm)
            msg = self.ros_image()
            msg.header.stamp.sec = int(wall)
            msg.header.stamp.nanosec = int((wall % 1) * 1e9)
            msg.header.frame_id = 'gopro'
            msg.height, msg.width = cm.shape
            msg.encoding, msg.step = 'mono8', cm.shape[1]
            msg.data = cm.tobytes()
            self.ros_pub.publish(msg)
        if self.json_file:
            rec = {'timestamp': wall,
                   'pts_ns': None if pts == Gst.CLOCK_TIME_NONE else int(pts),
                   'classes': [{'class_id': int(c), 'class_name': name, 'fraction': round(float(frac), 4)}
                               for c, name, frac in self.seg.coverage(cm, self.a.min_class_fraction)]}
            if not self.first_json:
                self.json_file.write(',\n')
            self.json_file.write(json.dumps(rec))
            self.first_json = False

    # ---------------------------------------------------------------- run / shutdown
    def run(self):
        self.loop = GLib.MainLoop()
        GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, 2, self._quit)    # SIGINT
        GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, 15, self._quit)   # SIGTERM
        try:
            self.start()
            self.loop.run()
        finally:
            self.shutdown()

    def shutdown(self):
        log("Stopping ...")
        if self.pipe is not None and self.hw and not self.got_eos:
            # the encoder/muxer live in this pipeline: push EOS through so .mp4/.mkv get finalized
            self.pipe.send_event(Gst.Event.new_eos())
            msg = self.pipe.get_bus().timed_pop_filtered(
                5 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
            if msg is None:
                log("EOS did not reach the end within 5 s; the recording may be truncated")
        if self.pipe is not None:
            self.pipe.set_state(Gst.State.NULL)       # stop producing before the worker exits
        self.stop.set()
        if self.worker is not None:
            self.worker.join(timeout=5)
        if self.st['n'] and not self.hw:
            self._print_stats()
        if self.hw and self.pipe is not None and self.a.record_overlay:
            log(f"Saved {self.a.record_overlay}")
        with self.out_lock:
            if self.out_pipe is not None:            # finish the recording file properly
                self.appsrc.emit('end-of-stream')
                self.out_pipe.get_bus().timed_pop_filtered(5 * Gst.SECOND,
                                                           Gst.MessageType.EOS | Gst.MessageType.ERROR)
                self.out_pipe.set_state(Gst.State.NULL)
                if self.a.record_overlay:
                    log(f"Saved {self.a.record_overlay}")
        if self.a.record_raw and self.pipe is not None:
            log(f"Saved {self.a.record_raw}")
        if self.json_file:
            self.json_file.write('\n]\n')
            self.json_file.close()
            self.json_file = None
            log(f"Saved {self.json_path}")
        if self.ros is not None:
            self.ros.destroy_node()
            self.ros = None


def main():
    p = argparse.ArgumentParser(description="GoPro -> NPU semantic segmentation with GStreamer",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    g = p.add_argument_group('camera')
    g.add_argument('--gopro-ip', default='172.28.114.51')
    g.add_argument('--no-gopro-control', action='store_true',
                   help="don't send the HTTP start/keep-alive/stop commands")
    g.add_argument('--port', type=int, default=8554, help='UDP port the GoPro streams to')
    g.add_argument('--source', default='',
                   help='a video file, or a GStreamer source chain replacing the GoPro UDP source')
    g.add_argument('--codec', choices=['h265', 'h264', 'auto'], default='h265',
                   help='video codec the GoPro sends; auto = let decodebin detect it')
    g.add_argument('--decoder', default='auto',
                   help='decode chain; auto = "tsdemux ! <codec>parse ! v4l2<codec>dec" (DMA-buf)')
    g.add_argument('--decoder-format', default='NV12',
                   help='format forced right after the decoder. NV12 = linear; never NV12_Q08C here')
    g.add_argument('--sync', action='store_true', help='play files at real-time speed')
    g = p.add_argument_group('model')
    g.add_argument('--model', default=DEFAULT_MODEL, help='class-map DLC (optimize_onnx.py --class_map)')
    g.add_argument('--engine', choices=['snpe', 'qnn'], default='snpe')
    g.add_argument('--delegate', default='dsp', help='qtimlsnpe delegate: dsp (HTP/NPU) | gpu | aip | none')
    g.add_argument('--perf-profile', default='burst', help='qtimlsnpe performance-profile')
    g.add_argument('--qnn-backend', default=os.path.expanduser('~/qairt/lib/libQnnHtp.so'))
    g.add_argument('--qnn-system', default=os.path.expanduser('~/qairt/lib/libQnnSystem.so'))
    g.add_argument('--qnn-device-id', type=int, default=None, help='qtimlqnn backend-device-id')
    g.add_argument('--converter-engine', default='',
                   help='qtimlvconverter engine: gles | fcv | c2d | none (gst-inspect-1.0 qtimlvconverter)')
    g.add_argument('--disposition', default='stretch',
                   help='qtimlvconverter image-disposition: stretch | top-left | centre')
    g.add_argument('--input-size', type=int, default=640)
    g.add_argument('--num-classes', type=int, default=65)
    g.add_argument('--labels-file', default='')
    g.add_argument('--class-ids', default='', help='e.g. "55,19" or "Car,Person"; empty = all')
    g.add_argument('--fake-npu', action='store_true', help='test the pipeline without Qualcomm plugins')
    g = p.add_argument_group('outputs')
    g.add_argument('--record-overlay', default='', help='overlay video file (.mkv or .mp4)')
    g.add_argument('--record-raw', default='', help='save the incoming camera stream as-is (e.g. raw.ts)')
    g.add_argument('--stream-host', default='', help='send the overlay as H.264 RTP to this IP')
    g.add_argument('--stream-port', type=int, default=5600)
    g.add_argument('--encoder', default='auto', help='H.264 encoder element (auto tries hardware first)')
    g.add_argument('--bitrate', type=int, default=4000, help='encoder bitrate in kbit/s (0 = encoder default)')
    g.add_argument('--output-fps', type=int, default=20, help='nominal rate given to the encoder')
    g.add_argument('--output-scale', type=float, default=1.0, help='e.g. 0.5 = overlay video at half size')
    g.add_argument('--alpha', type=float, default=0.5)
    g = p.add_argument_group('hardware overlay (--hw-overlay)')
    g.add_argument('--hw-overlay', action='store_true',
                   help='draw the overlay with the deeplab-argmax post-process module + qtivcomposer '
                        'and encode on the VPU: no per-frame Python/CPU image work')
    g.add_argument('--postprocess', choices=['auto', 'qtimlvsegmentation', 'qtimlpostprocess'],
                   default='auto', help='which IM SDK element provides deeplab-argmax '
                                        '(older releases: qtimlvsegmentation, newer: qtimlpostprocess)')
    g.add_argument('--mask-size', type=int, default=0,
                   help='resolution of the colour mask (0 = model size). The module draws it on the '
                        'CPU, so e.g. 320 is ~4x cheaper; the GPU stretches it over the frame')
    g.add_argument('--hw-latency-ms', type=float, default=150,
                   help='how long qtivcomposer waits for the mask of a frame before drawing the frame '
                        'with the previous mask (should cover convert+NPU+post-process time)')
    g.add_argument('--hw-labels-out', default='',
                   help='where to write the generated labels file (default: a temp file)')
    g.add_argument('--hw-stats-seconds', type=int, default=3)
    g.add_argument('--json-dir', default='', help='write per-frame class coverage JSON here')
    g.add_argument('--min-class-fraction', type=float, default=0.01)
    g.add_argument('--ros', action='store_true', help='also publish the class map to ROS 2')
    g.add_argument('--ros-topic', default='/video/segmentation')
    g = p.add_argument_group('misc')
    g.add_argument('--frame-buffer', type=int, default=30, help='decoded frames kept for matching')
    g.add_argument('--stats-every', type=int, default=30)
    a = p.parse_args()
    if 'Q08C' in a.decoder_format.upper():
        p.error("--decoder-format NV12_Q08C can't be read by the CPU overlay; use NV12")

    Gst.init(None)
    app = GstSegmentation(a)
    gopro = None
    if not a.source and not a.no_gopro_control:
        gopro = GoPro(a.gopro_ip)
        gopro.start()
    try:
        app.run()
    finally:
        if gopro:
            gopro.stop()


if __name__ == '__main__':
    main()