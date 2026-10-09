#!/usr/bin/env python3
"""
SemanticSegmenter: runs the quantized BiSeNetFormer DLC on the Rubik Pi NPU through
SNPE (via libsnpe_infer.so) and returns a per-pixel class map for a BGR image.

Works with:
  * the class-map model from optimize_onnx.py --class_map  (output: [1, H, W] class ids)
  * the original two-output model (class scores + masks); the semantic combination is
    then done here in numpy (slower; prefer the class-map model)
"""
import ctypes
import json
import os
import time

import cv2
import numpy as np

# Mapillary Vistas v1.2 label order (65 classes; index 65 = "Unlabeled").
# Verify against the label order your model was trained with, or pass labels_file.
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

IGNORE = 255  # class-map value for pixels whose class is filtered out

RUNTIMES = {"cpu": 0, "gpu": 1, "dsp": 2, "npu": 2}
PERF = {"default": 0, "balanced": 1, "high": 2, "sustained": 3, "burst": 4}


class SnpeModel:
    """ctypes wrapper around libsnpe_infer.so (one loaded DLC)."""

    def __init__(self, dlc_path, runtime="dsp", perf="burst", output_names="", lib_path=None):
        if lib_path is None:
            lib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libsnpe_infer.so")
        if not os.path.isfile(lib_path):
            raise FileNotFoundError(f"{lib_path} not found - build it with snpe_infer/build.sh")
        if not os.path.isfile(dlc_path):
            raise FileNotFoundError(f"DLC not found: {dlc_path}")
        lib = ctypes.CDLL(lib_path)
        vp, cp, L = ctypes.c_void_p, ctypes.c_char_p, ctypes.c_long
        fp = ctypes.POINTER(ctypes.c_float)
        lib.si_open.restype = vp
        lib.si_open.argtypes = [cp, ctypes.c_int, ctypes.c_int, cp]
        lib.si_last_error.restype = cp
        lib.si_input_size.restype = L
        lib.si_input_size.argtypes = [vp]
        lib.si_input_dims.restype = ctypes.c_int
        lib.si_input_dims.argtypes = [vp, ctypes.POINTER(L), ctypes.c_int]
        lib.si_run.restype = ctypes.c_int
        lib.si_run.argtypes = [vp, fp, L]
        lib.si_num_outputs.restype = ctypes.c_int
        lib.si_num_outputs.argtypes = [vp]
        lib.si_output_name.restype = cp
        lib.si_output_name.argtypes = [vp, ctypes.c_int]
        lib.si_output_size.restype = L
        lib.si_output_size.argtypes = [vp, ctypes.c_int]
        lib.si_get_output.restype = ctypes.c_int
        lib.si_get_output.argtypes = [vp, ctypes.c_int, fp, L]
        lib.si_close.argtypes = [vp]
        self.lib = lib

        h = lib.si_open(dlc_path.encode(), RUNTIMES[runtime], PERF[perf], output_names.encode())
        if not h:
            raise RuntimeError(f"SNPE failed to load {dlc_path}: {lib.si_last_error().decode()}")
        self.h = h
        dims = (L * 8)()
        n = lib.si_input_dims(h, dims, 8)
        self.input_shape = tuple(dims[i] for i in range(n))
        self.input_size = lib.si_input_size(h)
        self.outputs = [(lib.si_output_name(h, i).decode(), lib.si_output_size(h, i))
                        for i in range(lib.si_num_outputs(h))]
        self._out_bufs = [np.empty(size, np.float32) for _, size in self.outputs]

    def run(self, x):
        x = np.ascontiguousarray(x, dtype=np.float32)
        ptr = x.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        if self.lib.si_run(self.h, ptr, x.size) != 0:
            raise RuntimeError(f"inference failed: {self.lib.si_last_error().decode()}")
        res = {}
        for i, ((name, size), buf) in enumerate(zip(self.outputs, self._out_bufs)):
            if self.lib.si_get_output(self.h, i, buf.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                                      size) != 0:
                raise RuntimeError(f"reading output {name}: {self.lib.si_last_error().decode()}")
            res[name] = buf
        return res

    def close(self):
        if getattr(self, "h", None):
            self.lib.si_close(self.h)
            self.h = None


class SemanticSegmenter:
    def __init__(self, dlc_path, runtime="dsp", perf="burst", num_classes=65,
                 input_size=640, labels_file="", lib_path=None, output_names="", backend=None):
        self.num_classes = num_classes
        self.input_size = input_size
        self.model = backend or SnpeModel(dlc_path, runtime, perf, output_names, lib_path)
        self.names, self.palette = self._load_labels(labels_file)
        self.last_timing = {}
        self.selected = None       # None = all classes
        self._lut = None
        n = input_size * input_size
        sizes = [size for _, size in self.model.outputs]
        if len(sizes) == 1 and sizes[0] == n:
            self.mode = "class_map"
        elif len(sizes) == 2:
            self.mode = "query"
        else:
            raise RuntimeError(f"unexpected model outputs {self.model.outputs}; expected a "
                               f"{input_size}x{input_size} class map or scores + masks")

    # ------------------------------------------------------------------ labels
    def _load_labels(self, labels_file):
        names = list(VISTAS_V12_NAMES[: self.num_classes])
        names += [f"class_{i}" for i in range(len(names), self.num_classes)]
        palette = np.random.RandomState(12345).randint(40, 256, (256, 3)).astype(np.uint8)
        if labels_file:
            with open(labels_file) as f:
                cfg = json.load(f)
            labels = cfg["labels"] if isinstance(cfg, dict) else cfg
            for i, lab in enumerate(labels[:256]):
                if isinstance(lab, dict):
                    if i < self.num_classes and "readable" in lab:
                        names[i] = lab["readable"]
                    elif i < self.num_classes and "name" in lab:
                        names[i] = lab["name"]
                    if "color" in lab:
                        palette[i] = lab["color"]
        palette_bgr = palette[:, ::-1].copy()  # OpenCV images are BGR
        return names, palette_bgr

    # ------------------------------------------------------------------ inference
    def preprocess(self, bgr):
        """BGR uint8 image -> 1 x S x S x 3 float32 RGB, values 0-255 (the model normalizes).
        The image is resized (stretched) to S x S, as in training/evaluation."""
        resized = cv2.resize(bgr, (self.input_size, self.input_size), interpolation=cv2.INTER_LINEAR)
        x = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return x.astype(np.float32)[None]

    def _postprocess(self, outs):
        s = self.input_size
        if self.mode == "class_map":
            cm = next(iter(outs.values()))
            return np.clip(np.rint(cm), 0, 255).astype(np.uint8).reshape(s, s)
        # two outputs: [Q, K] class probabilities and [h, w, Q] (NHWC) mask probabilities
        (_, a), (_, b) = outs.items()
        scores, masks = (a, b) if a.size < b.size else (b, a)
        k = self.num_classes + 1
        q = scores.size // k
        probs = scores.reshape(q, k)[:, : self.num_classes]
        hw = masks.size // q
        side = int(round(np.sqrt(hw)))
        m = masks.reshape(side, side, q)
        sem = m.reshape(-1, q) @ probs                  # [h*w, C]
        cm = sem.argmax(1).reshape(side, side).astype(np.uint8)
        return cv2.resize(cm, (s, s), interpolation=cv2.INTER_NEAREST)

    def infer_small(self, x):
        """Preprocessed input -> class map at model resolution (640 x 640), class filter applied.
        The returned array is a fresh copy, safe to use while the next frame runs."""
        cm = self._postprocess(self.model.run(x))
        if self._lut is not None:
            cm = self._lut[cm]
        return cm

    @staticmethod
    def upscale(cm, shape):
        """Nearest-neighbour resize of a class map to shape (h, w)."""
        h, w = shape[:2]
        if (h, w) == cm.shape:
            return cm
        return cv2.resize(cm, (w, h), interpolation=cv2.INTER_NEAREST)

    def segment(self, bgr):
        """Returns a uint8 class map with the same height/width as the input image.
        Pixels of classes outside the class filter (set_class_filter) are IGNORE (255)."""
        t0 = time.perf_counter()
        x = self.preprocess(bgr)
        t1 = time.perf_counter()
        outs = self.model.run(x)
        t2 = time.perf_counter()
        cm = self._postprocess(outs)
        if self._lut is not None:
            cm = self._lut[cm]
        cm = self.upscale(cm, bgr.shape)
        t3 = time.perf_counter()
        self.last_timing = {"pre_ms": 1000 * (t1 - t0), "infer_ms": 1000 * (t2 - t1),
                            "post_ms": 1000 * (t3 - t2)}
        return cm

    # ------------------------------------------------------------------ class filter
    def parse_classes(self, spec):
        """'55, 19' / '[55, 19]' / 'Car, Person' / mixed -> sorted list of class ids ([] = all)."""
        ids = []
        lower = {n.lower(): i for i, n in enumerate(self.names)}
        for tok in str(spec).strip().strip("[]").split(","):
            tok = tok.strip().strip("'\"")
            if not tok:
                continue
            if tok.lstrip("-").isdigit():
                i = int(tok)
            elif tok.lower() in lower:
                i = lower[tok.lower()]
            else:
                raise ValueError(f"unknown class '{tok}'")
            if not 0 <= i < self.num_classes:
                raise ValueError(f"class id {i} out of range 0-{self.num_classes - 1}")
            ids.append(i)
        return sorted(set(ids))

    def set_class_filter(self, class_ids):
        """Only keep these classes; everything else becomes IGNORE. Empty/None = keep all."""
        if not class_ids:
            self.selected, self._lut = None, None
            return
        self.selected = sorted(set(int(c) for c in class_ids))
        lut = np.full(256, IGNORE, np.uint8)
        lut[self.selected] = self.selected
        self._lut = lut

    # ------------------------------------------------------------------ helpers
    def colorize(self, class_map):
        out = self.palette[class_map]
        out[class_map == IGNORE] = 0
        return out

    def overlay(self, bgr, class_map, alpha=0.5):
        """Blend class colours onto the image; IGNORE pixels keep the original image.
        class_map may be any size (pass the small 640 x 640 map: the colours are looked up
        there and nearest-resized, which gives the same picture as colouring the full-size
        map but is much cheaper)."""
        h, w = bgr.shape[:2]
        color = self.palette[class_map]
        if color.shape[:2] != (h, w):
            color = cv2.resize(color, (w, h), interpolation=cv2.INTER_NEAREST)
        blended = cv2.addWeighted(bgr, 1.0 - alpha, color, alpha, 0)
        if self.selected is not None:          # IGNORE pixels only exist with a class filter
            mask = cv2.compare(class_map, IGNORE, cv2.CMP_EQ)
            if mask.shape[:2] != (h, w):
                mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
            cv2.copyTo(bgr, mask, blended)
        return blended

    def coverage(self, class_map, min_fraction=0.01):
        """[(class_id, name, fraction)] for classes covering at least min_fraction of the image
        (fractions are of the whole image; filtered-out pixels are not listed)."""
        counts = np.bincount(class_map.ravel(), minlength=256)
        total = class_map.size
        out = [(int(c), self.names[c] if c < len(self.names) else f"class_{c}", float(n / total))
               for c, n in enumerate(counts[: self.num_classes]) if n / total >= min_fraction]
        return sorted(out, key=lambda r: -r[2])

    def close(self):
        if hasattr(self.model, "close"):
            self.model.close()