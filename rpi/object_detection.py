"""
StellaRay — Object Detection
Edge inference using TensorFlow Lite on Raspberry Pi 4.
Designed as a plug-in detection_fn for vision_pipeline.py.

Supports:
    - TFLite model inference (quantized INT8 / FLOAT32)
    - COCO label map (80 classes) or custom label file
    - NMS (Non-Maximum Suppression) to filter overlapping boxes
    - GPS-based geolocation tagging of detections
    - Confidence threshold filtering
    - Detection logging to CSV

Model recommendations for RPi4:
    - EfficientDet-Lite0  (~200ms/frame)  — good balance
    - MobileNet SSD v2    (~100ms/frame)  — fastest
    - EfficientDet-Lite2  (~400ms/frame)  — most accurate

Download pretrained TFLite models:
    https://www.tensorflow.org/lite/examples/object_detection/overview
    https://coral.ai/models/object-detection/

Author : Inderpal Singh
Project: StellaRay — Autonomous Quadcopter Platform
Status : Active Development
"""

import numpy as np
import csv
import os
import time
from datetime import datetime

try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    # Fallback to full TF if tflite_runtime not installed
    from tensorflow.lite.python.interpreter import Interpreter


# -----------------------------
# CONFIG
# -----------------------------
DEFAULT_MODEL_PATH  = "models/detect.tflite"
DEFAULT_LABEL_PATH  = "models/labelmap.txt"
DEFAULT_CONF_THRESH = 0.5       # minimum confidence to report detection
DEFAULT_NMS_THRESH  = 0.4       # IoU threshold for Non-Maximum Suppression
LOG_DIR             = "logs"


# -----------------------------
# LABEL LOADER
# -----------------------------
def load_labels(label_path):
    """
    Loads class labels from a text file (one label per line).
    Handles COCO-style labelmap with optional index prefix.

    Returns: list of label strings, index = class ID
    """
    if not os.path.exists(label_path):
        raise FileNotFoundError(f"Label file not found: {label_path}")

    with open(label_path, 'r') as f:
        lines = [l.strip() for l in f.readlines()]

    # Strip numeric prefix if present (e.g. "1 person" → "person")
    labels = []
    for line in lines:
        parts = line.split(maxsplit=1)
        if len(parts) == 2 and parts[0].isdigit():
            labels.append(parts[1])
        else:
            labels.append(line)

    return labels


# -----------------------------
# NMS
# Filters overlapping bounding boxes by IoU threshold.
# -----------------------------
def non_maximum_suppression(boxes, scores, iou_threshold=DEFAULT_NMS_THRESH):
    """
    Args:
        boxes         : list of (x, y, w, h) tuples
        scores        : list of float confidence scores
        iou_threshold : overlap threshold (lower = stricter)

    Returns:
        List of indices to keep.
    """
    if len(boxes) == 0:
        return []

    boxes_arr = np.array(boxes, dtype=float)
    scores_arr = np.array(scores, dtype=float)

    x1 = boxes_arr[:, 0]
    y1 = boxes_arr[:, 1]
    x2 = boxes_arr[:, 0] + boxes_arr[:, 2]
    y2 = boxes_arr[:, 1] + boxes_arr[:, 3]
    areas = boxes_arr[:, 2] * boxes_arr[:, 3]

    order = scores_arr.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h

        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]

    return keep


# -----------------------------
# DETECTION LOGGER
# Appends detections to CSV with timestamp + GPS.
# -----------------------------
class DetectionLogger:
    def __init__(self, log_dir=LOG_DIR):
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(
            log_dir,
            f"detections_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        with open(self.log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "label", "confidence", "bbox_x", "bbox_y",
                             "bbox_w", "bbox_h", "lat", "lon", "alt"])
        print(f"Detection log: {self.log_path}")

    def log(self, detections, gps=None):
        if not detections:
            return
        ts = datetime.now().isoformat()
        lat = gps['lat'] if gps else ""
        lon = gps['lon'] if gps else ""
        alt = gps['alt'] if gps else ""

        with open(self.log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            for det in detections:
                x, y, w, h = det['bbox']
                writer.writerow([ts, det['label'], f"{det['conf']:.4f}",
                                 x, y, w, h, lat, lon, alt])


# -----------------------------
# OBJECT DETECTOR
# -----------------------------
class ObjectDetector:
    def __init__(
        self,
        model_path=DEFAULT_MODEL_PATH,
        label_path=DEFAULT_LABEL_PATH,
        conf_threshold=DEFAULT_CONF_THRESH,
        nms_threshold=DEFAULT_NMS_THRESH,
        num_threads=4
    ):
        """
        Args:
            model_path    : path to .tflite model file
            label_path    : path to label text file
            conf_threshold: minimum confidence to keep a detection
            nms_threshold : IoU overlap threshold for NMS
            num_threads   : number of CPU threads for inference (4 = RPi4 max)
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")

        self.conf_threshold = conf_threshold
        self.nms_threshold  = nms_threshold
        self.labels         = load_labels(label_path)
        self.logger         = DetectionLogger()

        # Load TFLite model
        self.interpreter = Interpreter(model_path=model_path, num_threads=num_threads)
        self.interpreter.allocate_tensors()

        self.input_details  = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Input shape expected by the model
        self.input_h = self.input_details[0]['shape'][1]
        self.input_w = self.input_details[0]['shape'][2]
        self.is_quantized = self.input_details[0]['dtype'] == np.uint8

        print(f"Model loaded: {model_path}")
        print(f"Input size: {self.input_w}x{self.input_h} | Quantized: {self.is_quantized}")
        print(f"Classes: {len(self.labels)} | Conf threshold: {self.conf_threshold}")

    def _preprocess_frame(self, frame):
        """Resize and format frame for model input."""
        import cv2
        resized = cv2.resize(frame, (self.input_w, self.input_h))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        input_data = np.expand_dims(rgb, axis=0)

        if not self.is_quantized:
            input_data = input_data.astype(np.float32) / 255.0
        else:
            input_data = input_data.astype(np.uint8)

        return input_data

    def _parse_outputs(self, frame_h, frame_w):
        """
        Parses TFLite output tensors.
        Handles both SSD-style (boxes, classes, scores, count)
        and single-output formats.

        Returns: list of (bbox, class_id, score) before filtering.
        """
        # Standard TFLite detection model output order:
        # [0] boxes      : [1, N, 4]  — normalized [ymin, xmin, ymax, xmax]
        # [1] class ids  : [1, N]
        # [2] scores     : [1, N]
        # [3] num detections: [1]
        try:
            boxes   = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
            classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
            scores  = self.interpreter.get_tensor(self.output_details[2]['index'])[0]
            count   = int(self.interpreter.get_tensor(self.output_details[3]['index'])[0])
        except IndexError:
            # Fallback for models with single output tensor
            raw = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
            boxes   = raw[:, :4]
            scores  = raw[:, 4]
            classes = raw[:, 5] if raw.shape[1] > 5 else np.zeros(len(raw))
            count   = len(raw)

        results = []
        for i in range(min(count, len(scores))):
            if scores[i] < self.conf_threshold:
                continue

            ymin, xmin, ymax, xmax = boxes[i]
            x = int(xmin * frame_w)
            y = int(ymin * frame_h)
            w = int((xmax - xmin) * frame_w)
            h = int((ymax - ymin) * frame_h)

            results.append(((x, y, w, h), int(classes[i]), float(scores[i])))

        return results

    def detect(self, frame, gps=None):
        """
        Run inference on a single BGR frame.

        Args:
            frame : BGR numpy array from OpenCV
            gps   : dict with lat/lon/alt (optional, for logging)

        Returns:
            List of dicts compatible with vision_pipeline.py detection_fn format:
            [{'label': str, 'bbox': (x, y, w, h), 'conf': float}]
        """
        h, w = frame.shape[:2]

        input_data = self._preprocess_frame(frame)
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)

        t0 = time.time()
        self.interpreter.invoke()
        inference_ms = (time.time() - t0) * 1000

        raw_results = self._parse_outputs(h, w)

        if not raw_results:
            return []

        # NMS per class
        detections = []
        class_ids = list(set(r[1] for r in raw_results))

        for cid in class_ids:
            class_results = [r for r in raw_results if r[1] == cid]
            bboxes = [r[0] for r in class_results]
            scores = [r[2] for r in class_results]

            keep = non_maximum_suppression(bboxes, scores, self.nms_threshold)

            for idx in keep:
                label = self.labels[cid] if cid < len(self.labels) else f"class_{cid}"
                detections.append({
                    'label': label,
                    'bbox':  bboxes[idx],
                    'conf':  scores[idx]
                })

        if detections:
            print(f"[{inference_ms:.1f}ms] Detected: "
                  + ", ".join(f"{d['label']} ({d['conf']:.2f})" for d in detections))
            self.logger.log(detections, gps=gps)

        return detections


# -----------------------------
# PLUG-IN HELPER
# Returns a detection_fn ready to pass into VisionPipeline.run()
# -----------------------------
def build_detection_fn(model_path=DEFAULT_MODEL_PATH,
                       label_path=DEFAULT_LABEL_PATH,
                       conf_threshold=DEFAULT_CONF_THRESH,
                       mavlink=None):
    """
    Convenience wrapper. Returns a callable detection_fn for VisionPipeline.

    Usage:
        from object_detection import build_detection_fn
        from vision_pipeline import VisionPipeline

        detect = build_detection_fn(model_path="models/detect.tflite",
                                    label_path="models/labelmap.txt")
        pipeline = VisionPipeline(camera_src=0, display=False)
        pipeline.run(detection_fn=detect)
    """
    detector = ObjectDetector(
        model_path=model_path,
        label_path=label_path,
        conf_threshold=conf_threshold
    )

    def detection_fn(frame):
        gps = mavlink.get_gps(timeout=0.05) if mavlink else None
        return detector.detect(frame, gps=gps)

    return detection_fn


# -----------------------------
# TEST SCRIPT
# -----------------------------
if __name__ == "__main__":
    import cv2
    from vision_pipeline import VisionPipeline

    # Build detection function
    detect = build_detection_fn(
        model_path="models/detect.tflite",
        label_path="models/labelmap.txt",
        conf_threshold=0.5
    )

    # Run full pipeline with detection
    pipeline = VisionPipeline(camera_src=0, mavlink=None, display=True)
    pipeline.run(detection_fn=detect)
