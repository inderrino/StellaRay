"""
StellaRay — Vision Pipeline
Real-time camera capture and frame processing using OpenCV.
Designed to run on Raspberry Pi 4 onboard StellaRay.

Supports:
    - USB camera or Raspberry Pi Camera (via V4L2)
    - Threaded capture for non-blocking frame reads
    - Preprocessing (resize, grayscale, denoise)
    - Motion detection
    - Frame saving with GPS-based filename tagging
    - Hook points for object detection model (see object_detection.py)

Author : Inderpal Singh
Project: StellaRay — Autonomous Quadcopter Platform
Status : Active Development
"""

import cv2
import time
import os
import threading
from datetime import datetime


# -----------------------------
# CONFIG
# -----------------------------
FRAME_WIDTH     = 640
FRAME_HEIGHT    = 480
FPS_TARGET      = 30
SAVE_DIR        = "captures"
MOTION_THRESH   = 500       # contour area threshold for motion detection


# -----------------------------
# THREADED CAMERA CAPTURE
# Reads frames in background so main loop never blocks on camera I/O.
# -----------------------------
class CameraStream:
    def __init__(self, src=0):
        """
        src: camera index (0 = first USB/Pi cam) or device path e.g. '/dev/video0'
        """
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS_TARGET)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera at source: {src}")

        self.frame = None
        self.lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._update, daemon=True)
        self._thread.start()
        print("Camera stream started.")
        return self

    def _update(self):
        while self._running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self.cap.release()
        print("Camera stream stopped.")


# -----------------------------
# PREPROCESSING
# -----------------------------
def preprocess(frame, grayscale=False, denoise=False):
    """
    Resize and optionally convert/denoise frame.

    Args:
        frame     : raw BGR frame from camera
        grayscale : convert to single channel
        denoise   : apply Gaussian blur to reduce noise

    Returns:
        Processed frame.
    """
    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

    if denoise:
        frame = cv2.GaussianBlur(frame, (5, 5), 0)

    if grayscale:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    return frame


# -----------------------------
# MOTION DETECTION
# Compares consecutive frames to detect movement in scene.
# -----------------------------
class MotionDetector:
    def __init__(self, threshold=MOTION_THRESH):
        self.prev_frame = None
        self.threshold = threshold

    def detect(self, frame):
        """
        Returns (motion_detected: bool, contours: list, diff_frame: ndarray)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        if self.prev_frame is None:
            self.prev_frame = gray
            return False, [], None

        diff = cv2.absdiff(self.prev_frame, gray)
        self.prev_frame = gray

        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        thresh = cv2.dilate(thresh, None, iterations=2)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        significant = [c for c in contours if cv2.contourArea(c) > self.threshold]

        return len(significant) > 0, significant, diff

    def draw_motion_boxes(self, frame, contours):
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "MOTION", (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return frame


# -----------------------------
# FRAME ANNOTATOR
# Draws overlays: timestamp, GPS coords, FPS, status text.
# -----------------------------
def annotate_frame(frame, fps=None, gps=None, label=None):
    """
    Draws HUD overlay on frame.

    Args:
        frame : BGR frame
        fps   : current FPS (float)
        gps   : dict with 'lat', 'lon', 'alt' keys (from MAVLink)
        label : optional status string (e.g. "OBJECT DETECTED")
    """
    h, w = frame.shape[:2]
    overlay_color = (0, 255, 180)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Timestamp
    ts = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
    cv2.putText(frame, ts, (10, 20), font, 0.5, overlay_color, 1)

    # FPS
    if fps is not None:
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 42), font, 0.5, overlay_color, 1)

    # GPS
    if gps:
        gps_text = f"LAT {gps['lat']:.6f}  LON {gps['lon']:.6f}  ALT {gps['alt']:.1f}m"
        cv2.putText(frame, gps_text, (10, h - 12), font, 0.45, overlay_color, 1)

    # Status label
    if label:
        cv2.putText(frame, label, (w // 2 - 80, 30), font, 0.7, (0, 0, 255), 2)

    return frame


# -----------------------------
# FRAME SAVER
# Saves frames to disk with timestamp + GPS tag in filename.
# -----------------------------
class FrameSaver:
    def __init__(self, save_dir=SAVE_DIR):
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)
        print(f"Saving captures to: {os.path.abspath(save_dir)}")

    def save(self, frame, gps=None, tag=""):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        if gps:
            geo = f"_lat{gps['lat']:.5f}_lon{gps['lon']:.5f}"
        else:
            geo = ""
        filename = f"{ts}{geo}{'_' + tag if tag else ''}.jpg"
        path = os.path.join(self.save_dir, filename)
        cv2.imwrite(path, frame)
        return path


# -----------------------------
# MAIN PIPELINE
# -----------------------------
class VisionPipeline:
    def __init__(self, camera_src=0, mavlink=None, display=False):
        """
        Args:
            camera_src : camera index or device path
            mavlink    : MAVLinkInterface instance (optional, for GPS overlay)
            display    : show live OpenCV window (False on headless RPi)
        """
        self.stream   = CameraStream(src=camera_src).start()
        self.motion   = MotionDetector()
        self.saver    = FrameSaver()
        self.mavlink  = mavlink
        self.display  = display

        self._running = False
        self._fps     = 0.0

    def run(self, detection_fn=None):
        """
        Main loop.

        Args:
            detection_fn : optional callable — takes a BGR frame,
                           returns list of dicts: [{'label': str, 'bbox': (x,y,w,h), 'conf': float}]
                           Plug in your TFLite / YOLO inference here.
        """
        print("Vision pipeline running. Press Q to quit.")
        self._running = True
        prev_time = time.time()

        while self._running:
            frame = self.stream.read()
            if frame is None:
                continue

            # FPS calculation
            now = time.time()
            self._fps = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now

            # Preprocess (keep BGR for detection, use denoised copy for motion)
            processed = preprocess(frame.copy(), denoise=True)

            # GPS (non-blocking — use last known if slow)
            gps = self.mavlink.get_gps(timeout=0.05) if self.mavlink else None

            # Motion detection
            motion_detected, contours, _ = self.motion.detect(processed)
            if motion_detected:
                frame = self.motion.draw_motion_boxes(frame, contours)

            # Object detection hook
            detections = []
            if detection_fn:
                detections = detection_fn(processed)
                frame = self._draw_detections(frame, detections)

            # Annotate HUD
            label = "OBJECT DETECTED" if detections else ("MOTION" if motion_detected else None)
            frame = annotate_frame(frame, fps=self._fps, gps=gps, label=label)

            # Auto-save on detection event
            if detections or motion_detected:
                saved_path = self.saver.save(frame, gps=gps, tag="event")
                print(f"Saved: {saved_path}")

            # Display
            if self.display:
                cv2.imshow("StellaRay — Vision", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.stop()

    def _draw_detections(self, frame, detections):
        """Draws bounding boxes from detection_fn output."""
        for det in detections:
            x, y, w, h = det['bbox']
            label = f"{det['label']} {det.get('conf', 0):.2f}"
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, label, (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        return frame

    def get_frame(self):
        """Returns latest raw frame — use this to pull frames from other modules."""
        return self.stream.read()

    def stop(self):
        self._running = False
        self.stream.stop()
        if self.display:
            cv2.destroyAllWindows()
        print("Vision pipeline stopped.")


# -----------------------------
# TEST SCRIPT
# -----------------------------
if __name__ == "__main__":
    # Standalone test — no MAVLink, with display window
    # On headless RPi set display=False
    pipeline = VisionPipeline(camera_src=0, mavlink=None, display=True)
    pipeline.run()
