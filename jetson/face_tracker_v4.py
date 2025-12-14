#!/usr/bin/env python3
"""
Face Tracker v4 - JETSON GPU POWERED
=====================================
Uses NVIDIA GPU for face detection:
- CUDA-accelerated DNN (SSD MobileNet or similar)
- TensorRT optimization if available
- Kalman filter for smooth, stable tracking
- CSI camera with hardware encoding

This should give STABLE face positions even when sitting still.
"""

import cv2
import numpy as np
import socket
import json
import time
import sys
import threading
import os
from flask import Flask, Response

app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()

# =============================================================================
# KALMAN FILTER - For smooth, stable face tracking
# =============================================================================

class FaceKalmanFilter:
    """
    Kalman filter for 2D face position.
    Predicts face position and smooths out detection noise.
    """
    def __init__(self):
        # State: [x, y, vx, vy]
        self.kf = cv2.KalmanFilter(4, 2)
        
        # Transition matrix (constant velocity model)
        self.kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        
        # Measurement matrix
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        
        # Process noise - lower = smoother but slower response
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.01
        
        # Measurement noise - higher = trust predictions more
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.1
        
        # Initial state
        self.kf.statePost = np.zeros((4, 1), dtype=np.float32)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)
        
        self.initialized = False
        self.frames_without_detection = 0
        self.max_predict_frames = 10  # Max frames to predict without detection
    
    def update(self, detected, x=0, y=0):
        """
        Update filter with detection (or lack thereof).
        Returns smoothed (x, y) position.
        """
        if detected:
            if not self.initialized:
                # First detection - initialize state
                self.kf.statePost = np.array([[x], [y], [0], [0]], dtype=np.float32)
                self.initialized = True
                self.frames_without_detection = 0
                return x, y
            
            # Predict then correct
            self.kf.predict()
            measurement = np.array([[x], [y]], dtype=np.float32)
            self.kf.correct(measurement)
            self.frames_without_detection = 0
            
        else:
            if self.initialized and self.frames_without_detection < self.max_predict_frames:
                # No detection - just predict
                self.kf.predict()
                self.frames_without_detection += 1
            else:
                # Lost face too long - reset
                self.initialized = False
                return None, None
        
        # Return smoothed position
        state = self.kf.statePost
        return float(state[0]), float(state[1])
    
    def reset(self):
        self.initialized = False
        self.frames_without_detection = 0

    def get_velocity_px_per_frame(self):
        """Return (vx, vy) in pixels per frame, if initialized."""
        if not self.initialized:
            return 0.0, 0.0
        state = self.kf.statePost
        return float(state[2]), float(state[3])

    def get_stability(self):
        """A simple 0..1 stability score based on prediction-only frames."""
        if not self.initialized:
            return 0.0
        if self.max_predict_frames <= 0:
            return 1.0
        return max(0.0, min(1.0, 1.0 - (self.frames_without_detection / float(self.max_predict_frames))))


# =============================================================================
# GPU FACE DETECTOR
# =============================================================================

class GPUFaceDetector:
    """
    GPU-accelerated face detection using OpenCV DNN with CUDA backend.
    """
    def __init__(self):
        self.net = None
        self.use_gpu = False
        self.confidence_threshold = 0.5
        
        # Try to load DNN model
        self._load_model()
    
    def _load_model(self):
        """Load face detection model with GPU support"""
        
        # Paths to try for the model
        model_paths = [
            # OpenCV's face detector (SSD)
            ("/usr/share/opencv4/haarcascades/res10_300x300_ssd_iter_140000.caffemodel",
             "/usr/share/opencv4/haarcascades/deploy.prototxt"),
            # Common locations
            ("/home/acp/models/res10_300x300_ssd_iter_140000.caffemodel",
             "/home/acp/models/deploy.prototxt"),
            ("/opt/models/res10_300x300_ssd_iter_140000.caffemodel",
             "/opt/models/deploy.prototxt"),
        ]
        
        # Download model if not present
        model_dir = "/home/acp/optimus/models"
        os.makedirs(model_dir, exist_ok=True)
        
        caffemodel = f"{model_dir}/res10_300x300_ssd_iter_140000.caffemodel"
        prototxt = f"{model_dir}/deploy.prototxt"
        
        if not os.path.exists(caffemodel):
            print("Downloading face detection model...")
            import urllib.request
            try:
                urllib.request.urlretrieve(
                    "https://raw.githubusercontent.com/opencv/opencv_3rdparty/dnn_samples_face_detector_20170830/res10_300x300_ssd_iter_140000.caffemodel",
                    caffemodel
                )
                urllib.request.urlretrieve(
                    "https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt",
                    prototxt
                )
                print("Model downloaded!")
            except Exception as e:
                print(f"Failed to download model: {e}")
                print("Falling back to Haar cascade")
                self.net = None
                return
        
        if os.path.exists(caffemodel) and os.path.exists(prototxt):
            try:
                self.net = cv2.dnn.readNetFromCaffe(prototxt, caffemodel)
                
                # Try to use CUDA
                if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                    self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                    self.use_gpu = True
                    print("✓ Using CUDA GPU for face detection")
                else:
                    self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                    print("⚠ CUDA not available, using CPU")
                
                return
            except Exception as e:
                print(f"Failed to load DNN model: {e}")
        
        print("Using Haar cascade fallback")
        self.net = None
    
    def detect(self, frame):
        """
        Detect faces in frame.
        Returns list of (x, y, w, h, confidence) bounding boxes.
        """
        if self.net is None:
            return self._detect_haar(frame)
        
        h, w = frame.shape[:2]
        
        # Prepare blob for DNN
        blob = cv2.dnn.blobFromImage(
            cv2.resize(frame, (300, 300)), 
            1.0, (300, 300), 
            (104.0, 177.0, 123.0)
        )
        
        self.net.setInput(blob)
        detections = self.net.forward()
        
        faces = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            
            if confidence > self.confidence_threshold:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                x1, y1, x2, y2 = box.astype(int)
                
                # Convert to (x, y, w, h, conf)
                faces.append((x1, y1, x2 - x1, y2 - y1, float(confidence)))
        
        return faces
    
    def _detect_haar(self, frame):
        """Fallback Haar cascade detection"""
        # Try multiple paths for Haar cascade
        cascade_paths = [
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        ]
        
        if not hasattr(self, 'haar_cascade'):
            for path in cascade_paths:
                if os.path.exists(path):
                    self.haar_cascade = cv2.CascadeClassifier(path)
                    print(f"Loaded Haar cascade: {path}")
                    break
            else:
                print("ERROR: Could not find Haar cascade file!")
                return []
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.haar_cascade.detectMultiScale(gray, 1.1, 5, minSize=(60, 60))
        # Haar does not provide confidence; use a conservative fixed value.
        return [(int(x), int(y), int(w), int(h), 0.6) for (x, y, w, h) in faces]


# =============================================================================
# MAIN TRACKER
# =============================================================================

class JetsonFaceTracker:
    def __init__(self, rpi_ip="192.168.1.136", udp_port=5555):
        self.rpi_ip = rpi_ip
        self.udp_port = udp_port
        
        # Camera settings
        self.frame_width = 640
        self.frame_height = 480
        self.cap = None
        
        # Detection
        self.detector = GPUFaceDetector()
        
        # Kalman filter for stability
        self.kalman = FaceKalmanFilter()
        
        # UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # State
        self.running = False
        self.fps = 0
        self.last_face_box = None
        
        # Smoothed output
        self.smooth_x = 0
        self.smooth_y = 0
        self.output_alpha = 0.5  # Output smoothing
    
    def _create_gstreamer_pipeline(self, sensor_id=0):
        """Create GStreamer pipeline for CSI camera with GPU processing"""
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
            f"nvvidconv flip-method=0 ! "
            f"video/x-raw, width={self.frame_width}, height={self.frame_height}, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! "
            f"appsink drop=1 max-buffers=1"
        )
    
    def _open_camera(self):
        """Open CSI or USB camera"""
        # Try CSI camera first
        try:
            pipeline = self._create_gstreamer_pipeline(0)
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                print(f"✓ CSI camera opened ({self.frame_width}x{self.frame_height})")
                return True
        except Exception as e:
            print(f"CSI camera failed: {e}")
        
        # Fallback to USB
        for i in range(3):
            self.cap = cv2.VideoCapture(i)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                print(f"✓ USB camera {i} opened")
                return True
        
        return False
    
    def _send_position(
        self,
        x_offset,
        y_offset,
        *,
        face_detected=True,
        confidence=1.0,
        stability=1.0,
        box_size=0.01,
        x_velocity=0.0,
        y_velocity=0.0,
        timestamp=None,
    ):
        """Send face position to RPi"""
        try:
            if timestamp is None:
                timestamp = time.time()
            msg = {
                "type": "face_position",
                # Legacy keys (kept for backward compatibility)
                "face_detected": bool(face_detected),
                "offset_x": round(float(x_offset), 4),
                "offset_y": round(float(y_offset), 4),
                "x_offset": round(float(x_offset), 4),
                "y_offset": round(float(y_offset), 4),

                # New canonical keys used by the decoupled RPi tracker
                "detected": bool(face_detected),
                "x": round(float(x_offset), 4),
                "y": round(float(y_offset), 4),
                "confidence": round(float(confidence), 3),
                "stability": round(float(stability), 3),
                "box_size": round(float(box_size), 4),
                "x_velocity": round(float(x_velocity), 4),
                "y_velocity": round(float(y_velocity), 4),
                "timestamp": float(timestamp),
            }
            self.socket.sendto(json.dumps(msg).encode(), (self.rpi_ip, self.udp_port))
        except Exception as e:
            print(f"UDP send error: {e}")
    
    def _process_frame(self, frame):
        """Process frame and detect face"""
        global latest_frame
        
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        
        # Detect faces
        faces = self.detector.detect(frame)
        
        face_detected = len(faces) > 0
        face_cx, face_cy = None, None
        
        conf = 0.0
        box_size = 0.01

        if face_detected:
            # Get largest face
            largest = max(faces, key=lambda f: f[2] * f[3])
            x, y, fw, fh, conf = largest
            
            face_cx = x + fw // 2
            face_cy = y + fh // 2
            
            self.last_face_box = largest

            # Normalized box size proxy (0..1)
            try:
                box_size = max(0.0, min(1.0, (fw * fh) / float(w * h)))
            except Exception:
                box_size = 0.01
        
        # Update Kalman filter
        kx, ky = self.kalman.update(face_detected, face_cx, face_cy)
        
        if kx is not None:
            # Kalman gave us a position (detected or predicted)
            # Convert to normalized offset (-1 to +1)
            raw_x = (kx - cx) / cx
            raw_y = (ky - cy) / cy
            
            # Additional output smoothing
            self.smooth_x = self.smooth_x * (1 - self.output_alpha) + raw_x * self.output_alpha
            self.smooth_y = self.smooth_y * (1 - self.output_alpha) + raw_y * self.output_alpha
            
            # Clamp to valid range
            self.smooth_x = max(-1, min(1, self.smooth_x))
            self.smooth_y = max(-1, min(1, self.smooth_y))

            # Velocity estimate from Kalman (px/frame -> normalized units/sec)
            vx_px_pf, vy_px_pf = self.kalman.get_velocity_px_per_frame()
            # Convert using FPS and frame half-size
            fps = max(1.0, float(self.fps) if self.fps else 30.0)
            x_vel = (vx_px_pf * fps) / float(cx)
            y_vel = (vy_px_pf * fps) / float(cy)
            # Clamp to keep packets sane
            x_vel = max(-5.0, min(5.0, x_vel))
            y_vel = max(-5.0, min(5.0, y_vel))

            stability = self.kalman.get_stability()

            # Confidence is best-effort: DNN gives it; Haar is a fixed proxy.
            conf_out = max(0.0, min(1.0, float(conf)))
            
            # Send to RPi
            self._send_position(
                self.smooth_x,
                self.smooth_y,
                face_detected=True,
                confidence=conf_out,
                stability=stability,
                box_size=box_size,
                x_velocity=x_vel,
                y_velocity=y_vel,
                timestamp=time.time(),
            )
            
            # Draw on frame
            if self.last_face_box is not None:
                if len(self.last_face_box) >= 4:
                    x, y, fw, fh = self.last_face_box[:4]
                cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
            
            # Draw Kalman predicted center
            cv2.circle(frame, (int(kx), int(ky)), 8, (0, 255, 255), -1)
            cv2.line(frame, (cx, cy), (int(kx), int(ky)), (255, 0, 0), 2)
            
            status = "TRACKING"
            color = (0, 255, 0)
        else:
            # No face
            self.smooth_x *= 0.9  # Decay smoothly
            self.smooth_y *= 0.9
            status = "SEARCHING"
            color = (0, 0, 255)

            # Send "no face" updates occasionally so the RPi can react.
            self._send_position(
                self.smooth_x,
                self.smooth_y,
                face_detected=False,
                confidence=0.0,
                stability=0.0,
                box_size=0.01,
                x_velocity=0.0,
                y_velocity=0.0,
                timestamp=time.time(),
            )
        
        # Draw crosshair at center
        cv2.line(frame, (cx - 30, cy), (cx + 30, cy), (0, 255, 0), 2)
        cv2.line(frame, (cx, cy - 30), (cx, cy + 30), (0, 255, 0), 2)
        
        # Draw status
        cv2.putText(frame, f"{status} | FPS:{self.fps:.0f} | GPU:{self.detector.use_gpu}",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(frame, f"Output: X:{self.smooth_x:+.3f} Y:{self.smooth_y:+.3f}",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        with frame_lock:
            latest_frame = frame.copy()
    
    def start(self):
        """Start tracking"""
        if not self._open_camera():
            print("ERROR: Could not open camera!")
            return
        
        self.running = True
        frame_count = 0
        fps_start = time.time()
        
        print("Face tracking started...")
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            self._process_frame(frame)
            
            # Calculate FPS
            frame_count += 1
            if frame_count >= 30:
                elapsed = time.time() - fps_start
                self.fps = frame_count / elapsed
                frame_count = 0
                fps_start = time.time()
        
        self.cap.release()
    
    def stop(self):
        self.running = False


# =============================================================================
# WEB SERVER
# =============================================================================

@app.route('/')
def index():
    return '''
    <html>
    <head><title>Jetson Face Tracker v4</title></head>
    <body style="background:#111;color:#fff;text-align:center;font-family:sans-serif;">
        <h1>Jetson GPU Face Tracker</h1>
        <img src="/video" style="max-width:100%;border:2px solid #0f0;">
        <p>Kalman-filtered, GPU-accelerated face detection</p>
    </body>
    </html>
    '''

@app.route('/video')
def video():
    def generate():
        while True:
            with frame_lock:
                if latest_frame is not None:
                    _, jpeg = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.033)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("JETSON FACE TRACKER v4 - GPU POWERED")
    print("=" * 60)
    
    # Check CUDA
    cuda_devices = cv2.cuda.getCudaEnabledDeviceCount()
    print(f"CUDA devices: {cuda_devices}")
    
    if cuda_devices > 0:
        print(f"GPU: {cv2.cuda.getDevice()}")
    
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--rpi-ip", default=os.environ.get("RPI_IP", "192.168.1.136"))
    ap.add_argument("--udp-port", type=int, default=int(os.environ.get("UDP_PORT", "5555")))
    args = ap.parse_args()

    tracker = JetsonFaceTracker(rpi_ip=args.rpi_ip, udp_port=args.udp_port)
    
    # Start web server in background
    from threading import Thread
    web_thread = Thread(target=lambda: app.run(host='0.0.0.0', port=8080, threaded=True, use_reloader=False))
    web_thread.daemon = True
    web_thread.start()
    print("Web server at http://0.0.0.0:8080")
    
    try:
        tracker.start()
    except KeyboardInterrupt:
        print("\nStopping...")
        tracker.stop()
