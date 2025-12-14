#!/usr/bin/env python3
"""
Face Tracker v2 - Enhanced YOLO with full data output
Sends: x, y, bbox_size, confidence, stability, depth
"""

import cv2
import numpy as np
import socket
import json
import time
import threading
from collections import deque

# Config
RPI_IP = "192.168.1.136"
UDP_PORT = 5555
COMMAND_PORT = 5556
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# Detection settings
YOLO_CONF_THRESHOLD = 0.3
EMA_ALPHA = 0.3  # Position smoothing (higher = more responsive)
BOX_ALPHA = 0.2  # Box size smoothing

# Flask for web UI (optional)
from flask import Flask, Response, jsonify
app = Flask(__name__)
frame_lock = threading.Lock()
latest_frame = None
tracker = None


class FaceDetectorYOLO:
    """YOLO-based face/person detector with confidence tracking"""
    
    def __init__(self):
        self.model = None
        self.use_gpu = False
        self.haar = None
        
        try:
            from ultralytics import YOLO
            import torch
            
            # Try face-specific model first, fall back to person detection
            self.model = YOLO("yolov8n.pt")
            
            if torch.cuda.is_available():
                self.model.to('cuda')
                self.use_gpu = True
                print(f"✓ YOLO on GPU: {torch.cuda.get_device_name(0)}")
            else:
                print("⚠ YOLO on CPU")
            
            # Warmup
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self.model.predict(dummy, verbose=False)
            print("✓ YOLO warmed up")
            
        except Exception as e:
            print(f"YOLO failed: {e}, using Haar cascade")
            self.model = None
            self.haar = cv2.CascadeClassifier(
                cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    
    def detect(self, frame):
        """
        Detect faces and return list of (x, y, w, h, confidence)
        NOW INCLUDES CONFIDENCE!
        """
        if self.model is None:
            return self._detect_haar(frame)
        
        try:
            results = self.model.predict(frame, verbose=False, conf=YOLO_CONF_THRESHOLD)
            
            faces = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])  # YOLO confidence
                    
                    # Class 0 = person in COCO dataset
                    # Upper body region approximation for person class
                    if cls == 0 and conf >= YOLO_CONF_THRESHOLD:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        w, h = x2 - x1, y2 - y1
                        
                        # Estimate head region (top 30% of person bbox)
                        head_h = h * 0.3
                        head_y1 = y1
                        head_y2 = y1 + head_h
                        head_center_y = (head_y1 + head_y2) / 2
                        
                        # Face center estimate
                        x, y = int((x1 + x2) / 2), int(head_center_y)
                        fw, fh = int(w * 0.4), int(head_h)  # Approximate face size
                        
                        faces.append((x - fw//2, y - fh//2, fw, fh, conf))
            
            return faces
            
        except Exception as e:
            print(f"YOLO error: {e}")
            return self._detect_haar(frame)
    
    def _detect_haar(self, frame):
        """Fallback Haar cascade (conf = 0.5 fixed)"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.haar.detectMultiScale(gray, 1.1, 4, minSize=(60, 60))
        return [(x, y, w, h, 0.5) for (x, y, w, h) in faces]


class FaceTrackerV2:
    """Enhanced tracker with rich data output"""
    
    def __init__(self):
        self.running = True
        self.paused = False
        self.protocol_state = "ACTIVE"
        
        # Detector
        self.detector = FaceDetectorYOLO()
        
        # Smoothed outputs
        self.smooth_x = 0.0
        self.smooth_y = 0.0
        self.smooth_conf = 0.0
        self.smooth_box_size = 0.0  # 0-1 normalized (larger = closer)
        
        # Raw values
        self.raw_x = 0.0
        self.raw_y = 0.0
        
        # Stability tracking - variance in recent positions
        self.history_x = deque(maxlen=10)
        self.history_y = deque(maxlen=10)
        self.stability = 1.0  # 0=unstable, 1=stable
        
        # Stats
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.face_detected = False
        
        # Stereo depth (placeholder)
        self.depth = 0.5
        self.use_stereo = False
        self.camera_mode = "SINGLE"
        
        # UDP
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Camera
        self.cap = None
        
        # Command listener
        self.cmd_thread = threading.Thread(target=self._command_listener, daemon=True)
        self.cmd_thread.start()
    
    def _command_listener(self):
        """Listen for PAUSE/RESUME from RPI"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', COMMAND_PORT))
        sock.settimeout(0.1)
        
        print(f"[CMD] Listening on port {COMMAND_PORT}")
        
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                cmd = data.decode().strip().upper()
                
                if cmd == "PAUSE":
                    self.paused = True
                    self.protocol_state = "PAUSED"
                    print("[CMD] PAUSED")
                elif cmd == "RESUME":
                    self.paused = False
                    self.protocol_state = "ACTIVE"
                    self._reset()
                    print("[CMD] RESUMED")
                elif cmd == "PING":
                    sock.sendto(b"PONG", addr)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[CMD] Error: {e}")
    
    def _reset(self):
        """Reset filters on RESUME"""
        self.history_x.clear()
        self.history_y.clear()
        self.stability = 1.0
    
    def _open_camera(self):
        """Open camera with GStreamer or V4L2"""
        pipeline = (
            f"nvarguscamerasrc sensor-id=0 ! "
            f"video/x-raw(memory:NVMM), width={CAMERA_WIDTH}, height={CAMERA_HEIGHT}, "
            f"format=NV12, framerate={CAMERA_FPS}/1 ! "
            f"nvvidconv ! video/x-raw, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! "
            f"appsink drop=1"
        )
        
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if self.cap.isOpened():
            print("✓ Camera opened (GStreamer)")
            return True
        
        # Fallback
        self.cap = cv2.VideoCapture(0)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            print("✓ Camera opened (V4L2)")
            return True
        
        print("✗ No camera")
        return False
    
    def _calculate_stability(self):
        """Calculate stability from position variance"""
        if len(self.history_x) < 5:
            return 1.0
        
        var_x = np.var(list(self.history_x))
        var_y = np.var(list(self.history_y))
        variance = var_x + var_y
        
        # Map variance to stability (higher variance = lower stability)
        # variance of 0.01 or less = stable (1.0)
        # variance of 0.1 or more = unstable (0.0)
        stability = 1.0 - min(1.0, variance / 0.1)
        return stability
    
    def process_frame(self, frame):
        """Process frame, return (x, y) offset"""
        global latest_frame
        
        h, w = frame.shape[:2]
        cx, cy = w / 2, h / 2
        
        # Detect faces (now with confidence!)
        faces = self.detector.detect(frame)
        
        offset_x, offset_y = 0.0, 0.0
        confidence = 0.0
        box_size = 0.0
        
        if faces:
            # Get best face (highest confidence)
            best = max(faces, key=lambda f: f[4])  # f[4] is confidence
            fx, fy, fw, fh, conf = best
            
            face_cx = fx + fw / 2
            face_cy = fy + fh / 2
            
            # Normalized offset (-1 to 1)
            self.raw_x = (face_cx - cx) / cx
            self.raw_y = (face_cy - cy) / cy
            
            # EMA smoothing
            self.smooth_x = self.smooth_x * (1 - EMA_ALPHA) + self.raw_x * EMA_ALPHA
            self.smooth_y = self.smooth_y * (1 - EMA_ALPHA) + self.raw_y * EMA_ALPHA
            
            # Track stability
            self.history_x.append(self.raw_x)
            self.history_y.append(self.raw_y)
            self.stability = self._calculate_stability()
            
            # Bbox size normalized (0-1, larger = closer)
            # Assume face at 50% of frame width = 0.5
            raw_box_size = (fw * fh) / (w * h)  # Area ratio
            box_size = min(1.0, raw_box_size * 10)  # Scale up, cap at 1.0
            self.smooth_box_size = self.smooth_box_size * (1 - BOX_ALPHA) + box_size * BOX_ALPHA
            
            # Smooth confidence
            self.smooth_conf = self.smooth_conf * 0.7 + conf * 0.3
            
            offset_x = self.smooth_x
            offset_y = self.smooth_y
            confidence = self.smooth_conf
            
            self.face_detected = True
            
            # Draw detection
            cv2.rectangle(frame, (fx, fy), (fx+fw, fy+fh), (0, 255, 0), 2)
            cv2.circle(frame, (int(face_cx), int(face_cy)), 5, (0, 0, 255), -1)
            
        else:
            self.face_detected = False
            # Decay smoothed values slowly when face lost
            self.smooth_x *= 0.95
            self.smooth_y *= 0.95
            self.smooth_conf *= 0.9
        
        # Draw info
        status = f"{'GPU' if self.detector.use_gpu else 'CPU'} | FPS:{self.fps:.0f} | {self.protocol_state}"
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"X:{offset_x:+.2f} Y:{offset_y:+.2f}", (10, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, f"Conf:{confidence:.2f} Size:{self.smooth_box_size:.2f} Stab:{self.stability:.2f}", 
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Center crosshair
        cv2.line(frame, (int(cx)-20, int(cy)), (int(cx)+20, int(cy)), (0, 255, 255), 1)
        cv2.line(frame, (int(cx), int(cy)-20), (int(cx), int(cy)+20), (0, 255, 255), 1)
        
        # FPS
        self.frame_count += 1
        if time.time() - self.last_fps_time >= 1.0:
            self.fps = self.frame_count
            self.frame_count = 0
            self.last_fps_time = time.time()
        
        # Store for web
        with frame_lock:
            latest_frame = frame.copy()
        
        return offset_x, offset_y
    
    def send_to_rpi(self, offset_x, offset_y):
        """Send rich data to RPI"""
        if self.paused:
            return
        
        data = {
            "x": round(offset_x, 4),
            "y": round(offset_y, 4),
            "detected": self.face_detected,
            "confidence": round(self.smooth_conf, 3),
            "box_size": round(self.smooth_box_size, 3),  # NEW: larger = closer
            "stability": round(self.stability, 3),       # NEW: position stability
            "z": round(self.depth, 3),
            "stereo": self.use_stereo,
            "timestamp": time.time()
        }
        
        try:
            msg = json.dumps(data).encode()
            self.udp_socket.sendto(msg, (RPI_IP, UDP_PORT))
        except Exception as e:
            print(f"UDP error: {e}")
    
    def run(self):
        """Main loop"""
        print("\n" + "="*50)
        print("FACE TRACKER V2 - Enhanced YOLO")
        print("="*50)
        print(f"Target: {RPI_IP}:{UDP_PORT}")
        print(f"Commands: port {COMMAND_PORT}")
        print("="*50 + "\n")
        
        if not self._open_camera():
            return
        
        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue
                
                offset_x, offset_y = self.process_frame(frame)
                self.send_to_rpi(offset_x, offset_y)
                
                # Small sleep to prevent 100% CPU
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            if self.cap:
                self.cap.release()
            self.running = False


# Web endpoints
@app.route('/')
def index():
    return '''<!DOCTYPE html>
<html><head><title>Face Tracker V2</title>
<style>
body { font-family: monospace; background: #111; color: #eee; margin: 20px; }
img { border: 2px solid #333; }
.stats { display: inline-block; margin-left: 20px; vertical-align: top; }
</style>
</head><body>
<h1>Face Tracker V2</h1>
<img src="/video_feed" width="640" height="480">
<div class="stats">
<h3>Data Stream</h3>
<pre id="data">Loading...</pre>
</div>
<script>
setInterval(() => {
    fetch('/data').then(r=>r.json()).then(d => {
        document.getElementById('data').textContent = JSON.stringify(d, null, 2);
    });
}, 100);
</script>
</body></html>'''

@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            with frame_lock:
                if latest_frame is not None:
                    _, jpg = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')
            time.sleep(0.033)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/data')
def get_data():
    if tracker:
        return jsonify({
            'x': tracker.smooth_x,
            'y': tracker.smooth_y,
            'detected': tracker.face_detected,
            'confidence': tracker.smooth_conf,
            'box_size': tracker.smooth_box_size,
            'stability': tracker.stability,
            'fps': tracker.fps,
            'gpu': tracker.detector.use_gpu,
            'protocol': tracker.protocol_state
        })
    return jsonify({})


def main():
    global tracker
    
    tracker = FaceTrackerV2()
    tracker_thread = threading.Thread(target=tracker.run, daemon=True)
    tracker_thread.start()
    
    print("Starting web UI on port 5000...")
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
