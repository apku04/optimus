#!/usr/bin/env python3
"""
Face Tracker v3 - HAAR CASCADE PRIMARY (actual face detection)
YOLO person detection was causing Y-axis instability because it detects body, not face.
Haar cascade detects actual faces much more reliably for head tracking.
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
EMA_ALPHA = 0.4  # Position smoothing

# Flask for web UI
from flask import Flask, Response, jsonify
app = Flask(__name__)
frame_lock = threading.Lock()
latest_frame = None
tracker = None


class FaceDetectorHaar:
    """Haar cascade face detector - detects ACTUAL FACES, not bodies"""
    
    def __init__(self):
        # Load multiple cascades for better detection
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.face_alt = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_alt.xml')
        self.profile_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_profileface.xml')
        
        print("✓ Haar cascades loaded (frontal + profile)")
    
    def detect(self, frame):
        """Detect faces using Haar cascade - returns (x, y, w, h, conf)"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Try frontal face first
        faces = self.face_cascade.detectMultiScale(
            gray, 
            scaleFactor=1.1, 
            minNeighbors=5, 
            minSize=(60, 60),
            maxSize=(400, 400)
        )
        
        if len(faces) == 0:
            # Try alt cascade
            faces = self.face_alt.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=4, minSize=(60, 60))
        
        if len(faces) == 0:
            # Try profile (side face)
            faces = self.profile_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=4, minSize=(60, 60))
        
        # Return with confidence estimate based on size
        result = []
        for (x, y, w, h) in faces:
            # Larger face = higher confidence (closer = more reliable)
            conf = min(0.95, 0.5 + (w * h) / (frame.shape[0] * frame.shape[1]) * 10)
            result.append((x, y, w, h, conf))
        
        return result


class FaceTrackerV3:
    """Face tracker using Haar cascade - stable Y axis"""
    
    def __init__(self):
        self.running = True
        self.paused = False
        self.protocol_state = "ACTIVE"
        
        # Detector - HAAR CASCADE for real faces
        self.detector = FaceDetectorHaar()
        
        # Smoothed outputs
        self.smooth_x = 0.0
        self.smooth_y = 0.0
        self.smooth_conf = 0.0
        self.smooth_box_size = 0.0
        
        # Stability tracking
        self.history_x = deque(maxlen=10)
        self.history_y = deque(maxlen=10)
        self.stability = 1.0
        
        # Last known good position (for when face is briefly lost)
        self.last_good_x = 0.0
        self.last_good_y = 0.0
        self.frames_since_detection = 0
        
        # Stats
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.face_detected = False
        
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
                    print("[CMD] RESUMED")
                elif cmd == "PING":
                    sock.sendto(b"PONG", addr)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[CMD] Error: {e}")
    
    def _open_camera(self):
        """Open camera"""
        # Try GStreamer first
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
        
        # Fallback to V4L2
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
        
        stability = 1.0 - min(1.0, variance / 0.05)
        return max(0.0, stability)
    
    def process_frame(self, frame):
        """Process frame, return (x, y) offset"""
        global latest_frame
        
        h, w = frame.shape[:2]
        cx, cy = w / 2, h / 2
        
        # Detect ACTUAL faces (not person bodies)
        faces = self.detector.detect(frame)
        
        if faces:
            # Get largest face
            best = max(faces, key=lambda f: f[2] * f[3])
            fx, fy, fw, fh, conf = best
            
            # Face center
            face_cx = fx + fw / 2
            face_cy = fy + fh / 2
            
            # Normalized offset (-1 to 1)
            raw_x = (face_cx - cx) / cx
            raw_y = (face_cy - cy) / cy
            
            # EMA smoothing
            self.smooth_x = self.smooth_x * (1 - EMA_ALPHA) + raw_x * EMA_ALPHA
            self.smooth_y = self.smooth_y * (1 - EMA_ALPHA) + raw_y * EMA_ALPHA
            
            # Track stability
            self.history_x.append(raw_x)
            self.history_y.append(raw_y)
            self.stability = self._calculate_stability()
            
            # Box size (larger = closer)
            raw_box_size = (fw * fh) / (w * h)
            box_size = min(1.0, raw_box_size * 8)
            self.smooth_box_size = self.smooth_box_size * 0.8 + box_size * 0.2
            
            # Confidence
            self.smooth_conf = self.smooth_conf * 0.7 + conf * 0.3
            
            self.face_detected = True
            self.last_good_x = self.smooth_x
            self.last_good_y = self.smooth_y
            self.frames_since_detection = 0
            
            # Draw
            cv2.rectangle(frame, (fx, fy), (fx+fw, fy+fh), (0, 255, 0), 2)
            cv2.circle(frame, (int(face_cx), int(face_cy)), 5, (0, 0, 255), -1)
            
        else:
            self.frames_since_detection += 1
            
            # Keep last position for a short time
            if self.frames_since_detection < 10:
                # Decay towards center slowly
                self.smooth_x *= 0.95
                self.smooth_y *= 0.95
            else:
                self.face_detected = False
                self.smooth_conf *= 0.9
        
        # Draw info
        status = f"HAAR | FPS:{self.fps:.0f} | {self.protocol_state}"
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"X:{self.smooth_x:+.2f} Y:{self.smooth_y:+.2f}", (10, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, f"Conf:{self.smooth_conf:.2f} Stab:{self.stability:.2f}", 
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
        
        with frame_lock:
            latest_frame = frame.copy()
        
        return self.smooth_x, self.smooth_y
    
    def send_to_rpi(self, offset_x, offset_y):
        """Send data to RPI"""
        if self.paused:
            return
        
        data = {
            "x": round(offset_x, 4),
            "y": round(offset_y, 4),
            "detected": self.face_detected,
            "confidence": round(self.smooth_conf, 3),
            "box_size": round(self.smooth_box_size, 3),
            "stability": round(self.stability, 3),
            "z": 0.5,
            "stereo": False,
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
        print("FACE TRACKER V3 - HAAR CASCADE")
        print("Detects ACTUAL FACES, not person bodies")
        print("="*50)
        print(f"Target: {RPI_IP}:{UDP_PORT}")
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
                
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            if self.cap:
                self.cap.release()
            self.running = False


# Web UI
@app.route('/')
def index():
    return '''<!DOCTYPE html>
<html><head><title>Face Tracker V3 - Haar</title>
<style>
body { font-family: monospace; background: #111; color: #eee; margin: 20px; }
img { border: 2px solid #333; }
.stats { display: inline-block; margin-left: 20px; vertical-align: top; }
</style>
</head><body>
<h1>Face Tracker V3 - HAAR CASCADE</h1>
<p>Detects actual faces, not person bodies</p>
<img src="/video_feed" width="640" height="480">
<div class="stats">
<h3>Data</h3>
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
            'fps': tracker.fps
        })
    return jsonify({})


def main():
    global tracker
    
    tracker = FaceTrackerV3()
    tracker_thread = threading.Thread(target=tracker.run, daemon=True)
    tracker_thread.start()
    
    print("Web UI on port 5000...")
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
