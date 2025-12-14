#!/usr/bin/env python3
"""
Face Tracker - Jetson Side
Detects faces using OpenCV and sends position to RPi via UDP
Includes web streaming for debugging (http://jetson-ip:8080)

Simple, no ROS required.
"""

import cv2
import os
import socket
import json
import time
import signal
import sys
import threading
from flask import Flask, Response, render_template_string

app = Flask(__name__)

# Global frame for web streaming
latest_frame = None
frame_lock = threading.Lock()


class FaceTracker:
    """Detects faces and sends position via UDP"""
    
    def __init__(self, rpi_ip: str = "192.168.1.136", udp_port: int = 5555, 
                 invert_pan: bool = False, invert_tilt: bool = False):
        self.rpi_ip = rpi_ip
        self.udp_port = udp_port
        self.invert_pan = invert_pan
        self.invert_tilt = invert_tilt
        
        # Camera settings
        self.camera_width = 1280
        self.camera_height = 720
        self.process_width = 640
        self.process_height = 480
        
        # Face detection
        cascade_paths = [
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        ]
        if hasattr(cv2, 'data') and cv2.data.haarcascades:
            cascade_paths.insert(0, cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
        
        self.face_cascade = None
        for path in cascade_paths:
            if path and os.path.exists(path):
                self.face_cascade = cv2.CascadeClassifier(path)
                print(f"Loaded cascade from: {path}")
                break
        
        if self.face_cascade is None:
            raise RuntimeError("Could not find Haar cascade file!")
        
        # UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Camera
        self.cap = None
        self.running = False
        
        # Stats
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0
        self.last_x_offset = 0
        self.last_y_offset = 0
    
    def _create_gstreamer_pipeline(self):
        """Create GStreamer pipeline for CSI camera"""
        return (
            f"nvarguscamerasrc sensor-id=0 ! "
            f"video/x-raw(memory:NVMM), width={self.camera_width}, height={self.camera_height}, "
            f"format=NV12, framerate=30/1 ! "
            f"nvvidconv flip-method=0 ! "
            f"video/x-raw, width={self.process_width}, height={self.process_height}, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
        )
    
    def _open_camera(self):
        """Try to open camera (CSI first, then USB)"""
        # Try CSI camera with GStreamer
        try:
            pipeline = self._create_gstreamer_pipeline()
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                print("Opened CSI camera via GStreamer")
                return True
        except Exception as e:
            print(f"CSI camera failed: {e}")
        
        # Try USB camera
        for i in range(3):
            self.cap = cv2.VideoCapture(i)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.process_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.process_height)
                print(f"Opened USB camera /dev/video{i}")
                return True
        
        return False
    
    def _send_udp(self, message: dict):
        """Send message to RPi via UDP"""
        try:
            data = json.dumps(message).encode()
            self.socket.sendto(data, (self.rpi_ip, self.udp_port))
        except Exception as e:
            print(f"UDP send error: {e}")
    
    def start(self):
        """Start face tracking"""
        global latest_frame
        
        if not self._open_camera():
            print("ERROR: Could not open any camera!")
            return False
        
        print(f"Face Tracker started - sending to {self.rpi_ip}:{self.udp_port}")
        print(f"Invert pan: {self.invert_pan}, Invert tilt: {self.invert_tilt}")
        print("Press Ctrl+C to stop")
        
        # Send initial status
        self._send_udp({"type": "status", "status": "Active"})
        
        self.running = True
        self._run_loop()
        
        return True
    
    def _run_loop(self):
        """Main detection loop"""
        global latest_frame
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                time.sleep(0.1)
                continue
            
            # Convert to grayscale for detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(60, 60),
                flags=cv2.CASCADE_SCALE_IMAGE
            )
            
            # Draw center crosshair
            h, w = frame.shape[:2]
            cv2.line(frame, (w//2 - 30, h//2), (w//2 + 30, h//2), (0, 255, 0), 2)
            cv2.line(frame, (w//2, h//2 - 30), (w//2, h//2 + 30), (0, 255, 0), 2)
            
            x_offset = 0
            y_offset = 0
            
            if len(faces) > 0:
                # Track the largest face
                largest_face = max(faces, key=lambda f: f[2] * f[3])
                x, y, fw, fh = largest_face
                
                # Draw rectangle around face
                cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 3)
                
                # Calculate center of face
                face_center_x = x + fw / 2
                face_center_y = y + fh / 2
                
                # Draw face center point
                cv2.circle(frame, (int(face_center_x), int(face_center_y)), 8, (0, 0, 255), -1)
                
                # Draw line from center to face
                cv2.line(frame, (w//2, h//2), (int(face_center_x), int(face_center_y)), (255, 0, 0), 2)
                
                # Convert to normalized offset (-1 to 1)
                frame_center_x = w / 2
                frame_center_y = h / 2
                
                x_offset = (face_center_x - frame_center_x) / frame_center_x
                y_offset = (face_center_y - frame_center_y) / frame_center_y
                
                # Apply inversion if needed
                if self.invert_pan:
                    x_offset = -x_offset
                if self.invert_tilt:
                    y_offset = -y_offset
                
                self.last_x_offset = x_offset
                self.last_y_offset = y_offset
                
                # Face size relative to frame
                face_size = (fw * fh) / (w * h)
                
                # Send to RPi
                self._send_udp({
                    "type": "face_position",
                    "x_offset": round(x_offset, 3),
                    "y_offset": round(y_offset, 3),
                    "face_size": round(face_size, 3)
                })
            
            # Draw status info on frame
            status_text = f"FPS: {self.fps:.1f} | Faces: {len(faces)}"
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            offset_text = f"X: {self.last_x_offset:+.2f} | Y: {self.last_y_offset:+.2f}"
            cv2.putText(frame, offset_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            direction = ""
            if abs(self.last_x_offset) > 0.05:
                direction += "LEFT " if self.last_x_offset < 0 else "RIGHT "
            if abs(self.last_y_offset) > 0.05:
                direction += "UP" if self.last_y_offset < 0 else "DOWN"
            if direction:
                cv2.putText(frame, f"Move: {direction}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            
            # Target info
            cv2.putText(frame, f"Target: {self.rpi_ip}:{self.udp_port}", (10, h - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
            
            # Update global frame for web streaming
            with frame_lock:
                latest_frame = frame.copy()
            
            # Calculate FPS
            self.frame_count += 1
            now = time.time()
            if now - self.last_fps_time >= 1.0:
                self.fps = self.frame_count / (now - self.last_fps_time)
                self.frame_count = 0
                self.last_fps_time = now
            
            # Small delay
            time.sleep(0.01)
        
        # Cleanup
        self.cap.release()
        self._send_udp({"type": "status", "status": "Stopped"})
        print("Face Tracker stopped")
    
    def stop(self):
        """Stop the tracker"""
        self.running = False


# Flask web streaming
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Optimus Face Tracker</title>
    <style>
        body { 
            background: #1a1a1a; 
            color: white; 
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            text-align: center;
        }
        h1 { color: #4CAF50; }
        img { 
            max-width: 100%; 
            border: 2px solid #4CAF50;
            border-radius: 8px;
        }
        .info {
            margin-top: 10px;
            color: #888;
            font-size: 14px;
        }
        .legend {
            display: inline-block;
            text-align: left;
            margin-top: 15px;
            padding: 10px;
            background: #333;
            border-radius: 5px;
        }
    </style>
</head>
<body>
    <h1>🤖 Optimus Face Tracker</h1>
    <img src="/video_feed" alt="Camera Feed">
    <div class="info">
        <div class="legend">
            <b>Legend:</b><br>
            🟩 Green box = Detected face<br>
            🔴 Red dot = Face center<br>
            🔵 Blue line = Offset from center<br>
            ➕ Green crosshair = Frame center<br>
            <br>
            <b>Offsets:</b><br>
            X negative = face on LEFT → robot turns LEFT<br>
            X positive = face on RIGHT → robot turns RIGHT<br>
            Y negative = face ABOVE → robot tilts UP<br>
            Y positive = face BELOW → robot tilts DOWN<br>
        </div>
    </div>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

def generate_frames():
    """Generate frames for web streaming"""
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.1)
                continue
            frame = latest_frame.copy()
        
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ret:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)  # ~30fps

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


tracker = None

def signal_handler(sig, frame):
    """Handle Ctrl+C"""
    global tracker
    print("\nShutting down...")
    if tracker:
        tracker.stop()
    sys.exit(0)


def main():
    global tracker
    import argparse
    
    parser = argparse.ArgumentParser(description="Face Tracker for Optimus Robot")
    parser.add_argument("--rpi-ip", default="192.168.1.136", help="RPi IP address")
    parser.add_argument("--port", type=int, default=5555, help="UDP port")
    parser.add_argument("--web-port", type=int, default=8080, help="Web streaming port")
    parser.add_argument("--invert-pan", action="store_true", help="Invert pan direction")
    parser.add_argument("--invert-tilt", action="store_true", help="Invert tilt direction")
    args = parser.parse_args()
    
    print("=" * 50)
    print("  OPTIMUS Face Tracker (Jetson)")
    print("=" * 50)
    print()
    
    tracker = FaceTracker(
        rpi_ip=args.rpi_ip, 
        udp_port=args.port,
        invert_pan=args.invert_pan,
        invert_tilt=args.invert_tilt
    )
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start Flask in a thread
    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=args.web_port, threaded=True, use_reloader=False),
        daemon=True
    )
    flask_thread.start()
    print(f"Web preview at: http://192.168.1.133:{args.web_port}")
    
    tracker.start()


if __name__ == "__main__":
    main()
