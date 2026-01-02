#!/usr/bin/env python3
"""
Stream overlay - fetches video from Jetson and overlays face position data
received via UDP from Jetson.

Run this on RPi to see the video with face tracking overlay.
"""

import cv2
import socket
import json
import time
import threading
import numpy as np
import requests
from flask import Flask, Response

app = Flask(__name__)

# Frame from Jetson
latest_jetson_frame = None
jetson_frame_lock = threading.Lock()

# Global state
latest_face_data = {
    "x": 0.0,
    "y": 0.0,
    "detected": False,
    "confidence": 0.0,
    "timestamp": 0,
    "delay_ms": 0.0,
}
face_lock = threading.Lock()
delay_samples = []

# Config
JETSON_IP = "192.168.1.133"
JETSON_STREAM_URL = f"http://{JETSON_IP}:5000/video_feed"
UDP_PORT = 5555


def jetson_stream_reader():
    """Background thread to read MJPEG stream from Jetson"""
    global latest_jetson_frame
    
    while True:
        try:
            print(f"[STREAM] Connecting to {JETSON_STREAM_URL}...")
            response = requests.get(JETSON_STREAM_URL, stream=True, timeout=5)
            
            if response.status_code != 200:
                print(f"[STREAM] Bad status: {response.status_code}")
                time.sleep(2)
                continue
            
            print("[STREAM] Connected to Jetson!")
            
            # Read MJPEG stream
            bytes_buffer = b''
            for chunk in response.iter_content(chunk_size=4096):
                bytes_buffer += chunk
                
                # Look for JPEG start and end markers
                start = bytes_buffer.find(b'\xff\xd8')
                end = bytes_buffer.find(b'\xff\xd9')
                
                if start != -1 and end != -1 and end > start:
                    jpg_data = bytes_buffer[start:end+2]
                    bytes_buffer = bytes_buffer[end+2:]
                    
                    # Decode JPEG
                    frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        with jetson_frame_lock:
                            latest_jetson_frame = frame
                            
        except Exception as e:
            print(f"[STREAM] Error: {e}")
            time.sleep(2)


def udp_receiver():
    """Background thread to receive UDP face data from Jetson"""
    global latest_face_data, delay_samples
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(1.0)
    
    print(f"[UDP] Listening on port {UDP_PORT}...")
    
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            recv_time = time.time()
            
            msg = json.loads(data.decode("utf-8", errors="ignore"))
            
            # Calculate delay
            jetson_ts = float(msg.get("timestamp", recv_time))
            delay_ms = (recv_time - jetson_ts) * 1000.0
            
            delay_samples.append(delay_ms)
            if len(delay_samples) > 50:
                delay_samples.pop(0)
            
            with face_lock:
                latest_face_data = {
                    "x": float(msg.get("x", msg.get("x_offset", msg.get("offset_x", 0)))),
                    "y": float(msg.get("y", msg.get("y_offset", msg.get("offset_y", 0)))),
                    "detected": bool(msg.get("detected", msg.get("face_detected", False))),
                    "confidence": float(msg.get("confidence", 0)),
                    "timestamp": recv_time,
                    "delay_ms": delay_ms,
                }
                
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP] Error: {e}")
            time.sleep(0.1)


def generate_frames():
    """Generate video frames with overlay"""
    
    print("[OVERLAY] Starting frame generation...")
    
    while True:
        # Get latest frame from Jetson
        with jetson_frame_lock:
            frame = latest_jetson_frame.copy() if latest_jetson_frame is not None else None
        
        if frame is None:
            # Generate placeholder frame
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "Waiting for Jetson stream...", (50, 200),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(frame, f"URL: {JETSON_STREAM_URL}", (50, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        
        # Get latest face data
        with face_lock:
            fd = latest_face_data.copy()
        
        # Calculate average delay
        avg_delay = sum(delay_samples) / len(delay_samples) if delay_samples else 0
        
        # Draw image center crosshair (green)
        cv2.line(frame, (cx - 40, cy), (cx + 40, cy), (0, 255, 0), 2)
        cv2.line(frame, (cx, cy - 40), (cx, cy + 40), (0, 255, 0), 2)
        cv2.putText(frame, "CENTER", (cx + 45, cy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Convert normalized coordinates to pixel position
        face_px = int(cx + fd["x"] * cx)
        face_py = int(cy + fd["y"] * cy)
        
        # Draw face position (yellow/cyan)
        if fd["detected"]:
            color = (0, 255, 255)  # Yellow for detected
            # Draw crosshair at face position
            cv2.line(frame, (face_px - 30, face_py), (face_px + 30, face_py), color, 3)
            cv2.line(frame, (face_px, face_py - 30), (face_px, face_py + 30), color, 3)
            cv2.circle(frame, (face_px, face_py), 15, color, 2)
            
            # Draw line from center to face
            cv2.line(frame, (cx, cy), (face_px, face_py), (255, 0, 0), 2)
            
            # Label
            cv2.putText(frame, "FACE", (face_px + 20, face_py - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        else:
            color = (0, 0, 255)  # Red for not detected
            cv2.circle(frame, (face_px, face_py), 10, color, 2)
        
        # Draw info overlay (top-left)
        y_pos = 30
        cv2.putText(frame, f"UDP Face Position (from Jetson)", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_pos += 30
        cv2.putText(frame, f"X: {fd['x']:+.4f}  Y: {fd['y']:+.4f}", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        y_pos += 30
        cv2.putText(frame, f"Detected: {fd['detected']}  Conf: {fd['confidence']:.2f}", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if fd["detected"] else (0, 0, 255), 2)
        y_pos += 30
        cv2.putText(frame, f"Network Delay: {fd['delay_ms']:.1f}ms (avg: {avg_delay:.1f}ms)", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Pixel position
        y_pos += 30
        cv2.putText(frame, f"Face Pixel: ({face_px}, {face_py})", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Encode and yield
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        
        time.sleep(0.033)  # ~30fps


@app.route('/')
def index():
    return '''
    <html>
    <head><title>Face Tracker - Stream with Overlay</title></head>
    <body style="background:#111;color:#fff;text-align:center;font-family:sans-serif;margin:0;padding:20px;">
        <h1>Face Position Overlay</h1>
        <p>Video from Jetson + UDP face position data</p>
        <img src="/video" style="max-width:100%;border:2px solid #0f0;">
        <p style="color:#ff0;">Yellow crosshair = Face position from UDP | Green crosshair = Image center</p>
    </body>
    </html>
    '''


@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/data')
def data():
    """Return current face data as JSON"""
    with face_lock:
        fd = latest_face_data.copy()
    avg_delay = sum(delay_samples) / len(delay_samples) if delay_samples else 0
    fd["avg_delay_ms"] = avg_delay
    return fd


if __name__ == "__main__":
    # Start Jetson stream reader thread
    stream_thread = threading.Thread(target=jetson_stream_reader, daemon=True)
    stream_thread.start()
    
    # Start UDP receiver thread
    udp_thread = threading.Thread(target=udp_receiver, daemon=True)
    udp_thread.start()
    
    print("=" * 60)
    print("FACE TRACKER STREAM OVERLAY")
    print("=" * 60)
    print(f"Jetson stream: {JETSON_STREAM_URL}")
    print(f"UDP port: {UDP_PORT}")
    print()
    print("View at: http://192.168.1.136:8080/")
    print("=" * 60)
    
    app.run(host='0.0.0.0', port=8080, threaded=True, debug=False)
