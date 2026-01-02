#!/usr/bin/env python3
"""
Simple delay monitor that measures UDP packet delay from Jetson
and provides a web endpoint for the delay stats.
"""

import json
import socket
import statistics
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional
import threading
from flask import Flask, jsonify

app = Flask(__name__)

@dataclass
class FacePacket:
    x: float
    y: float
    detected: bool
    confidence: float
    timestamp: float

class DelayMonitor:
    def __init__(self, port: int = 5555):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        
        # Delay tracking
        self.delays = deque(maxlen=100)  # Keep last 100 samples
        self.latest_delay = 0.0
        self.packet_count = 0
        self.running = False
        
    def _parse_packet(self, msg: str) -> Optional[FacePacket]:
        msg = msg.strip()
        if not msg.startswith("{"):
            return None
        
        try:
            data = json.loads(msg)
        except json.JSONDecodeError:
            return None
        
        detected = bool(data.get("detected", data.get("face_detected", False)))
        x = data.get("x", data.get("x_offset", data.get("offset_x", 0.0)))
        y = data.get("y", data.get("y_offset", data.get("offset_y", 0.0)))
        timestamp = float(data.get("timestamp", time.time()))
        confidence = float(data.get("confidence", 1.0) or 0.0)
        
        return FacePacket(
            x=float(x),
            y=float(y),
            detected=detected,
            confidence=confidence,
            timestamp=timestamp
        )
    
    def start_monitoring(self):
        """Start monitoring UDP packets"""
        self.running = True
        print(f"Delay monitor started on UDP port {5555}")
        
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                receive_time = time.time()
                self.packet_count += 1
                
                pkt = self._parse_packet(data.decode("utf-8", errors="ignore"))
                if pkt:
                    delay_ms = (receive_time - pkt.timestamp) * 1000.0
                    self.latest_delay = delay_ms
                    self.delays.append(delay_ms)
                    
                    # Print every 30 packets
                    if self.packet_count % 30 == 0:
                        avg_delay = statistics.mean(self.delays) if self.delays else 0
                        print(f"Delay: {delay_ms:.1f}ms (avg: {avg_delay:.1f}ms, packets: {self.packet_count})")
                        
            except BlockingIOError:
                time.sleep(0.001)
            except Exception as e:
                print(f"Monitor error: {e}")
                time.sleep(0.1)
    
    def get_stats(self):
        """Get delay statistics"""
        if not self.delays:
            return {
                "latest_delay_ms": 0.0,
                "avg_delay_ms": 0.0,
                "min_delay_ms": 0.0,
                "max_delay_ms": 0.0,
                "packet_count": self.packet_count,
                "samples": 0
            }
        
        return {
            "latest_delay_ms": round(self.latest_delay, 1),
            "avg_delay_ms": round(statistics.mean(self.delays), 1),
            "min_delay_ms": round(min(self.delays), 1),
            "max_delay_ms": round(max(self.delays), 1),
            "packet_count": self.packet_count,
            "samples": len(self.delays)
        }

# Global monitor instance
monitor = DelayMonitor()

@app.route('/delay')
def get_delay():
    """Return delay statistics as JSON"""
    return jsonify(monitor.get_stats())

@app.route('/delay/text')
def get_delay_text():
    """Return delay as plain text for easy overlay"""
    stats = monitor.get_stats()
    return f"Delay: {stats['latest_delay_ms']}ms (avg: {stats['avg_delay_ms']}ms)"

def run_monitor():
    """Run the UDP monitor in a separate thread"""
    monitor.start_monitoring()

if __name__ == "__main__":
    # Start UDP monitoring in background thread
    monitor_thread = threading.Thread(target=run_monitor, daemon=True)
    monitor_thread.start()
    
    # Start Flask web server
    print("Starting delay monitor web server on port 8080")
    print("Access delay stats at: http://192.168.1.136:8080/delay")
    app.run(host='0.0.0.0', port=8080, debug=False)