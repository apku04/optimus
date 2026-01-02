#!/usr/bin/env python3
"""
VELOCITY MODE - Smooth continuous motion instead of choppy position jumps.
Sets motor speed/direction, not discrete positions.
"""

import json
import time
import threading
from pathlib import Path
from flask import Flask, render_template_string, jsonify, request

import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))

try:
    from klipper_direct import KlipperDirect
except ImportError:
    print("ERROR: klipper_direct not found")
    sys.exit(1)

app = Flask(__name__)

# Config
config_path = Path(__file__).parent.parent / "config.json"
with open(config_path) as f:
    config = json.load(f)

PAN_MIN = config["motors"]["pan"]["min"]
PAN_MAX = config["motors"]["pan"]["max"]
TILT_MIN = config["motors"]["tilt"]["min"]
TILT_MAX = config["motors"]["tilt"]["max"]

# Velocity control parameters
MAX_SPEED = 30       # Max degrees/sec - SLOW for smooth motion
ACCEL = 100          # Gentle acceleration
RAMP_TIME = 0.1      # Seconds to ramp speed

# State
state = {
    "pan": 0.0,
    "tilt": 0.0,
    "target_pan": 0.0,
    "target_tilt": 0.0,
    "moving": False,
}
state_lock = threading.Lock()
klipper = None


def connect_klipper():
    global klipper
    socket_path = "/home/acp/printer_data/comms/klippy.sock"
    try:
        klipper = KlipperDirect(socket_path)
        if klipper.connect():
            print(f"[KLIPPER] Connected")
            # Enable steppers
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1")
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=1")
            # Set current position as 0
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_0 SET_POSITION=0")
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_1 SET_POSITION=0")
            return True
    except Exception as e:
        print(f"[KLIPPER] Error: {e}")
    return False


def smooth_move_thread():
    """
    Single smooth move to target - no choppy intermediate steps.
    Only sends new command when target changes significantly.
    """
    global state
    
    current_pan, current_tilt = 0.0, 0.0
    last_target_pan, last_target_tilt = 0.0, 0.0
    
    while True:
        with state_lock:
            target_pan = state["target_pan"]
            target_tilt = state["target_tilt"]
        
        # Only send new move if target changed significantly
        pan_changed = abs(target_pan - last_target_pan) > 0.5
        tilt_changed = abs(target_tilt - last_target_tilt) > 0.5
        
        if pan_changed or tilt_changed:
            # Calculate distance
            pan_dist = abs(target_pan - current_pan)
            tilt_dist = abs(target_tilt - current_tilt)
            
            # Calculate appropriate speed based on distance
            # Longer distance = can go faster, short distance = go slow
            pan_speed = min(MAX_SPEED, max(10, pan_dist * 3))
            tilt_speed = min(MAX_SPEED, max(10, tilt_dist * 3))
            
            with state_lock:
                state["moving"] = True
            
            # Send ONE smooth move command with SYNC=0 for both
            # This lets both axes move simultaneously and smoothly
            if klipper:
                cmd = f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={target_pan:.2f} SPEED={pan_speed:.0f} ACCEL={ACCEL} SYNC=0"
                klipper.gcode(cmd, wait=False)
                cmd = f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={target_tilt:.2f} SPEED={tilt_speed:.0f} ACCEL={ACCEL} SYNC=0"
                klipper.gcode(cmd, wait=False)
            
            current_pan, current_tilt = target_pan, target_tilt
            last_target_pan, last_target_tilt = target_pan, target_tilt
            
            with state_lock:
                state["pan"] = target_pan
                state["tilt"] = target_tilt
                state["moving"] = False
        
        time.sleep(0.05)  # 20Hz check rate - but only sends when target changes


HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Velocity Mode Test</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { 
            background: #0a0a0a; 
            color: #fff; 
            font-family: sans-serif;
            height: 100vh;
            display: flex;
        }
        .control-area {
            flex: 1;
            display: flex;
            align-items: center;
            justify-content: center;
            background: #111;
        }
        .pad {
            width: 80%;
            height: 80%;
            max-width: 500px;
            max-height: 500px;
            background: #1a1a1a;
            border: 2px solid #f80;
            border-radius: 10px;
            position: relative;
            cursor: crosshair;
        }
        .crosshair { position: absolute; background: #333; }
        .crosshair-h { width: 100%; height: 1px; top: 50%; }
        .crosshair-v { height: 100%; width: 1px; left: 50%; }
        .cursor {
            position: absolute;
            width: 30px;
            height: 30px;
            border: 3px solid #f80;
            border-radius: 50%;
            transform: translate(-50%, -50%);
            pointer-events: none;
        }
        .sidebar {
            width: 280px;
            padding: 20px;
            background: #0d0d0d;
        }
        h1 { color: #f80; font-size: 1.2em; margin-bottom: 20px; }
        .stat { 
            display: flex; 
            justify-content: space-between; 
            margin: 10px 0;
            padding: 8px;
            background: #1a1a1a;
            border-radius: 4px;
        }
        .stat-value { color: #f80; font-family: monospace; }
        .btn {
            width: 100%;
            padding: 15px;
            margin: 10px 0;
            border: none;
            border-radius: 6px;
            font-size: 1em;
            cursor: pointer;
        }
        .btn-center { background: #a60; color: #fff; }
        .btn-stop { background: #a00; color: #fff; }
        .info { 
            font-size: 0.85em; 
            color: #888; 
            margin-top: 20px;
            padding: 10px;
            background: #1a1a1a;
            border-radius: 4px;
            line-height: 1.5;
        }
        .info strong { color: #f80; }
    </style>
</head>
<body>
    <div class="control-area">
        <div class="pad" id="pad">
            <div class="crosshair crosshair-h"></div>
            <div class="crosshair crosshair-v"></div>
            <div class="cursor" id="cursor"></div>
        </div>
    </div>
    <div class="sidebar">
        <h1>🎯 SMOOTH VELOCITY MODE</h1>
        
        <div class="stat">
            <span>Target Pan</span>
            <span class="stat-value" id="pan">0.00°</span>
        </div>
        <div class="stat">
            <span>Target Tilt</span>
            <span class="stat-value" id="tilt">0.00°</span>
        </div>
        <div class="stat">
            <span>Moving</span>
            <span class="stat-value" id="moving">No</span>
        </div>
        
        <button class="btn btn-center" onclick="center()">⟲ CENTER</button>
        <button class="btn btn-stop" onclick="stop()">⬛ STOP</button>
        
        <div class="info">
            <strong>How it works:</strong><br>
            • Drag to set target position<br>
            • Motor glides smoothly to target<br>
            • No choppy intermediate steps<br>
            • Speed adapts to distance<br>
            <br>
            <strong>Tip:</strong> Make big sweeping movements, not tiny adjustments.
        </div>
    </div>

    <script>
        const pad = document.getElementById('pad');
        const cursor = document.getElementById('cursor');
        let isDragging = false;
        let lastSend = 0;
        
        const panMin = {{ pan_min }}, panMax = {{ pan_max }};
        const tiltMin = {{ tilt_min }}, tiltMax = {{ tilt_max }};
        
        cursor.style.left = '50%';
        cursor.style.top = '50%';
        
        function sendMove(x, y) {
            // Throttle to max 10 updates/sec to avoid flooding
            const now = Date.now();
            if (now - lastSend < 100) return;
            lastSend = now;
            
            const rect = pad.getBoundingClientRect();
            const relX = Math.max(0, Math.min(1, (x - rect.left) / rect.width));
            const relY = Math.max(0, Math.min(1, (y - rect.top) / rect.height));
            
            const pan = (relX - 0.5) * 2 * Math.max(Math.abs(panMin), Math.abs(panMax));
            const tilt = (0.5 - relY) * 2 * Math.max(Math.abs(tiltMin), Math.abs(tiltMax));
            
            cursor.style.left = (relX * 100) + '%';
            cursor.style.top = (relY * 100) + '%';
            
            fetch('/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    pan: Math.max(panMin, Math.min(panMax, pan)),
                    tilt: Math.max(tiltMin, Math.min(tiltMax, tilt))
                })
            });
        }
        
        pad.onmousedown = (e) => { isDragging = true; sendMove(e.clientX, e.clientY); };
        document.onmousemove = (e) => { if (isDragging) sendMove(e.clientX, e.clientY); };
        document.onmouseup = () => { isDragging = false; };
        
        pad.ontouchstart = (e) => { e.preventDefault(); isDragging = true; sendMove(e.touches[0].clientX, e.touches[0].clientY); };
        document.ontouchmove = (e) => { if (isDragging) sendMove(e.touches[0].clientX, e.touches[0].clientY); };
        document.ontouchend = () => { isDragging = false; };
        
        function center() { 
            fetch('/move', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({pan: 0, tilt: 0})}); 
            cursor.style.left = '50%'; 
            cursor.style.top = '50%'; 
        }
        function stop() { fetch('/stop', {method: 'POST'}); }
        
        setInterval(() => {
            fetch('/status').then(r => r.json()).then(d => {
                document.getElementById('pan').textContent = d.target_pan.toFixed(2) + '°';
                document.getElementById('tilt').textContent = d.target_tilt.toFixed(2) + '°';
                document.getElementById('moving').textContent = d.moving ? 'Yes' : 'No';
                document.getElementById('moving').style.color = d.moving ? '#ff0' : '#0f0';
            });
        }, 200);
    </script>
</body>
</html>
"""


@app.route('/')
def index():
    return render_template_string(HTML,
        pan_min=PAN_MIN, pan_max=PAN_MAX,
        tilt_min=TILT_MIN, tilt_max=TILT_MAX)


@app.route('/move', methods=['POST'])
def move():
    data = request.json
    with state_lock:
        state["target_pan"] = max(PAN_MIN, min(PAN_MAX, float(data.get('pan', 0))))
        state["target_tilt"] = max(TILT_MIN, min(TILT_MAX, float(data.get('tilt', 0))))
    return jsonify({"ok": True})


@app.route('/stop', methods=['POST'])
def stop():
    if klipper:
        klipper.gcode("STOP_ALL_MOTORS")
    return jsonify({"ok": True})


@app.route('/status')
def status():
    with state_lock:
        return jsonify(state.copy())


if __name__ == "__main__":
    print("=" * 50)
    print("SMOOTH VELOCITY MODE TEST")
    print("=" * 50)
    
    connect_klipper()
    
    # Start smooth motion thread
    t = threading.Thread(target=smooth_move_thread, daemon=True)
    t.start()
    
    print("Open: http://192.168.1.136:8083/")
    print("=" * 50)
    
    app.run(host='0.0.0.0', port=8083, threaded=True, debug=False)
