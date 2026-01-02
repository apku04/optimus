#!/usr/bin/env python3
"""
Motor control test with SYNC moves - waits for each move to complete.
This prevents queue buildup in Klipper.
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
    USE_DIRECT = True
except ImportError:
    USE_DIRECT = False

app = Flask(__name__)

# Config
config_path = Path(__file__).parent.parent / "config.json"
with open(config_path) as f:
    config = json.load(f)

PAN_MIN = config["motors"]["pan"]["min"]
PAN_MAX = config["motors"]["pan"]["max"]
TILT_MIN = config["motors"]["tilt"]["min"]
TILT_MAX = config["motors"]["tilt"]["max"]

# Tuning parameters - BALANCED: smooth but responsive
PAN_SPEED = 60     # degrees/sec 
TILT_SPEED = 50    # degrees/sec
ACCEL = 200        # degrees/sec² - gentle but not too slow

# Smoothing - interpolate to target
SMOOTHING = 0.6    # Balanced smoothing
DEADZONE = 0.4     # Moderate deadzone
UPDATE_RATE = 40   # Hz
MIN_MOVE = 0.2     # Smaller minimum move

# State
state = {
    "pan": 0.0,
    "tilt": 0.0,
    "target_pan": 0.0,
    "target_tilt": 0.0,
    "busy": False,
    "commands_sent": 0,
    "latency_ms": 0,
}
state_lock = threading.Lock()

# Klipper connection
klipper = None


def connect_klipper():
    global klipper
    socket_path = "/home/acp/printer_data/comms/klippy.sock"
    try:
        klipper = KlipperDirect(socket_path)
        if klipper.connect():
            print(f"[KLIPPER] Connected: {socket_path}")
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1")
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=1")
            return True
    except Exception as e:
        print(f"[KLIPPER] Error: {e}")
    return False


def move_sync(pan: float, tilt: float):
    """Move and wait for completion - no queue buildup"""
    global state
    
    pan = max(PAN_MIN, min(PAN_MAX, pan))
    tilt = max(TILT_MIN, min(TILT_MAX, tilt))
    
    with state_lock:
        if state["busy"]:
            # Update target but don't send - motor thread will pick it up
            state["target_pan"] = pan
            state["target_tilt"] = tilt
            return
        state["busy"] = True
        state["target_pan"] = pan
        state["target_tilt"] = tilt
    
    start = time.perf_counter()
    
    # Send move with ACCEL for snappy response, SYNC=1 waits for completion
    cmd = f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={pan:.3f} SPEED={PAN_SPEED} ACCEL={ACCEL} SYNC=0\n"
    cmd += f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={tilt:.3f} SPEED={TILT_SPEED} ACCEL={ACCEL} SYNC=1"
    
    if klipper:
        klipper.gcode(cmd, wait=True)  # Wait for response
    
    elapsed = (time.perf_counter() - start) * 1000
    
    with state_lock:
        state["pan"] = pan
        state["tilt"] = tilt
        state["busy"] = False
        state["commands_sent"] += 1
        state["latency_ms"] = elapsed * 0.1 + state["latency_ms"] * 0.9  # Smooth avg


# Motor control thread - processes targets continuously
def motor_thread():
    """Continuously move to latest target with smoothing"""
    current_pan, current_tilt = 0.0, 0.0
    
    while True:
        with state_lock:
            target_pan = state["target_pan"]
            target_tilt = state["target_tilt"]
            busy = state["busy"]
        
        # Apply smoothing - interpolate towards target
        if SMOOTHING > 0:
            smooth_pan = current_pan + (target_pan - current_pan) * (1 - SMOOTHING)
            smooth_tilt = current_tilt + (target_tilt - current_tilt) * (1 - SMOOTHING)
        else:
            smooth_pan, smooth_tilt = target_pan, target_tilt
        
        # Calculate distance to move
        delta_pan = abs(smooth_pan - current_pan)
        delta_tilt = abs(smooth_tilt - current_tilt)
        total_delta = delta_pan + delta_tilt
        
        # Only move if change is significant (prevents vibration)
        if not busy and total_delta > DEADZONE:
            # Skip tiny movements that cause vibration
            if delta_pan < MIN_MOVE:
                smooth_pan = current_pan
            if delta_tilt < MIN_MOVE:
                smooth_tilt = current_tilt
            
            # Only send if we actually have something to move
            if abs(smooth_pan - current_pan) > 0.01 or abs(smooth_tilt - current_tilt) > 0.01:
                move_sync(smooth_pan, smooth_tilt)
                current_pan, current_tilt = smooth_pan, smooth_tilt
        
        time.sleep(1.0 / UPDATE_RATE)


HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Sync Motor Test</title>
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
            border: 2px solid #0f0;
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
            border: 3px solid #0f0;
            border-radius: 50%;
            transform: translate(-50%, -50%);
            pointer-events: none;
        }
        .sidebar {
            width: 250px;
            padding: 20px;
            background: #0d0d0d;
        }
        h1 { color: #0f0; font-size: 1.2em; margin-bottom: 20px; }
        .stat { 
            display: flex; 
            justify-content: space-between; 
            margin: 10px 0;
            padding: 8px;
            background: #1a1a1a;
            border-radius: 4px;
        }
        .stat-value { color: #0f0; font-family: monospace; }
        .btn {
            width: 100%;
            padding: 15px;
            margin: 10px 0;
            border: none;
            border-radius: 6px;
            font-size: 1em;
            cursor: pointer;
        }
        .btn-center { background: #0a4; color: #fff; }
        .btn-stop { background: #a00; color: #fff; }
        .info { 
            font-size: 0.8em; 
            color: #888; 
            margin-top: 20px;
            padding: 10px;
            background: #1a1a1a;
            border-radius: 4px;
        }
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
        <h1>⚡ SYNC MODE TEST</h1>
        
        <div class="stat">
            <span>Pan</span>
            <span class="stat-value" id="pan">0.00°</span>
        </div>
        <div class="stat">
            <span>Tilt</span>
            <span class="stat-value" id="tilt">0.00°</span>
        </div>
        <div class="stat">
            <span>Move Time</span>
            <span class="stat-value" id="latency">0 ms</span>
        </div>
        <div class="stat">
            <span>Moves</span>
            <span class="stat-value" id="moves">0</span>
        </div>
        <div class="stat">
            <span>Busy</span>
            <span class="stat-value" id="busy">No</span>
        </div>
        
        <button class="btn btn-center" onclick="center()">⟲ CENTER</button>
        <button class="btn btn-stop" onclick="stop()">⬛ STOP</button>
        
        <div class="info">
            <strong>SYNC Mode:</strong><br>
            Each move waits for completion before sending next.
            No queue buildup - motor goes to latest target only.
        </div>
    </div>

    <script>
        const pad = document.getElementById('pad');
        const cursor = document.getElementById('cursor');
        let isDragging = false;
        
        const panMin = {{ pan_min }}, panMax = {{ pan_max }};
        const tiltMin = {{ tilt_min }}, tiltMax = {{ tilt_max }};
        
        cursor.style.left = '50%';
        cursor.style.top = '50%';
        
        function sendMove(x, y) {
            const rect = pad.getBoundingClientRect();
            const relX = (x - rect.left) / rect.width;
            const relY = (y - rect.top) / rect.height;
            
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
        
        function center() { fetch('/move', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({pan: 0, tilt: 0})}); cursor.style.left = '50%'; cursor.style.top = '50%'; }
        function stop() { fetch('/stop', {method: 'POST'}); }
        
        setInterval(() => {
            fetch('/status').then(r => r.json()).then(d => {
                document.getElementById('pan').textContent = d.pan.toFixed(2) + '°';
                document.getElementById('tilt').textContent = d.tilt.toFixed(2) + '°';
                document.getElementById('latency').textContent = d.latency_ms.toFixed(0) + ' ms';
                document.getElementById('moves').textContent = d.commands_sent;
                document.getElementById('busy').textContent = d.busy ? 'Yes' : 'No';
                document.getElementById('busy').style.color = d.busy ? '#ff0' : '#0f0';
            });
        }, 100);
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
        state["target_pan"] = float(data.get('pan', 0))
        state["target_tilt"] = float(data.get('tilt', 0))
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
    print("SYNC MODE MOTOR TEST")
    print("=" * 50)
    
    connect_klipper()
    
    # Start motor control thread
    t = threading.Thread(target=motor_thread, daemon=True)
    t.start()
    
    print("Open: http://192.168.1.136:8082/")
    print("=" * 50)
    
    app.run(host='0.0.0.0', port=8082, threaded=True, debug=False)
