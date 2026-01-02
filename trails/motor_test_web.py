#!/usr/bin/env python3
"""
Direct Motor Control Test - Web UI with draggable control
For testing motor responsiveness without the vision pipeline.

Uses direct Klipper socket for minimum latency (~1-5ms).
"""

import json
import time
import socket
import threading
from pathlib import Path
from flask import Flask, render_template_string, jsonify, request

# Try to use direct Klipper socket for lowest latency
import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))

try:
    from klipper_direct import KlipperDirect
    USE_DIRECT = True
except ImportError:
    USE_DIRECT = False

app = Flask(__name__)

# Motor state
motor_state = {
    "pan": 0.0,
    "tilt": 0.0,
    "pan_target": 0.0,
    "tilt_target": 0.0,
    "last_command_time": 0,
    "command_latency_ms": 0,
    "commands_sent": 0,
    "commands_skipped": 0,
    "connected": False,
    "moving_until": 0,  # timestamp when current move should complete
}
state_lock = threading.Lock()

# Config
config_path = Path(__file__).parent.parent / "config.json"
with open(config_path) as f:
    config = json.load(f)

PAN_MIN = config["motors"]["pan"]["min"]
PAN_MAX = config["motors"]["pan"]["max"]
TILT_MIN = config["motors"]["tilt"]["min"]
TILT_MAX = config["motors"]["tilt"]["max"]
PAN_SPEED = config["motors"]["pan"].get("speed", 60)
TILT_SPEED = config["motors"]["tilt"].get("speed", 40)

# Klipper connection
klipper = None


def connect_klipper():
    """Connect to Klipper via direct socket"""
    global klipper, motor_state
    
    socket_path = "/home/acp/printer_data/comms/klippy.sock"
    
    try:
        klipper = KlipperDirect(socket_path)
        if klipper.connect():
            print(f"[KLIPPER] Connected via direct socket: {socket_path}")
            motor_state["connected"] = True
            
            # Enable steppers
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1")
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=1")
            return True
    except Exception as e:
        print(f"[KLIPPER] Direct connection failed: {e}")
    
    # Fallback to HTTP
    print("[KLIPPER] Falling back to Moonraker HTTP API")
    motor_state["connected"] = True  # Assume connected for HTTP
    return True


def send_motor_command(pan: float, tilt: float, speed_pan: float = None, speed_tilt: float = None, force: bool = False):
    """Send motor command - STOPS current motion first for real-time control"""
    global motor_state
    
    pan = max(PAN_MIN, min(PAN_MAX, pan))
    tilt = max(TILT_MIN, min(TILT_MAX, tilt))
    
    speed_pan = speed_pan or PAN_SPEED
    speed_tilt = speed_tilt or TILT_SPEED
    
    now = time.time()
    
    start = time.perf_counter()
    
    if klipper and USE_DIRECT:
        if not force:
            # Use custom macros to stop motors + clear toolhead queue
            stop_cmd = "STOP_ALL_MOTORS\n"
            stop_cmd += "M400"  # Wait for moves to complete/clear
            klipper.gcode(stop_cmd, wait=True)  # wait=True to ensure it's processed
        
        # Re-enable and move
        cmd = "MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1\n"
        cmd += "MANUAL_STEPPER STEPPER=stepper_1 ENABLE=1\n"
        cmd += f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={pan:.3f} SPEED={speed_pan} SYNC=0\n"
        cmd += f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={tilt:.3f} SPEED={speed_tilt} SYNC=0"
        klipper.gcode(cmd, wait=False)
    else:
        # HTTP fallback
        import requests
        url = config["motors"]["moonraker_url"] + "/printer/gcode/script"
        script = ""
        if not force:
            script = "STOP_ALL_MOTORS\nM400\n"
        script += "MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1\n"
        script += "MANUAL_STEPPER STEPPER=stepper_1 ENABLE=1\n"
        script += f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={pan:.3f} SPEED={speed_pan} SYNC=0\n"
        script += f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={tilt:.3f} SPEED={speed_tilt} SYNC=0"
        requests.post(url, json={"script": script}, timeout=0.5)
    
    elapsed = (time.perf_counter() - start) * 1000
    
    with state_lock:
        motor_state["pan"] = pan
        motor_state["tilt"] = tilt
        motor_state["pan_target"] = pan
        motor_state["tilt_target"] = tilt
        motor_state["last_command_time"] = now
        motor_state["command_latency_ms"] = elapsed
        motor_state["commands_sent"] += 1
    
    return elapsed


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Motor Control Test</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { 
            background: #0a0a0a; 
            color: #fff; 
            font-family: 'Segoe UI', Arial, sans-serif;
            overflow: hidden;
            height: 100vh;
        }
        .container {
            display: grid;
            grid-template-columns: 1fr 300px;
            height: 100vh;
        }
        .control-area {
            position: relative;
            background: #111;
            border-right: 2px solid #333;
        }
        .control-pad {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            width: 80%;
            height: 80%;
            max-width: 600px;
            max-height: 600px;
            background: #1a1a1a;
            border: 2px solid #0f0;
            border-radius: 10px;
            cursor: crosshair;
        }
        .crosshair-h, .crosshair-v {
            position: absolute;
            background: #333;
        }
        .crosshair-h {
            width: 100%;
            height: 1px;
            top: 50%;
        }
        .crosshair-v {
            width: 1px;
            height: 100%;
            left: 50%;
        }
        .cursor {
            position: absolute;
            width: 40px;
            height: 40px;
            border: 3px solid #0f0;
            border-radius: 50%;
            transform: translate(-50%, -50%);
            pointer-events: none;
            transition: none;
        }
        .cursor::before, .cursor::after {
            content: '';
            position: absolute;
            background: #0f0;
        }
        .cursor::before {
            width: 20px;
            height: 3px;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
        }
        .cursor::after {
            width: 3px;
            height: 20px;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
        }
        .target {
            border-color: #ff0;
            opacity: 0.5;
        }
        .target::before, .target::after {
            background: #ff0;
        }
        .sidebar {
            padding: 20px;
            background: #0d0d0d;
            overflow-y: auto;
        }
        h1 { 
            color: #0f0; 
            font-size: 1.2em;
            margin-bottom: 20px;
            text-align: center;
        }
        .stat-group {
            background: #1a1a1a;
            border-radius: 8px;
            padding: 15px;
            margin-bottom: 15px;
        }
        .stat-group h3 {
            color: #888;
            font-size: 0.8em;
            margin-bottom: 10px;
            text-transform: uppercase;
        }
        .stat {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            font-size: 0.9em;
        }
        .stat-label { color: #888; }
        .stat-value { color: #0f0; font-family: monospace; }
        .stat-value.warn { color: #ff0; }
        .stat-value.bad { color: #f00; }
        .latency-bar {
            height: 8px;
            background: #333;
            border-radius: 4px;
            margin-top: 5px;
            overflow: hidden;
        }
        .latency-fill {
            height: 100%;
            background: #0f0;
            transition: width 0.1s;
        }
        .btn {
            width: 100%;
            padding: 12px;
            border: none;
            border-radius: 6px;
            font-size: 1em;
            cursor: pointer;
            margin-bottom: 10px;
            transition: all 0.2s;
        }
        .btn-center {
            background: #0a4;
            color: #fff;
        }
        .btn-center:hover { background: #0c6; }
        .btn-stop {
            background: #a00;
            color: #fff;
        }
        .btn-stop:hover { background: #c00; }
        .speed-control {
            margin-top: 15px;
        }
        .speed-control label {
            display: block;
            color: #888;
            font-size: 0.8em;
            margin-bottom: 5px;
        }
        .speed-control input[type="range"] {
            width: 100%;
        }
        .mode-toggle {
            display: flex;
            gap: 5px;
            margin-bottom: 15px;
        }
        .mode-btn {
            flex: 1;
            padding: 8px;
            border: 1px solid #333;
            background: #1a1a1a;
            color: #888;
            cursor: pointer;
            border-radius: 4px;
        }
        .mode-btn.active {
            border-color: #0f0;
            color: #0f0;
        }
        .position-display {
            font-family: monospace;
            font-size: 1.5em;
            text-align: center;
            padding: 15px;
            background: #000;
            border-radius: 8px;
            margin-bottom: 15px;
        }
        .pos-pan { color: #0ff; }
        .pos-tilt { color: #f0f; }
        .axis-labels {
            position: absolute;
            color: #666;
            font-size: 0.8em;
        }
        .label-left { left: 10px; top: 50%; transform: translateY(-50%); }
        .label-right { right: 10px; top: 50%; transform: translateY(-50%); }
        .label-top { top: 10px; left: 50%; transform: translateX(-50%); }
        .label-bottom { bottom: 10px; left: 50%; transform: translateX(-50%); }
        .connection-status {
            display: flex;
            align-items: center;
            gap: 8px;
            margin-bottom: 15px;
        }
        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #0f0;
        }
        .status-dot.disconnected { background: #f00; }
    </style>
</head>
<body>
    <div class="container">
        <div class="control-area">
            <div class="control-pad" id="pad">
                <div class="crosshair-h"></div>
                <div class="crosshair-v"></div>
                <div class="axis-labels label-left">PAN -</div>
                <div class="axis-labels label-right">PAN +</div>
                <div class="axis-labels label-top">TILT +</div>
                <div class="axis-labels label-bottom">TILT -</div>
                <div class="cursor target" id="target"></div>
                <div class="cursor" id="cursor"></div>
            </div>
        </div>
        <div class="sidebar">
            <h1>⚡ MOTOR CONTROL TEST</h1>
            
            <div class="connection-status">
                <div class="status-dot" id="statusDot"></div>
                <span id="statusText">Connected</span>
            </div>
            
            <div class="position-display">
                <span class="pos-pan">PAN: <span id="panVal">0.00</span>°</span><br>
                <span class="pos-tilt">TILT: <span id="tiltVal">0.00</span>°</span>
            </div>
            
            <div class="stat-group">
                <h3>Latency</h3>
                <div class="stat">
                    <span class="stat-label">Command</span>
                    <span class="stat-value" id="cmdLatency">0.0 ms</span>
                </div>
                <div class="latency-bar">
                    <div class="latency-fill" id="latencyBar"></div>
                </div>
                <div class="stat">
                    <span class="stat-label">Commands sent</span>
                    <span class="stat-value" id="cmdTotal">0</span>
                </div>
                <div class="stat">
                    <span class="stat-label">Commands skipped</span>
                    <span class="stat-value" id="cmdSkipped">0</span>
                </div>
                <div class="stat">
                    <span class="stat-label">Queue mode</span>
                    <span class="stat-value" id="queueMode">NO QUEUE</span>
                </div>
            </div>
            
            <div class="stat-group">
                <h3>Limits</h3>
                <div class="stat">
                    <span class="stat-label">Pan</span>
                    <span class="stat-value">{{ pan_min }}° to {{ pan_max }}°</span>
                </div>
                <div class="stat">
                    <span class="stat-label">Tilt</span>
                    <span class="stat-value">{{ tilt_min }}° to {{ tilt_max }}°</span>
                </div>
            </div>
            
            <div class="mode-toggle">
                <button class="mode-btn active" id="modeStream" onclick="setMode('stream')">Stream</button>
                <button class="mode-btn" id="modeClick" onclick="setMode('click')">Click</button>
            </div>
            
            <div class="mode-toggle">
                <button class="mode-btn active" id="modeNoQueue" onclick="setQueueMode(false)">Real-time ⚡ (M410)</button>
                <button class="mode-btn" id="modeQueue" onclick="setQueueMode(true)">Queue (old)</button>
            </div>
            
            <div class="speed-control">
                <label>Pan Speed: <span id="panSpeedVal">{{ pan_speed }}</span> °/s</label>
                <input type="range" id="panSpeed" min="10" max="150" value="{{ pan_speed }}" oninput="updateSpeed()">
            </div>
            <div class="speed-control">
                <label>Tilt Speed: <span id="tiltSpeedVal">{{ tilt_speed }}</span> °/s</label>
                <input type="range" id="tiltSpeed" min="10" max="100" value="{{ tilt_speed }}" oninput="updateSpeed()">
            </div>
            
            <button class="btn btn-center" onclick="centerMotors()">⟲ CENTER</button>
            <button class="btn btn-stop" onclick="stopMotors()">⬛ STOP</button>
        </div>
    </div>
    
    <script>
        const pad = document.getElementById('pad');
        const cursor = document.getElementById('cursor');
        const target = document.getElementById('target');
        
        let mode = 'stream';  // 'stream' or 'click'
        let queueMode = false;  // false = no queue (skip commands while moving)
        let isDragging = false;
        let lastSendTime = 0;
        let sendInterval = 20;  // ms between commands in stream mode
        let commandCount = 0;
        let commandCountStart = Date.now();
        
        const panMin = {{ pan_min }};
        const panMax = {{ pan_max }};
        const tiltMin = {{ tilt_min }};
        const tiltMax = {{ tilt_max }};
        
        // Position cursor at center initially
        cursor.style.left = '50%';
        cursor.style.top = '50%';
        target.style.left = '50%';
        target.style.top = '50%';
        
        function setMode(m) {
            mode = m;
            document.getElementById('modeStream').classList.toggle('active', m === 'stream');
            document.getElementById('modeClick').classList.toggle('active', m === 'click');
        }
        
        function setQueueMode(q) {
            queueMode = q;
            document.getElementById('modeNoQueue').classList.toggle('active', !q);
            document.getElementById('modeQueue').classList.toggle('active', q);
            document.getElementById('queueMode').textContent = q ? 'QUEUE ALL' : 'NO QUEUE';
            document.getElementById('queueMode').style.color = q ? '#f00' : '#0f0';
        }
        
        function updateSpeed() {
            document.getElementById('panSpeedVal').textContent = document.getElementById('panSpeed').value;
            document.getElementById('tiltSpeedVal').textContent = document.getElementById('tiltSpeed').value;
        }
        
        function coordsToPosition(x, y) {
            const rect = pad.getBoundingClientRect();
            const relX = (x - rect.left) / rect.width;
            const relY = (y - rect.top) / rect.height;
            
            // Map to motor range (center is 0)
            const pan = (relX - 0.5) * 2 * Math.max(Math.abs(panMin), Math.abs(panMax));
            const tilt = (0.5 - relY) * 2 * Math.max(Math.abs(tiltMin), Math.abs(tiltMax));
            
            return {
                pan: Math.max(panMin, Math.min(panMax, pan)),
                tilt: Math.max(tiltMin, Math.min(tiltMax, tilt)),
                relX: relX,
                relY: relY
            };
        }
        
        function sendCommand(pan, tilt) {
            const now = Date.now();
            if (mode === 'stream' && now - lastSendTime < sendInterval) {
                return;
            }
            lastSendTime = now;
            commandCount++;
            
            const panSpeed = parseInt(document.getElementById('panSpeed').value);
            const tiltSpeed = parseInt(document.getElementById('tiltSpeed').value);
            
            fetch('/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({pan, tilt, pan_speed: panSpeed, tilt_speed: tiltSpeed, force: queueMode})
            }).then(r => r.json()).then(data => {
                updateStats(data);
            });
        }
        
        function updateStats(data) {
            document.getElementById('panVal').textContent = data.pan.toFixed(2);
            document.getElementById('tiltVal').textContent = data.tilt.toFixed(2);
            document.getElementById('cmdLatency').textContent = data.latency_ms.toFixed(1) + ' ms';
            document.getElementById('cmdTotal').textContent = data.commands_sent;
            document.getElementById('cmdSkipped').textContent = data.commands_skipped || 0;
            
            // Latency bar (scale: 0-20ms)
            const latencyPct = Math.min(100, (data.latency_ms / 20) * 100);
            const bar = document.getElementById('latencyBar');
            bar.style.width = latencyPct + '%';
            bar.style.background = data.latency_ms < 5 ? '#0f0' : data.latency_ms < 10 ? '#ff0' : '#f00';
            
            // Update latency text color
            const latencyEl = document.getElementById('cmdLatency');
            latencyEl.className = 'stat-value' + (data.latency_ms < 5 ? '' : data.latency_ms < 10 ? ' warn' : ' bad');
            
            // Update cursor position to match actual motor position
            const panRange = Math.max(Math.abs(panMin), Math.abs(panMax));
            const tiltRange = Math.max(Math.abs(tiltMin), Math.abs(tiltMax));
            const cursorX = 50 + (data.pan / panRange) * 50;
            const cursorY = 50 - (data.tilt / tiltRange) * 50;
            cursor.style.left = cursorX + '%';
            cursor.style.top = cursorY + '%';
        }
        
        function handleMove(x, y) {
            const pos = coordsToPosition(x, y);
            
            // Update target cursor
            target.style.left = (pos.relX * 100) + '%';
            target.style.top = (pos.relY * 100) + '%';
            
            sendCommand(pos.pan, pos.tilt);
        }
        
        // Mouse events
        pad.addEventListener('mousedown', (e) => {
            isDragging = true;
            handleMove(e.clientX, e.clientY);
        });
        
        document.addEventListener('mousemove', (e) => {
            if (isDragging || mode === 'stream') {
                if (e.target === pad || pad.contains(e.target) || isDragging) {
                    handleMove(e.clientX, e.clientY);
                }
            }
        });
        
        document.addEventListener('mouseup', () => {
            isDragging = false;
        });
        
        // Touch events
        pad.addEventListener('touchstart', (e) => {
            e.preventDefault();
            isDragging = true;
            handleMove(e.touches[0].clientX, e.touches[0].clientY);
        });
        
        document.addEventListener('touchmove', (e) => {
            if (isDragging) {
                handleMove(e.touches[0].clientX, e.touches[0].clientY);
            }
        });
        
        document.addEventListener('touchend', () => {
            isDragging = false;
        });
        
        function centerMotors() {
            sendCommand(0, 0);
            target.style.left = '50%';
            target.style.top = '50%';
        }
        
        function stopMotors() {
            fetch('/stop', {method: 'POST'});
        }
        
        // Update command rate every second
        setInterval(() => {
            const elapsed = (Date.now() - commandCountStart) / 1000;
            const rate = commandCount / elapsed;
            document.getElementById('cmdRate').textContent = rate.toFixed(1);
            
            // Reset counter every 5 seconds
            if (elapsed > 5) {
                commandCount = 0;
                commandCountStart = Date.now();
            }
        }, 500);
        
        // Poll status
        setInterval(() => {
            fetch('/status').then(r => r.json()).then(data => {
                const dot = document.getElementById('statusDot');
                const text = document.getElementById('statusText');
                dot.classList.toggle('disconnected', !data.connected);
                text.textContent = data.connected ? 'Connected' : 'Disconnected';
            });
        }, 2000);
    </script>
</body>
</html>
"""


@app.route('/')
def index():
    return render_template_string(HTML_PAGE,
        pan_min=PAN_MIN, pan_max=PAN_MAX,
        tilt_min=TILT_MIN, tilt_max=TILT_MAX,
        pan_speed=PAN_SPEED, tilt_speed=TILT_SPEED
    )


@app.route('/move', methods=['POST'])
def move():
    data = request.json
    pan = float(data.get('pan', 0))
    tilt = float(data.get('tilt', 0))
    pan_speed = float(data.get('pan_speed', PAN_SPEED))
    tilt_speed = float(data.get('tilt_speed', TILT_SPEED))
    force = bool(data.get('force', False))  # force = queue mode (don't skip)
    
    latency = send_motor_command(pan, tilt, pan_speed, tilt_speed, force=force)
    
    with state_lock:
        return jsonify({
            "pan": motor_state["pan"],
            "tilt": motor_state["tilt"],
            "latency_ms": latency,
            "commands_sent": motor_state["commands_sent"],
            "commands_skipped": motor_state["commands_skipped"],
        })


@app.route('/stop', methods=['POST'])
def stop():
    if klipper:
        klipper.gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=0")
        klipper.gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=0")
    return jsonify({"status": "stopped"})


@app.route('/status')
def status():
    with state_lock:
        return jsonify(motor_state)


if __name__ == "__main__":
    print("=" * 60)
    print("⚡ DIRECT MOTOR CONTROL TEST")
    print("=" * 60)
    print(f"Pan range: {PAN_MIN}° to {PAN_MAX}°")
    print(f"Tilt range: {TILT_MIN}° to {TILT_MAX}°")
    print()
    
    connect_klipper()
    
    print()
    print("Open in browser: http://192.168.1.136:8081/")
    print("=" * 60)
    
    app.run(host='0.0.0.0', port=8081, threaded=True, debug=False)
