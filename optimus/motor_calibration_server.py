#!/usr/bin/env python3
"""
Motor Calibration Web Server
Provides a web interface for calibrating motor positions and limits
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from flask import Flask, render_template_string, request, jsonify
import threading
import json
import os

app = Flask(__name__)

# Global state
calibration_state = {
    'current_motor': 'stepper_0',
    'motors': {
        'stepper_0': {'position': 0.0, 'center': None, 'min': None, 'max': None},
        'stepper_1': {'position': 0.0, 'center': None, 'min': None, 'max': None},
        'stepper_2': {'position': 0.0, 'center': None, 'min': None, 'max': None},
        'stepper_3': {'position': 0.0, 'center': None, 'min': None, 'max': None},
    },
    'step': 'idle',  # idle, find_center, find_min, find_max, complete
    'instructions': 'Select a motor to start calibration'
}

ros_node = None

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Motor Calibration</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 800px;
            margin: 50px auto;
            padding: 20px;
            background: #f0f0f0;
        }
        .container {
            background: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #333;
            text-align: center;
        }
        .motor-select {
            margin: 20px 0;
            text-align: center;
        }
        .motor-select button {
            margin: 5px;
            padding: 10px 20px;
            font-size: 16px;
            cursor: pointer;
            border: 2px solid #007bff;
            background: white;
            border-radius: 5px;
        }
        .motor-select button.active {
            background: #007bff;
            color: white;
        }
        .instructions {
            background: #e7f3ff;
            padding: 15px;
            border-left: 4px solid #007bff;
            margin: 20px 0;
            font-size: 18px;
            font-weight: bold;
        }
        .slider-container {
            margin: 30px 0;
        }
        .slider {
            width: 100%;
            height: 40px;
        }
        .position-display {
            text-align: center;
            font-size: 24px;
            margin: 10px 0;
            font-weight: bold;
        }
        .button-group {
            text-align: center;
            margin: 20px 0;
        }
        .button-group button {
            margin: 10px;
            padding: 15px 30px;
            font-size: 18px;
            cursor: pointer;
            border: none;
            border-radius: 5px;
            color: white;
        }
        .btn-primary {
            background: #007bff;
        }
        .btn-success {
            background: #28a745;
        }
        .btn-warning {
            background: #ffc107;
            color: #333;
        }
        .btn-danger {
            background: #dc3545;
        }
        .calibration-data {
            margin: 20px 0;
            padding: 15px;
            background: #f8f9fa;
            border-radius: 5px;
        }
        .calibration-data h3 {
            margin-top: 0;
        }
        .motor-status {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
            margin-top: 10px;
        }
        .motor-status div {
            padding: 10px;
            background: white;
            border-radius: 3px;
            border: 1px solid #ddd;
        }
        .complete {
            color: green;
        }
        .incomplete {
            color: orange;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Motor Calibration Tool</h1>
        
        <div class="motor-select">
            <button onclick="selectMotor('stepper_0')" id="btn_stepper_0">Motor 0</button>
            <button onclick="selectMotor('stepper_1')" id="btn_stepper_1">Motor 1</button>
            <button onclick="selectMotor('stepper_2')" id="btn_stepper_2">Motor 2</button>
            <button onclick="selectMotor('stepper_3')" id="btn_stepper_3">Motor 3</button>
        </div>

        <div class="instructions" id="instructions">
            Select a motor to start calibration
        </div>

        <div class="slider-container">
            <div class="position-display">
                Position: <span id="position">0.0</span>°
            </div>
            <input type="range" min="-180" max="180" value="0" step="0.5" 
                   class="slider" id="positionSlider" oninput="updatePosition(this.value)">
        </div>

        <div class="button-group" id="calibration-buttons" style="display:none;">
            <button class="btn-primary" onclick="startCalibration()">Start Calibration</button>
            <button class="btn-success" onclick="recordPosition()">Record Position</button>
            <button class="btn-warning" onclick="moveToCenter()">Move to Center</button>
            <button class="btn-danger" onclick="resetMotor()">Reset Motor</button>
        </div>

        <div class="calibration-data">
            <h3>Calibration Data</h3>
            <div class="motor-status" id="motorStatus"></div>
        </div>

        <div class="button-group">
            <button class="btn-success" onclick="saveCalibration()">Save All Calibration Data</button>
        </div>
    </div>

    <script>
        let currentMotor = 'stepper_0';
        let state = {{ state | tojson }};

        function updateUI() {
            fetch('/state')
                .then(r => r.json())
                .then(data => {
                    state = data;
                    document.getElementById('instructions').textContent = state.instructions;
                    
                    // Update motor status
                    let statusHtml = '';
                    for (let motor in state.motors) {
                        let m = state.motors[motor];
                        let complete = m.center !== null && m.min !== null && m.max !== null;
                        let status = complete ? '✓ Complete' : '○ Incomplete';
                        let className = complete ? 'complete' : 'incomplete';
                        statusHtml += `
                            <div class="${className}">
                                <strong>${motor}</strong><br>
                                Center: ${m.center !== null ? m.center.toFixed(1) : '---'}°<br>
                                Min: ${m.min !== null ? m.min.toFixed(1) : '---'}°<br>
                                Max: ${m.max !== null ? m.max.toFixed(1) : '---'}°<br>
                                ${status}
                            </div>
                        `;
                    }
                    document.getElementById('motorStatus').innerHTML = statusHtml;
                    
                    // Update button states
                    document.querySelectorAll('.motor-select button').forEach(btn => {
                        btn.classList.remove('active');
                    });
                    document.getElementById('btn_' + state.current_motor).classList.add('active');
                });
        }

        function selectMotor(motor) {
            currentMotor = motor;
            fetch('/select_motor', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({motor: motor})
            }).then(() => {
                updateUI();
                document.getElementById('calibration-buttons').style.display = 'block';
            });
        }

        function updatePosition(value) {
            document.getElementById('position').textContent = parseFloat(value).toFixed(1);
            fetch('/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({motor: currentMotor, position: parseFloat(value)})
            });
        }

        function startCalibration() {
            fetch('/start_calibration', {method: 'POST'})
                .then(() => updateUI());
        }

        function recordPosition() {
            let position = parseFloat(document.getElementById('positionSlider').value);
            fetch('/record', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({motor: currentMotor, position: position})
            }).then(() => updateUI());
        }

        function moveToCenter() {
            fetch('/move_center', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({motor: currentMotor})
            }).then(r => r.json()).then(data => {
                if (data.position !== undefined) {
                    document.getElementById('positionSlider').value = data.position;
                    document.getElementById('position').textContent = data.position.toFixed(1);
                }
                updateUI();
            });
        }

        function resetMotor() {
            if (confirm('Reset calibration for ' + currentMotor + '?')) {
                fetch('/reset', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({motor: currentMotor})
                }).then(() => updateUI());
            }
        }

        function saveCalibration() {
            fetch('/save', {method: 'POST'})
                .then(r => r.json())
                .then(data => {
                    alert('Calibration saved to: ' + data.file);
                });
        }

        // Update UI every 2 seconds
        setInterval(updateUI, 2000);
        updateUI();
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE, state=calibration_state)

@app.route('/state')
def get_state():
    return jsonify(calibration_state)

@app.route('/select_motor', methods=['POST'])
def select_motor():
    data = request.json
    motor = data.get('motor')
    calibration_state['current_motor'] = motor
    calibration_state['step'] = 'idle'
    calibration_state['instructions'] = f'Selected {motor}. Click "Start Calibration" to begin.'
    return jsonify({'success': True})

@app.route('/move', methods=['POST'])
def move_motor():
    data = request.json
    motor = data.get('motor')
    position = data.get('position')
    
    # Publish to ROS topic
    if ros_node:
        msg = Float32()
        msg.data = float(position)
        ros_node.motor_publishers[motor].publish(msg)
    
    calibration_state['motors'][motor]['position'] = position
    return jsonify({'success': True})

@app.route('/start_calibration', methods=['POST'])
def start_calibration():
    motor = calibration_state['current_motor']
    calibration_state['step'] = 'find_center'
    calibration_state['instructions'] = f'{motor}: Use the slider to move the motor to its CENTER position, then click "Record Position"'
    return jsonify({'success': True})

@app.route('/record', methods=['POST'])
def record_position():
    data = request.json
    motor = data.get('motor')
    position = data.get('position')
    
    step = calibration_state['step']
    
    if step == 'find_center':
        calibration_state['motors'][motor]['center'] = position
        calibration_state['step'] = 'find_min'
        calibration_state['instructions'] = f'{motor}: Now move to MINIMUM position (most counter-clockwise), then click "Record Position"'
    
    elif step == 'find_min':
        calibration_state['motors'][motor]['min'] = position
        calibration_state['step'] = 'find_max'
        calibration_state['instructions'] = f'{motor}: Click "Move to Center" to return, then move to MAXIMUM position (most clockwise), then click "Record Position"'
    
    elif step == 'find_max':
        calibration_state['motors'][motor]['max'] = position
        calibration_state['step'] = 'complete'
        calibration_state['instructions'] = f'{motor}: Calibration complete! Center={calibration_state["motors"][motor]["center"]:.1f}°, Range=[{calibration_state["motors"][motor]["min"]:.1f}° to {calibration_state["motors"][motor]["max"]:.1f}°]'
    
    return jsonify({'success': True})

@app.route('/move_center', methods=['POST'])
def move_center():
    data = request.json
    motor = data.get('motor')
    
    center = calibration_state['motors'][motor].get('center')
    if center is not None:
        # Move motor to center
        if ros_node:
            msg = Float32()
            msg.data = float(center)
            ros_node.motor_publishers[motor].publish(msg)
        
        calibration_state['motors'][motor]['position'] = center
        return jsonify({'success': True, 'position': center})
    
    return jsonify({'success': False, 'error': 'Center not set'})

@app.route('/reset', methods=['POST'])
def reset_motor():
    data = request.json
    motor = data.get('motor')
    
    calibration_state['motors'][motor] = {
        'position': 0.0,
        'center': None,
        'min': None,
        'max': None
    }
    calibration_state['step'] = 'idle'
    calibration_state['instructions'] = f'{motor} reset. Click "Start Calibration" to begin.'
    
    return jsonify({'success': True})

@app.route('/save', methods=['POST'])
def save_calibration():
    output_file = '/home/acp/motor_calibration.json'
    
    with open(output_file, 'w') as f:
        json.dump(calibration_state['motors'], f, indent=2)
    
    return jsonify({'success': True, 'file': output_file})


class MotorCalibrationNode(Node):
    def __init__(self):
        super().__init__('motor_calibration_server')
        
        # Create publishers for each motor
        self.motor_publishers = {
            'stepper_0': self.create_publisher(Float32, '/motor/stepper_0/position', 10),
            'stepper_1': self.create_publisher(Float32, '/motor/stepper_1/position', 10),
            'stepper_2': self.create_publisher(Float32, '/motor/stepper_2/position', 10),
            'stepper_3': self.create_publisher(Float32, '/motor/stepper_3/position', 10),
        }
        
        self.get_logger().info('Motor Calibration Node Started')


def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


def main(args=None):
    global ros_node
    
    rclpy.init(args=args)
    ros_node = MotorCalibrationNode()
    
    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    ros_node.get_logger().info('Motor Calibration Web Interface: http://localhost:5000')
    print('\n' + '='*60)
    print('Motor Calibration Web Interface Ready!')
    print('Open in browser: http://localhost:5000')
    print('='*60 + '\n')
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
