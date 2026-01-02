#!/usr/bin/env python3
"""
Direct Klipper communication via Unix socket.

Bypasses Moonraker HTTP API entirely for minimal latency (~1-5ms vs 10-50ms).
Connects directly to Klipper's Unix domain socket and sends G-code commands.
"""

import socket
import json
import time
import threading
from typing import Optional, Tuple, Dict, Any
from queue import Queue, Empty


class KlipperDirect:
    """
    Direct connection to Klipper via Unix socket for low-latency motor control.
    
    This bypasses Moonraker and talks directly to Klipper for minimal command latency.
    """
    
    def __init__(
        self,
        socket_path: str = "/home/acp/printer_data/comms/klippy.sock",
        timeout: float = 2.0,
    ):
        self.socket_path = socket_path
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._response_queue: Queue = Queue()
        self._reader_thread: Optional[threading.Thread] = None
        self._running = False
        self._request_id = 0
        self._pending_requests: Dict[int, Any] = {}
        
    def connect(self) -> bool:
        """Connect to Klipper Unix socket."""
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect(self.socket_path)
            self._running = True
            
            # Start reader thread
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()
            
            # Subscribe to get responses
            self._send_request("info", {})
            time.sleep(0.1)  # Wait for connection to stabilize
            
            print(f"[KLIPPER] Connected to {self.socket_path}")
            return True
            
        except Exception as e:
            print(f"[KLIPPER] Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Klipper."""
        self._running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
    
    def _reader_loop(self):
        """Background thread to read responses from Klipper."""
        buffer = b""
        while self._running and self.sock:
            try:
                data = self.sock.recv(4096)
                if not data:
                    break
                buffer += data
                
                # Process complete JSON messages (newline-delimited)
                while b'\x03' in buffer:
                    msg, buffer = buffer.split(b'\x03', 1)
                    try:
                        response = json.loads(msg.decode('utf-8'))
                        req_id = response.get('id')
                        if req_id is not None and req_id in self._pending_requests:
                            self._pending_requests[req_id] = response
                    except json.JSONDecodeError:
                        pass
                        
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"[KLIPPER] Reader error: {e}")
                break
    
    def _send_request(self, method: str, params: dict) -> int:
        """Send a JSON-RPC request to Klipper."""
        with self._lock:
            self._request_id += 1
            req_id = self._request_id
            
            request = {
                "id": req_id,
                "method": method,
                "params": params
            }
            
            self._pending_requests[req_id] = None
            msg = json.dumps(request) + '\x03'
            
            try:
                self.sock.sendall(msg.encode('utf-8'))
            except Exception as e:
                print(f"[KLIPPER] Send error: {e}")
                
            return req_id
    
    def _wait_response(self, req_id: int, timeout: float = 1.0) -> Optional[dict]:
        """Wait for a response to a specific request."""
        start = time.time()
        while time.time() - start < timeout:
            if self._pending_requests.get(req_id) is not None:
                response = self._pending_requests.pop(req_id)
                return response
            time.sleep(0.001)
        self._pending_requests.pop(req_id, None)
        return None
    
    def gcode(self, script: str, wait: bool = False) -> bool:
        """
        Send G-code command to Klipper.
        
        Args:
            script: G-code command(s) to execute
            wait: If True, wait for response (slower but confirms execution)
            
        Returns:
            True if command was sent successfully
        """
        req_id = self._send_request("gcode/script", {"script": script})
        
        if wait:
            response = self._wait_response(req_id, timeout=self.timeout)
            return response is not None and "error" not in response
        return True
    
    def move_stepper(
        self,
        stepper: str,
        move: float,
        speed: Optional[float] = None,
        accel: Optional[float] = None,
        sync: bool = True,
    ) -> bool:
        """
        Move a manual stepper.
        
        Args:
            stepper: Stepper name (e.g., "stepper_0")
            move: Target position in mm/degrees
            speed: Movement speed (optional, uses config default)
            accel: Acceleration (optional, uses config default)
            sync: If True, command includes SYNC=1
            
        Returns:
            True if command was sent
        """
        cmd = f"MANUAL_STEPPER STEPPER={stepper} MOVE={move:.4f}"
        if speed is not None:
            cmd += f" SPEED={speed:.2f}"
        if accel is not None:
            cmd += f" ACCEL={accel:.2f}"
        if sync:
            cmd += " SYNC=1"
        
        return self.gcode(cmd, wait=False)
    
    def set_position(self, stepper: str, position: float) -> bool:
        """Set the current position of a stepper without moving."""
        cmd = f"MANUAL_STEPPER STEPPER={stepper} SET_POSITION={position:.4f}"
        return self.gcode(cmd, wait=False)
    
    def enable_stepper(self, stepper: str, enable: bool = True) -> bool:
        """Enable or disable a stepper."""
        cmd = f"MANUAL_STEPPER STEPPER={stepper} ENABLE={'1' if enable else '0'}"
        return self.gcode(cmd, wait=False)
    
    def move_pan_tilt(
        self,
        pan_pos: float,
        tilt_pos: float,
        pan_speed: Optional[float] = None,
        tilt_speed: Optional[float] = None,
    ) -> Tuple[bool, bool]:
        """
        Move both pan and tilt steppers.
        
        Sends both commands in rapid succession for near-simultaneous movement.
        """
        # Build combined G-code for minimal latency
        cmds = []
        
        pan_cmd = f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={pan_pos:.4f}"
        if pan_speed:
            pan_cmd += f" SPEED={pan_speed:.2f}"
        cmds.append(pan_cmd)
        
        tilt_cmd = f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={tilt_pos:.4f}"
        if tilt_speed:
            tilt_cmd += f" SPEED={tilt_speed:.2f}"
        cmds.append(tilt_cmd)
        
        # Send as single script for atomicity
        script = "\n".join(cmds)
        return self.gcode(script, wait=False), True
    
    def emergency_stop(self):
        """Emergency stop - disable all steppers."""
        self.gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=0", wait=False)
        self.gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=0", wait=False)


class KlipperMotorControl:
    """
    Drop-in replacement for MotorControl that uses direct Klipper connection.
    
    Provides the same interface as the Moonraker-based MotorControl but with
    much lower latency.
    """
    
    def __init__(
        self,
        socket_path: str = "/home/acp/printer_data/comms/klippy.sock",
        pan_stepper: str = "stepper_0",
        tilt_stepper: str = "stepper_1",
    ):
        self.klipper = KlipperDirect(socket_path)
        self.pan_stepper = pan_stepper
        self.tilt_stepper = tilt_stepper
        
        self._pan_pos = 0.0
        self._tilt_pos = 0.0
        self._connected = False
        
    def connect(self) -> bool:
        """Connect to Klipper and initialize steppers."""
        if not self.klipper.connect():
            return False
        
        # Enable steppers and set initial position
        self.klipper.enable_stepper(self.pan_stepper, True)
        self.klipper.enable_stepper(self.tilt_stepper, True)
        self.klipper.set_position(self.pan_stepper, 0)
        self.klipper.set_position(self.tilt_stepper, 0)
        
        self._pan_pos = 0.0
        self._tilt_pos = 0.0
        self._connected = True
        
        return True
    
    def disconnect(self):
        """Disconnect from Klipper."""
        if self._connected:
            self.klipper.emergency_stop()
            self.klipper.disconnect()
            self._connected = False
    
    def move_pan(self, position: float, speed: Optional[float] = None) -> bool:
        """Move pan axis to absolute position."""
        self._pan_pos = position
        return self.klipper.move_stepper(
            self.pan_stepper, position, speed=speed
        )
    
    def move_tilt(self, position: float, speed: Optional[float] = None) -> bool:
        """Move tilt axis to absolute position."""
        self._tilt_pos = position
        return self.klipper.move_stepper(
            self.tilt_stepper, position, speed=speed
        )
    
    def move_both(
        self,
        pan: float,
        tilt: float,
        pan_speed: Optional[float] = None,
        tilt_speed: Optional[float] = None,
    ) -> bool:
        """Move both axes simultaneously."""
        self._pan_pos = pan
        self._tilt_pos = tilt
        self.klipper.move_pan_tilt(pan, tilt, pan_speed, tilt_speed)
        return True
    
    @property
    def pan_position(self) -> float:
        return self._pan_pos
    
    @property
    def tilt_position(self) -> float:
        return self._tilt_pos


# Test direct connection
if __name__ == "__main__":
    print("Testing direct Klipper connection...")
    
    klipper = KlipperDirect()
    if not klipper.connect():
        print("Failed to connect!")
        exit(1)
    
    print("Connected! Testing G-code commands...")
    
    # Test basic commands
    t0 = time.time()
    klipper.gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1 SET_POSITION=0")
    t1 = time.time()
    print(f"Enable command: {(t1-t0)*1000:.1f}ms")
    
    # Test movement
    t0 = time.time()
    klipper.move_stepper("stepper_0", 2.0, speed=50)
    t1 = time.time()
    print(f"Move command: {(t1-t0)*1000:.1f}ms")
    
    time.sleep(0.5)
    
    # Move back
    t0 = time.time()
    klipper.move_stepper("stepper_0", 0.0, speed=50)
    t1 = time.time()
    print(f"Move back command: {(t1-t0)*1000:.1f}ms")
    
    time.sleep(0.5)
    
    # Disable
    klipper.enable_stepper("stepper_0", False)
    klipper.disconnect()
    
    print("Done!")
