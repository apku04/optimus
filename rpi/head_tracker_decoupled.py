#!/usr/bin/env python3
"""Decoupled head tracker (fresh approach)

Goal
----
Stop oscillation by cancelling the camera-induced measurement shift.

Model (per axis)
----------------
Let Jetson report normalized offsets (x,y) in [-1, +1]. When we move the
camera by some motor angle, the *measured* offset changes even if the face
is stationary. We approximate:

  x ≈ x_face + a_pan * pan_deg
  y ≈ y_face + a_tilt * tilt_deg

Where a_pan = dx/dpan, a_tilt = dy/dtilt (units: normalized offset per degree).
If we know a_pan/a_tilt, a single correction that drives x,y toward 0 is:

  delta_pan  = - x / a_pan
  delta_tilt = - y / a_tilt

This removes the feedback-loop oscillation for stationary targets.

Usage
-----
- Calibrate mapping:
    python3 rpi/head_tracker_decoupled.py --calibrate

- Run tracker:
    python3 rpi/head_tracker_decoupled.py

- Dry-run (no motor moves):
    python3 rpi/head_tracker_decoupled.py --dry-run

Notes
-----
- Supports Jetson v5 packet fields: x,y,detected,confidence,stability,box_size.
- Also accepts older JSON fields (x_offset/y_offset or offset_x/offset_y) and CSV.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))
from motor_control import MotorController


@dataclass
class FacePacket:
    x: float
    y: float
    detected: bool
    confidence: float
    stability: float
    box_size: float
    x_velocity: float
    y_velocity: float
    timestamp: float


class UdpFaceReceiver:
    def __init__(self, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

    def _parse(self, msg: str) -> Optional[FacePacket]:
        msg = msg.strip()

        # CSV: "x,y" (legacy)
        if "," in msg and not msg.startswith("{"):
            parts = msg.split(",")
            if len(parts) >= 2:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    return FacePacket(
                        x=x,
                        y=y,
                        detected=True,
                        confidence=1.0,
                        stability=1.0,
                        box_size=0.01,
                        x_velocity=0.0,
                        y_velocity=0.0,
                        timestamp=time.time(),
                    )
                except ValueError:
                    return None

        if not msg.startswith("{"):
            return None

        try:
            data = json.loads(msg)
        except json.JSONDecodeError:
            return None

        # Jetson v5 fields
        detected = bool(data.get("detected", data.get("face_detected", False)))

        # Accept several key variants from past experiments
        x = data.get("x", data.get("x_offset", data.get("offset_x", 0.0)))
        y = data.get("y", data.get("y_offset", data.get("offset_y", 0.0)))

        try:
            x = float(x)
            y = float(y)
        except (TypeError, ValueError):
            return None

        conf = float(data.get("confidence", 1.0) or 0.0)
        stability = float(data.get("stability", 1.0) or 0.0)
        box_size = float(data.get("box_size", 0.01) or 0.01)
        x_vel = float(data.get("x_velocity", 0.0) or 0.0)
        y_vel = float(data.get("y_velocity", 0.0) or 0.0)
        ts = float(data.get("timestamp", time.time()) or time.time())

        return FacePacket(
            x=max(-1.0, min(1.0, x)),
            y=max(-1.0, min(1.0, y)),
            detected=detected,
            confidence=conf,
            stability=max(0.0, min(1.0, stability)),
            box_size=max(0.0, min(1.0, box_size)),
            x_velocity=max(-5.0, min(5.0, x_vel)),
            y_velocity=max(-5.0, min(5.0, y_vel)),
            timestamp=ts,
        )

    def get_latest(self) -> Optional[FacePacket]:
        latest: Optional[FacePacket] = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break

            try:
                pkt = self._parse(data.decode("utf-8", errors="ignore"))
            except Exception:
                pkt = None

            if pkt is not None:
                latest = pkt

        return latest

    def drain(self):
        while True:
            try:
                self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break


def _median_samples(receiver: UdpFaceReceiver, seconds: float, min_samples: int = 8) -> Optional[FacePacket]:
    """Collect samples for a short window and return a median-smoothed packet."""
    xs, ys, confs, stabs, boxes, xvs, yvs, tss = [], [], [], [], [], [], [], []
    deadline = time.time() + seconds

    while time.time() < deadline:
        pkt = receiver.get_latest()
        if pkt is None:
            time.sleep(0.01)
            continue
        if not pkt.detected:
            time.sleep(0.01)
            continue
        xs.append(pkt.x)
        ys.append(pkt.y)
        confs.append(pkt.confidence)
        stabs.append(pkt.stability)
        boxes.append(pkt.box_size)
        xvs.append(pkt.x_velocity)
        yvs.append(pkt.y_velocity)
        tss.append(pkt.timestamp)
        time.sleep(0.01)

    if len(xs) < min_samples:
        return None

    return FacePacket(
        x=float(statistics.median(xs)),
        y=float(statistics.median(ys)),
        detected=True,
        confidence=float(statistics.median(confs)),
        stability=float(statistics.median(stabs)),
        box_size=float(statistics.median(boxes)),
        x_velocity=float(statistics.median(xvs)),
        y_velocity=float(statistics.median(yvs)),
        timestamp=float(statistics.median(tss)),
    )


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _safe_div(num: float, den: float, eps: float = 1e-6) -> Optional[float]:
    if abs(den) < eps:
        return None
    return num / den


def _estimate_trap_time(distance: float, speed: float, accel: float) -> float:
    """Estimate motion time for a 1D trapezoid/triangle profile.

    Units are arbitrary but must be consistent (deg, deg/s, deg/s^2).
    """
    distance = abs(float(distance))
    speed = max(1e-6, abs(float(speed)))
    accel = max(0.0, float(accel))

    if distance <= 0.0:
        return 0.0
    if accel <= 1e-6:
        return distance / speed

    t_to_v = speed / accel
    dist_accel = 0.5 * accel * (t_to_v ** 2)

    # Triangular profile (can't reach max speed)
    if (2.0 * dist_accel) >= distance:
        return 2.0 * math.sqrt(distance / accel)

    # Trapezoid profile
    cruise_dist = distance - (2.0 * dist_accel)
    cruise_t = cruise_dist / speed
    return (2.0 * t_to_v) + cruise_t


def calibrate(config_path: Path, *, settle_s: float, step_deg: float, sample_s: float, sync: bool) -> int:
    with open(config_path) as f:
        cfg = json.load(f)

    receiver = UdpFaceReceiver(cfg["network"]["udp_port"])
    motors = MotorController(str(config_path))

    if not motors.is_ready():
        print("[ERROR] Moonraker not ready")
        return 2

    pan_cfg = cfg["motors"]["pan"]
    tilt_cfg = cfg["motors"]["tilt"]

    pan_center = float(pan_cfg["center"])
    tilt_center = float(tilt_cfg["center"])

    print("Centering head...")
    motors.move_pan(pan_center, speed=pan_cfg.get("speed"), sync=sync)
    motors.move_tilt(tilt_center, speed=tilt_cfg.get("speed"), sync=sync)
    time.sleep(settle_s)

    print("Hold your face still and look at the camera.")
    print(f"Sampling baseline for {sample_s:.2f}s...")
    receiver.drain()
    base = _median_samples(receiver, seconds=sample_s)
    if base is None:
        print("[ERROR] Not enough samples / no face detected")
        return 3

    def measure_after_move(pan: float, tilt: float) -> Optional[FacePacket]:
        motors.move_pan(pan, speed=pan_cfg.get("speed"), sync=sync)
        motors.move_tilt(tilt, speed=tilt_cfg.get("speed"), sync=sync)
        time.sleep(settle_s)
        receiver.drain()
        return _median_samples(receiver, seconds=sample_s)

    # PAN slope dx/dpan
    print(f"Calibrating pan slope with ±{step_deg}° steps...")
    p_plus = measure_after_move(pan_center + step_deg, tilt_center)
    p_minus = measure_after_move(pan_center - step_deg, tilt_center)
    # Return to center
    measure_after_move(pan_center, tilt_center)

    if p_plus is None or p_minus is None:
        print("[ERROR] Pan calibration failed (no samples)")
        return 4

    # Central difference derivative
    dx_dpan = (p_plus.x - p_minus.x) / (2.0 * step_deg)
    dy_dpan = (p_plus.y - p_minus.y) / (2.0 * step_deg)

    # TILT slope dy/dtilt
    print(f"Calibrating tilt slope with ±{step_deg}° steps...")
    t_plus = measure_after_move(pan_center, tilt_center + step_deg)
    t_minus = measure_after_move(pan_center, tilt_center - step_deg)
    measure_after_move(pan_center, tilt_center)

    if t_plus is None or t_minus is None:
        print("[ERROR] Tilt calibration failed (no samples)")
        return 5

    dx_dtilt = (t_plus.x - t_minus.x) / (2.0 * step_deg)
    dy_dtilt = (t_plus.y - t_minus.y) / (2.0 * step_deg)

    print("\nCalibration results (normalized offset per degree):")
    print(f"  dx/dpan  = {dx_dpan:+.6f}")
    print(f"  dy/dpan  = {dy_dpan:+.6f}  (cross-coupling)")
    print(f"  dx/dtilt = {dx_dtilt:+.6f}  (cross-coupling)")
    print(f"  dy/dtilt = {dy_dtilt:+.6f}")

    # Basic sanity checks
    if abs(dx_dpan) < 1e-4 or abs(dy_dtilt) < 1e-4:
        print("[WARN] Very small slopes; calibration may be invalid. Try larger step_deg or increase settle time.")

    cfg.setdefault("tracking_decoupled", {})
    cfg["tracking_decoupled"].update(
        {
            # Full 2x2 mapping from motor degrees to image offsets:
            # [x; y] \approx [[dx_dpan, dx_dtilt], [dy_dpan, dy_dtilt]] * [pan; tilt]
            "dx_dpan": dx_dpan,
            "dx_dtilt": dx_dtilt,
            "dy_dpan": dy_dpan,
            "dy_dtilt": dy_dtilt,
            "step_deg": step_deg,
            "settle_s": settle_s,
            "sample_s": sample_s,
            "timestamp": time.time(),
            "notes": "Decoupled control calibration. Matrix A maps motor degrees to image offsets: [x;y] ≈ A*[pan;tilt]. Used as delta = -inv(A)*[x;y].",
        }
    )

    with open(config_path, "w") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")

    print(f"\nSaved to {config_path}")
    return 0


def run_tracker(config_path: Path, *, dry_run: bool, sync: bool) -> int:
    with open(config_path) as f:
        cfg = json.load(f)

    receiver = UdpFaceReceiver(cfg["network"]["udp_port"])
    motors = MotorController(str(config_path))

    if not dry_run and not motors.is_ready():
        print("[ERROR] Moonraker not ready")
        return 2

    pan_cfg = cfg["motors"]["pan"]
    tilt_cfg = cfg["motors"]["tilt"]
    track_cfg = cfg.get("tracking", {})
    dec = cfg.get("tracking_decoupled", {})

    dx_dpan = dec.get("dx_dpan")
    dx_dtilt = dec.get("dx_dtilt")
    dy_dpan = dec.get("dy_dpan")
    dy_dtilt = dec.get("dy_dtilt")

    if dx_dpan is None or dy_dtilt is None:
        print("[ERROR] Missing calibration in config.json: tracking_decoupled.dx_dpan / dy_dtilt")
        print("Run: python3 rpi/head_tracker_decoupled.py --calibrate")
        return 3

    # Cross-coupling is optional; if absent we fall back to diagonal control.
    if dx_dtilt is None:
        dx_dtilt = 0.0
    if dy_dpan is None:
        dy_dpan = 0.0

    invert_pan = bool(track_cfg.get("invert_pan", False))
    invert_tilt = bool(track_cfg.get("invert_tilt", False))

    # Control params
    deadzone = float(track_cfg.get("deadzone", 0.08))
    min_conf = float(dec.get("min_confidence", 0.40))
    base_alpha = float(dec.get("alpha", 0.85))
    # Velocity lookahead can easily turn into "cat chasing mouse" because the
    # measured x_velocity/y_velocity includes camera-induced motion. Keep it
    # OFF by default; enable explicitly if you really need prediction.
    use_velocity_prediction = bool(dec.get("use_velocity_prediction", False))
    lookahead_s = float(dec.get("lookahead_s", 0.00))

    # Optional: velocity/rate-based control (more gimbal-like).
    # In this mode we compute desired image velocity and map it to motor rates
    # using the same calibrated matrix A, then integrate over dt.
    control_mode = str(dec.get("control_mode", "position")).strip().lower()
    rate_kp = float(dec.get("rate_kp", 2.5))  # 1/s, higher = more aggressive centering
    rate_kd = float(dec.get("rate_kd", 0.35))  # damping on measured x_velocity/y_velocity
    rate_use_velocity_feedback = bool(dec.get("rate_use_velocity_feedback", False))
    rate_motor_ema = float(dec.get("rate_motor_ema", 0.80))  # 0=no smoothing, 0.7..0.9 recommended
    rate_speed_dt_min_s = float(dec.get("rate_speed_dt_min_s", 0.06))

    # Position-mode settling helpers: reduce gain near center to avoid "bounce".
    alpha_near_mult = float(dec.get("alpha_near_mult", 0.60))
    alpha_far_mult = float(dec.get("alpha_far_mult", 1.00))

    # Command pacing / smoothing
    ema = float(dec.get("ema", 0.60))  # 0=no smoothing, 0.6..0.9 recommended
    move_cooldown_s = float(dec.get("move_cooldown_s", 0.12))  # minimum time between commands
    combine_moves = bool(dec.get("combine_moves", False))

    # Ignore measurement-driven updates briefly after a move. This reduces
    # oscillation where the camera motion itself makes the measured box move.
    # Typical values: 0.04..0.12 seconds.
    post_move_settle_s = float(dec.get("post_move_settle_s", 0.0))
    hold_measurements_during_motion = bool(dec.get("hold_measurements_during_motion", True))

    # Optional "sweep" mode: when far from center, do one longer smooth move
    # instead of many tiny corrections. Implemented as a dynamic clamp/speed
    # boost when far from center (non-blocking) to preserve tracking.
    sweep_enabled = bool(dec.get("sweep_enabled", False))
    sweep_error = float(dec.get("sweep_error", 0.35))
    sweep_max_step_pan = float(dec.get("sweep_max_step_pan_deg", 3.0))
    sweep_max_step_tilt = float(dec.get("sweep_max_step_tilt_deg", 1.2))
    sweep_pan_speed = dec.get("sweep_pan_speed")
    sweep_tilt_speed = dec.get("sweep_tilt_speed")

    # Smooth commanded setpoints (not measurements): this reduces the
    # "ink-printer stepping" feel without sacrificing convergence accuracy.
    # 0.0 = no smoothing, 0.7..0.9 = very smooth.
    cmd_smoothing = float(dec.get("cmd_smoothing", 0.0))
    cmd_smoothing_near = float(dec.get("cmd_smoothing_near", cmd_smoothing))
    cmd_smoothing_far = float(dec.get("cmd_smoothing_far", cmd_smoothing))

    # Anti-dither: hysteresis + require a few consecutive samples outside the
    # deadzone before issuing a move.
    deadzone_hyst = float(dec.get("deadzone_hysteresis", 0.03))
    require_outside_n = int(dec.get("require_outside_samples", 3))

    max_step_pan = float(dec.get("max_step_pan_deg", 0.60))
    max_step_tilt = float(dec.get("max_step_tilt_deg", 0.25))
    min_step_pan = float(dec.get("min_step_pan_deg", 0.05))
    min_step_tilt = float(dec.get("min_step_tilt_deg", 0.03))

    # Backlash / reversal suppression: if we keep issuing tiny opposite-direction
    # moves, steppers can appear to "oscillate" forever (mechanical play + latency).
    reverse_deadband_pan = float(dec.get("reverse_deadband_pan_deg", 0.0))
    reverse_deadband_tilt = float(dec.get("reverse_deadband_tilt_deg", 0.0))

    loop_hz = float(dec.get("loop_hz", 20.0))
    loop_dt = 1.0 / max(1.0, loop_hz)

    speed_scaling = bool(dec.get("speed_scaling", True))

    # Speed caps (previously hard-coded to 40/30). Keep conservative defaults,
    # but allow raising them when Klipper is configured accordingly.
    max_pan_speed = float(dec.get("max_pan_speed", 40.0))
    max_tilt_speed = float(dec.get("max_tilt_speed", 30.0))

    return_to_center_delay = float(track_cfg.get("return_to_center_delay", 3.0))

    # Start at center
    pan = float(pan_cfg["center"])
    tilt = float(tilt_cfg["center"])

    pan_accel = float(pan_cfg.get("accel", 0.0) or 0.0)
    tilt_accel = float(tilt_cfg.get("accel", 0.0) or 0.0)

    cmd_pan = pan
    cmd_tilt = tilt

    last_pan_dir = 0  # -1,0,+1
    last_tilt_dir = 0

    filt_x = 0.0
    filt_y = 0.0
    filt_init = False

    last_pkt_ts: Optional[float] = None

    # Rate-mode state
    pan_dot_filt = 0.0
    tilt_dot_filt = 0.0

    if not dry_run:
        motors.move_pan(pan, speed=pan_cfg.get("speed"), sync=sync)
        motors.move_tilt(tilt, speed=tilt_cfg.get("speed"), sync=sync)

    last_face_time = time.time()
    last_print = 0.0
    last_cmd_time = 0.0
    hold_until = 0.0
    outside_count = 0
    tracking_active = False

    print("=" * 72)
    print("DECOUPLED HEAD TRACKER")
    print(f"UDP port: {cfg['network']['udp_port']} | dry_run={dry_run} | sync={sync}")
    print(f"deadzone={deadzone} alpha={base_alpha} lookahead={lookahead_s}s")
    det = (dx_dpan * dy_dtilt) - (dx_dtilt * dy_dpan)
    print(
        f"A=[[{dx_dpan:+.6f},{dx_dtilt:+.6f}],[{dy_dpan:+.6f},{dy_dtilt:+.6f}]] det={det:+.6f}"
    )
    print("=" * 72)

    try:
        while True:
            loop_start = time.time()
            pkt = receiver.get_latest()

            # If we recently issued a move, optionally ignore correction updates
            # until the move is expected to have settled.
            if hold_measurements_during_motion and (time.time() < hold_until):
                if pkt is not None and pkt.detected and pkt.confidence >= min_conf:
                    last_face_time = time.time()
                sleep_s = min(loop_dt, max(0.0, hold_until - time.time()))
                if sleep_s > 0:
                    time.sleep(sleep_s)
                continue

            if pkt is None or not pkt.detected or pkt.confidence < min_conf:
                # lost face
                if time.time() - last_face_time > return_to_center_delay:
                    if abs(pan - float(pan_cfg["center"])) > 0.2 or abs(tilt - float(tilt_cfg["center"])) > 0.2:
                        pan = float(pan_cfg["center"])
                        tilt = float(tilt_cfg["center"])
                        if dry_run:
                            print("[CENTER] no face -> center")
                        else:
                            motors.move_pan(pan, speed=max(5, int(pan_cfg.get("speed", 15) * 0.7)), sync=sync)
                            motors.move_tilt(tilt, speed=max(5, int(tilt_cfg.get("speed", 12) * 0.7)), sync=sync)
                    last_face_time = time.time()

                time.sleep(0.02)
                continue

            last_face_time = time.time()

            # dt based on sender timestamps (Jetson). We only need delta, so
            # absolute clock sync does not matter.
            dt = loop_dt
            if pkt.timestamp and (last_pkt_ts is not None):
                try:
                    dt = float(pkt.timestamp - last_pkt_ts)
                except Exception:
                    dt = loop_dt
            last_pkt_ts = float(pkt.timestamp) if pkt.timestamp else last_pkt_ts
            dt = _clamp(dt, 1e-3, 0.25)

            # Optional prediction
            if use_velocity_prediction and lookahead_s > 1e-6:
                pred_x = _clamp(pkt.x + pkt.x_velocity * lookahead_s, -1.0, 1.0)
                pred_y = _clamp(pkt.y + pkt.y_velocity * lookahead_s, -1.0, 1.0)
            else:
                pred_x = float(pkt.x)
                pred_y = float(pkt.y)

            # Light EMA smoothing (reduces tiny sign flips that cause dithering)
            if not filt_init:
                filt_x, filt_y = pred_x, pred_y
                filt_init = True
            else:
                a = _clamp(ema, 0.0, 0.98)
                filt_x = (a * filt_x) + ((1.0 - a) * pred_x)
                filt_y = (a * filt_y) + ((1.0 - a) * pred_y)

            pred_x, pred_y = filt_x, filt_y

            max_err = max(abs(pred_x), abs(pred_y))

            if abs(pred_x) < deadzone and abs(pred_y) < deadzone:
                if time.time() - last_print > 1.0:
                    print(f"[HOLD] x={pred_x:+.3f} y={pred_y:+.3f} conf={pkt.confidence:.2f} stab={pkt.stability:.2f}")
                    last_print = time.time()
                outside_count = 0
                tracking_active = False
                time.sleep(loop_dt)
                continue

            # Hysteresis: only start tracking when clearly outside; stop when
            # clearly inside.
            dz_enter = max(0.0, deadzone + deadzone_hyst)
            dz_exit = max(0.0, deadzone - deadzone_hyst)

            if tracking_active:
                if abs(pred_x) < dz_exit and abs(pred_y) < dz_exit:
                    outside_count = 0
                    tracking_active = False
                    time.sleep(loop_dt)
                    continue
            else:
                if abs(pred_x) > dz_enter or abs(pred_y) > dz_enter:
                    outside_count += 1
                    if outside_count < max(1, require_outside_n):
                        time.sleep(loop_dt)
                        continue
                    tracking_active = True
                else:
                    outside_count = 0
                    time.sleep(loop_dt)
                    continue

            # Rate-limit commands so we don't queue moves faster than they execute.
            # This is critical when SYNC=0 (non-blocking): if commands queue, the
            # controller's internal pan/tilt state diverges from the physical head,
            # creating limit cycles.
            now = time.time()
            if (now - last_cmd_time) < move_cooldown_s:
                time.sleep(loop_dt)
                continue

            # Behavior modulation
            # - close face -> slightly more aggressive
            dist_mult = _clamp(0.5 + pkt.box_size * 2.0, 0.5, 1.5)
            # - unstable -> less aggressive
            stab_mult = _clamp(pkt.stability, 0.3, 1.0)

            alpha = base_alpha * dist_mult * stab_mult
            alpha = _clamp(alpha, 0.2, 1.2)

            # In position mode, ease off near the center so we settle instead of
            # constantly stepping past the target due to latency/backlash.
            if control_mode != "rate":
                t_err = _clamp((max_err - deadzone) / max(1e-6, (1.0 - deadzone)), 0.0, 1.0)
                near = _clamp(alpha_near_mult, 0.1, 1.0)
                far = _clamp(alpha_far_mult, 0.3, 1.5)
                alpha *= ((1.0 - t_err) * near) + (t_err * far)
                alpha = _clamp(alpha, 0.1, 1.2)

            # Core decoupled control
            # A maps motor degrees -> image offset, so:
            # - position mode: delta_deg = -inv(A) * [x;y]
            # - rate mode: deg/s = inv(A) * v_img, where v_img = d[x;y]/dt
            det = (dx_dpan * dy_dtilt) - (dx_dtilt * dy_dpan)
            if abs(det) < 1e-6:
                print("[ERROR] Calibration matrix near-singular; re-run --calibrate")
                return 4

            if control_mode == "rate":
                kp = _clamp(rate_kp * dist_mult * stab_mult, 0.1, 20.0)
                kd = _clamp(rate_kd * stab_mult, 0.0, 5.0)

                # Desired image velocity (normalized units/sec)
                # IMPORTANT: measured x_velocity/y_velocity can include camera-induced motion,
                # so feeding it back can create a loop. Keep it OFF by default.
                if rate_use_velocity_feedback:
                    v_img_x = (-kp * pred_x) - (kd * pkt.x_velocity)
                    v_img_y = (-kp * pred_y) - (kd * pkt.y_velocity)
                else:
                    v_img_x = (-kp * pred_x)
                    v_img_y = (-kp * pred_y)

                # Convert to motor rates (deg/sec): [pan_dot; tilt_dot] = inv(A) * [v_img_x; v_img_y]
                pan_dot = ((dy_dtilt * v_img_x) + (-dx_dtilt * v_img_y)) / det
                tilt_dot = (((-dy_dpan) * v_img_x) + (dx_dpan * v_img_y)) / det

                # Smooth motor rates to avoid limit cycles from quantization/delay.
                rm = _clamp(rate_motor_ema, 0.0, 0.95)
                pan_dot_filt = (rm * pan_dot_filt) + ((1.0 - rm) * pan_dot)
                tilt_dot_filt = (rm * tilt_dot_filt) + ((1.0 - rm) * tilt_dot)
                pan_dot, tilt_dot = pan_dot_filt, tilt_dot_filt

                # Integrate to get next target step (deg)
                delta_pan = pan_dot * dt
                delta_tilt = tilt_dot * dt
            else:
                # Position mode (default): delta = -inv(A) * [x; y]
                delta_pan = -((dy_dtilt * pred_x) + (-dx_dtilt * pred_y)) / det
                delta_tilt = -(((-dy_dpan) * pred_x) + (dx_dpan * pred_y)) / det
                delta_pan *= alpha
                delta_tilt *= alpha

            # Clamp step sizes for smooth motion. In sweep mode, allow larger
            # steps when far from center while still updating each loop.
            max_step_pan_now = max_step_pan
            max_step_tilt_now = max_step_tilt
            sweep_zone = sweep_enabled and (max_err > sweep_error)
            if sweep_zone:
                # Smoothly scale from normal limits -> sweep limits as error grows.
                t = _clamp((max_err - sweep_error) / max(1e-6, (1.0 - sweep_error)), 0.0, 1.0)
                max_step_pan_now = (1.0 - t) * max_step_pan + t * max(max_step_pan, sweep_max_step_pan)
                max_step_tilt_now = (1.0 - t) * max_step_tilt + t * max(max_step_tilt, sweep_max_step_tilt)

            delta_pan = _clamp(delta_pan, -max_step_pan_now, max_step_pan_now)
            delta_tilt = _clamp(delta_tilt, -max_step_tilt_now, max_step_tilt_now)

            if invert_pan:
                delta_pan *= -1.0
            if invert_tilt:
                delta_tilt *= -1.0

            # Compute unclipped target first.
            target_pan = _clamp(pan + delta_pan, float(pan_cfg["min"]), float(pan_cfg["max"]))
            target_tilt = _clamp(tilt + delta_tilt, float(tilt_cfg["min"]), float(tilt_cfg["max"]))

            # Smooth the command trajectory toward the computed new target.
            # This preserves precision because the target is still driven by
            # the decoupled inverse model; we just glide toward it.
            cs = _clamp(cmd_smoothing_far if sweep_zone else cmd_smoothing_near, 0.0, 0.98)
            if cs > 0.0:
                cmd_pan = (cs * cmd_pan) + ((1.0 - cs) * target_pan)
                cmd_tilt = (cs * cmd_tilt) + ((1.0 - cs) * target_tilt)
                target_pan, target_tilt = cmd_pan, cmd_tilt

            # Re-clamp after smoothing (defensive).
            target_pan = _clamp(target_pan, float(pan_cfg["min"]), float(pan_cfg["max"]))
            target_tilt = _clamp(target_tilt, float(tilt_cfg["min"]), float(tilt_cfg["max"]))

            # Minimum motion threshold (apply AFTER smoothing).
            # Otherwise, cmd_smoothing can turn a valid correction into a stream
            # of tiny incremental commands that look/feel like jitter.
            step_pan = target_pan - pan
            step_tilt = target_tilt - tilt

            # Reversal suppression (anti-backlash): avoid tiny direction flips.
            # If the controller asks to reverse direction but the step is smaller
            # than the configured deadband, skip it and wait for a clearer error.
            if step_pan != 0.0:
                pan_dir = 1 if step_pan > 0.0 else -1
                if last_pan_dir != 0 and pan_dir != last_pan_dir and abs(step_pan) < reverse_deadband_pan:
                    target_pan = pan
                    step_pan = 0.0
                elif abs(step_pan) >= 1e-6:
                    last_pan_dir = pan_dir

            if step_tilt != 0.0:
                tilt_dir = 1 if step_tilt > 0.0 else -1
                if last_tilt_dir != 0 and tilt_dir != last_tilt_dir and abs(step_tilt) < reverse_deadband_tilt:
                    target_tilt = tilt
                    step_tilt = 0.0
                elif abs(step_tilt) >= 1e-6:
                    last_tilt_dir = tilt_dir
            if abs(step_pan) < min_step_pan:
                target_pan = pan
                step_pan = 0.0
            if abs(step_tilt) < min_step_tilt:
                target_tilt = tilt
                step_tilt = 0.0

            if step_pan == 0.0 and step_tilt == 0.0:
                time.sleep(loop_dt)
                continue

            # Speed scaling: match your gentle style
            base_pan_speed = float(dec.get("pan_speed", pan_cfg.get("speed", 15)))
            base_tilt_speed = float(dec.get("tilt_speed", tilt_cfg.get("speed", 12)))

            if sweep_zone:
                t = _clamp((max_err - sweep_error) / max(1e-6, (1.0 - sweep_error)), 0.0, 1.0)
                if sweep_pan_speed is not None:
                    base_pan_speed = (1.0 - t) * base_pan_speed + t * float(sweep_pan_speed)
                if sweep_tilt_speed is not None:
                    base_tilt_speed = (1.0 - t) * base_tilt_speed + t * float(sweep_tilt_speed)

            if control_mode == "rate":
                # In rate mode, SPEED should reflect desired angular rate.
                # We infer desired rate from (step / dt) and clamp to caps.
                eff_dt = max(1e-3, max(dt, rate_speed_dt_min_s, move_cooldown_s))
                pan_rate = abs(step_pan) / eff_dt
                tilt_rate = abs(step_tilt) / eff_dt
                pan_speed = float(_clamp(pan_rate, 0.5, max_pan_speed))
                tilt_speed = float(_clamp(tilt_rate, 0.5, max_tilt_speed))
            else:
                # For fluid motion, keep speeds constant (no per-step speed jumps).
                # If enabled, scale speed with step magnitude.
                if speed_scaling:
                    pan_speed = float(_clamp(base_pan_speed * (0.5 + abs(delta_pan) / max_step_pan_now), 0.5, max_pan_speed))
                    tilt_speed = float(_clamp(base_tilt_speed * (0.5 + abs(delta_tilt) / max_step_tilt_now), 0.5, max_tilt_speed))
                else:
                    pan_speed = float(_clamp(base_pan_speed, 0.5, max_pan_speed))
                    tilt_speed = float(_clamp(base_tilt_speed, 0.5, max_tilt_speed))

            if dry_run:
                print(
                    f"[MOVE] x={pred_x:+.3f} y={pred_y:+.3f} a={alpha:.2f} "
                    f"-> Δpan={step_pan:+.3f} Δtilt={step_tilt:+.3f} "
                    f"=> pan={target_pan:+.2f} tilt={target_tilt:+.2f}"
                )
            else:
                # Non-blocking by default for smoothness; SYNC is user-selectable.
                pan_changed = abs(target_pan - pan) > 1e-6
                tilt_changed = abs(target_tilt - tilt) > 1e-6

                if combine_moves and pan_changed and tilt_changed:
                    motors.move_pan_tilt(target_pan, target_tilt, pan_speed=pan_speed, tilt_speed=tilt_speed, sync=sync)
                else:
                    if pan_changed:
                        motors.move_pan(target_pan, speed=pan_speed, sync=sync)
                    if tilt_changed:
                        motors.move_tilt(target_tilt, speed=tilt_speed, sync=sync)

            pan, tilt = target_pan, target_tilt

            # Estimate how long the move will take and delay next command accordingly.
            # Avoids command pileup when SYNC=0.
            # Use accel-aware estimate when accel is configured; this reduces
            # mid-move corrections that feel like overshoot.
            est_pan_t = _estimate_trap_time(step_pan, pan_speed, pan_accel) if pan_accel > 1e-6 else (abs(step_pan) / max(0.01, pan_speed))
            est_tilt_t = _estimate_trap_time(step_tilt, tilt_speed, tilt_accel) if tilt_accel > 1e-6 else (abs(step_tilt) / max(0.01, tilt_speed))
            last_cmd_time = time.time()
            extra_wait = max(est_pan_t, est_tilt_t, move_cooldown_s)

            # Sample-hold: ignore fresh measurements until motion is done and
            # the image has had a chance to settle.
            if hold_measurements_during_motion:
                hold_until = last_cmd_time + extra_wait + max(0.0, post_move_settle_s)

            # Maintain loop rate
            elapsed = time.time() - loop_start
            # If moves are slow, prefer waiting for physical motion rather than
            # re-reading immediately and reacting to the transient.
            target_sleep = max(loop_dt, extra_wait)
            if elapsed < target_sleep:
                time.sleep(target_sleep - elapsed)

    except KeyboardInterrupt:
        print("\n[STOP] exiting")
        return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(Path(__file__).parent.parent / "config.json"))
    ap.add_argument("--dry-run", action="store_true", help="Print moves without commanding motors")
    ap.add_argument("--sync", action="store_true", help="Use SYNC=1 blocking moves (slower but deterministic)")

    ap.add_argument("--calibrate", action="store_true", help="Estimate dx/dpan and dy/dtilt and save to config")
    ap.add_argument("--cal-step-deg", type=float, default=2.0)
    ap.add_argument("--cal-settle-s", type=float, default=0.6)
    ap.add_argument("--cal-sample-s", type=float, default=0.6)

    args = ap.parse_args()
    config_path = Path(args.config)

    if args.calibrate:
        return calibrate(
            config_path,
            settle_s=args.cal_settle_s,
            step_deg=args.cal_step_deg,
            sample_s=args.cal_sample_s,
            sync=args.sync,
        )

    return run_tracker(config_path, dry_run=args.dry_run, sync=args.sync)


if __name__ == "__main__":
    raise SystemExit(main())
