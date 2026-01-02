#!/usr/bin/env python3
"""Cascaded PI pan/tilt visual servo controller.

This is a fresh implementation designed to feel "gimbal-like" while avoiding
limit-cycle oscillation from camera-on-actuator feedback + latency.

Core idea (standard approach)
-----------------------------
Use a cascaded structure:
- Outer loop: image error -> desired image velocity (PI controller)
- Mapping: desired image velocity -> motor angular rates using calibrated A
- Inner loop (implemented via rate/accel limiting): track the requested rates
- Integrate rates to produce short position segments (Klipper MANUAL_STEPPER)

Why this works better than step/hold
------------------------------------
Instead of "error -> jump position -> wait", we continuously command a *rate*
with acceleration limits, producing smooth motion and reducing overshoot in the
presence of measurement delay.

Configuration
-------------
This controller uses your existing 2x2 calibration matrix stored in
config.json under tracking_decoupled:
  A = [[dx_dpan, dx_dtilt], [dy_dpan, dy_dtilt]]
which maps motor degrees to normalized image offsets.

Optional: add a tracking_cascaded section to config.json to tune.

Run:
  python3 rpi/head_tracker_cascaded.py
  python3 rpi/head_tracker_cascaded.py --dry-run

"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

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

        # Legacy CSV: "x,y"
        if "," in msg and not msg.startswith("{"):
            parts = msg.split(",")
            if len(parts) >= 2:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    return FacePacket(
                        x=max(-1.0, min(1.0, x)),
                        y=max(-1.0, min(1.0, y)),
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

        detected = bool(data.get("detected", data.get("face_detected", False)))
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


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _sign(x: float) -> int:
    return 1 if x > 0 else (-1 if x < 0 else 0)


def run(config_path: Path, *, dry_run: bool, sync: bool) -> int:
    cfg = json.loads(config_path.read_text())

    receiver = UdpFaceReceiver(int(cfg["network"]["udp_port"]))
    motors = MotorController(str(config_path))

    if not dry_run and not motors.is_ready():
        print("[ERROR] Moonraker not ready")
        return 2

    motors_cfg = cfg.get("motors", {})
    pan_cfg = motors_cfg.get("pan", {})
    tilt_cfg = motors_cfg.get("tilt", {})

    track_cfg = cfg.get("tracking", {})
    cas = cfg.get("tracking_cascaded", {})
    dec = cfg.get("tracking_decoupled", {})

    # Calibration matrix A (motor degrees -> normalized image offset)
    dx_dpan = float(dec.get("dx_dpan")) if dec.get("dx_dpan") is not None else None
    dx_dtilt = float(dec.get("dx_dtilt")) if dec.get("dx_dtilt") is not None else 0.0
    dy_dpan = float(dec.get("dy_dpan")) if dec.get("dy_dpan") is not None else 0.0
    dy_dtilt = float(dec.get("dy_dtilt")) if dec.get("dy_dtilt") is not None else None

    if dx_dpan is None or dy_dtilt is None:
        print("[ERROR] Missing calibration in config.json: tracking_decoupled.dx_dpan / dy_dtilt")
        print("Run: python3 rpi/head_tracker_decoupled.py --calibrate")
        return 3

    det = (dx_dpan * dy_dtilt) - (dx_dtilt * dy_dpan)
    if abs(det) < 1e-6:
        print("[ERROR] Calibration matrix near-singular; re-run --calibrate")
        return 4

    # User-facing tracking params
    deadzone = float(cas.get("deadzone", track_cfg.get("deadzone", 0.10)))
    min_conf = float(cas.get("min_confidence", 0.40))
    invert_pan = bool(track_cfg.get("invert_pan", False))
    invert_tilt = bool(track_cfg.get("invert_tilt", False))

    # Loop / command pacing
    loop_hz = float(cas.get("loop_hz", 60.0))
    loop_dt = 1.0 / max(1.0, loop_hz)
    min_loop_dt_s = float(cas.get("min_loop_dt_s", 0.008))
    loop_dt = max(loop_dt, min_loop_dt_s)

    segment_dt_s = float(cas.get("segment_dt_s", 0.05))
    segment_dt_s = _clamp(segment_dt_s, 0.015, 0.20)

    move_cooldown_s = float(cas.get("move_cooldown_s", 0.0))
    queue_limit_s = float(cas.get("queue_limit_s", 0.25))

    # Controller gains (image domain)
    # v_img = -kp*e - ki*∫e dt
    kp = float(cas.get("kp", 1.1))  # 1/s
    ki = float(cas.get("ki", 0.35))  # 1/s^2
    integral_limit = float(cas.get("integral_limit", 0.8))  # normalized*s
    integral_leak = float(cas.get("integral_leak", 0.0))  # 1/s (0 = none)

    # Measurement smoothing
    meas_ema = float(cas.get("meas_ema", 0.80))  # 0=no smoothing, 0.7..0.9 typical
    meas_ema = _clamp(meas_ema, 0.0, 0.98)

    # Delay compensation (simple Smith predictor style)
    delay_comp_s = float(cas.get("delay_comp_s", 0.06))
    delay_comp_s = _clamp(delay_comp_s, 0.0, 0.25)

    # Rate/accel limits (motor domain)
    max_pan_rate = float(cas.get("max_pan_rate_dps", dec.get("max_pan_speed", pan_cfg.get("speed", 40.0))))
    max_tilt_rate = float(cas.get("max_tilt_rate_dps", dec.get("max_tilt_speed", tilt_cfg.get("speed", 30.0))))
    max_pan_rate = max(0.5, max_pan_rate)
    max_tilt_rate = max(0.5, max_tilt_rate)

    max_pan_accel = float(cas.get("max_pan_accel_dps2", 600.0))
    max_tilt_accel = float(cas.get("max_tilt_accel_dps2", 450.0))
    max_pan_accel = max(10.0, max_pan_accel)
    max_tilt_accel = max(10.0, max_tilt_accel)

    # Speed clamps for Klipper SPEED parameter
    min_speed = float(cas.get("min_speed", 0.5))
    max_speed = float(cas.get("max_speed", max(max_pan_rate, max_tilt_rate)))
    max_speed = max(max_speed, 1.0)

    # Other behavior
    return_to_center_delay = float(cas.get("return_to_center_delay", track_cfg.get("return_to_center_delay", 3.0)))
    lost_timeout_s = float(cas.get("lost_timeout_s", 0.25))
    combine_moves = bool(cas.get("combine_moves", True))

    # Initial pose
    pan = float(pan_cfg.get("center", 0.0))
    tilt = float(tilt_cfg.get("center", 0.0))

    if not dry_run:
        motors.move_pan(pan, speed=pan_cfg.get("speed"), sync=sync)
        motors.move_tilt(tilt, speed=tilt_cfg.get("speed"), sync=sync)

    # Controller state
    ex_f = 0.0
    ey_f = 0.0
    filt_init = False

    int_ex = 0.0
    int_ey = 0.0

    pan_rate = 0.0
    tilt_rate = 0.0

    last_pkt_time = 0.0
    last_face_time = time.time()

    last_loop_t = time.time()
    last_cmd_t = 0.0
    last_sent_t = 0.0
    commanded_until = 0.0

    last_print = 0.0

    print("=" * 72)
    print("CASCaded PI HEAD TRACKER")
    print(f"UDP port: {cfg['network']['udp_port']} | dry_run={dry_run} | sync={sync}")
    print(f"loop_hz={loop_hz:.1f} (dt={loop_dt*1000:.1f}ms) | segment_dt_s={segment_dt_s:.3f} | queue_limit_s={queue_limit_s:.3f}")
    print(f"deadzone={deadzone} | kp={kp:.3f} ki={ki:.3f} | meas_ema={meas_ema:.2f} | delay_comp_s={delay_comp_s:.3f}")
    print(f"max_rate: pan={max_pan_rate:.1f} tilt={max_tilt_rate:.1f} deg/s | max_accel: pan={max_pan_accel:.0f} tilt={max_tilt_accel:.0f} deg/s^2")
    print(f"A=[[{dx_dpan:+.6f},{dx_dtilt:+.6f}],[{dy_dpan:+.6f},{dy_dtilt:+.6f}]] det={det:+.6f}")
    print("=" * 72)

    def invA_times(vx: float, vy: float) -> tuple[float, float]:
        # inv([[a,b],[c,d]]) * [vx,vy] = 1/det * [ d*vx - b*vy, -c*vx + a*vy]
        pan_dot = ((dy_dtilt * vx) + (-dx_dtilt * vy)) / det
        tilt_dot = (((-dy_dpan) * vx) + (dx_dpan * vy)) / det
        return pan_dot, tilt_dot

    def A_times(pan_dot: float, tilt_dot: float) -> tuple[float, float]:
        # A * [pan_dot, tilt_dot]
        vx = (dx_dpan * pan_dot) + (dx_dtilt * tilt_dot)
        vy = (dy_dpan * pan_dot) + (dy_dtilt * tilt_dot)
        return vx, vy

    while True:
        now = time.time()
        dt = _clamp(now - last_loop_t, 1e-3, 0.25)
        last_loop_t = now

        pkt = receiver.get_latest()
        if pkt is not None:
            last_pkt_time = now

        have_face = pkt is not None and pkt.detected and pkt.confidence >= min_conf

        if not have_face:
            # Lost
            if (now - last_pkt_time) > lost_timeout_s:
                # Smoothly ramp rates to zero
                pan_rate *= max(0.0, 1.0 - 6.0 * dt)
                tilt_rate *= max(0.0, 1.0 - 6.0 * dt)

                # Optional return to center
                if (now - last_face_time) > return_to_center_delay:
                    tgt_pan = float(pan_cfg.get("center", 0.0))
                    tgt_tilt = float(tilt_cfg.get("center", 0.0))
                    if abs(pan - tgt_pan) > 0.2 or abs(tilt - tgt_tilt) > 0.2:
                        pan = tgt_pan
                        tilt = tgt_tilt
                        if dry_run:
                            print("[CENTER] no face -> center")
                        else:
                            motors.move_pan_tilt(
                                pan,
                                tilt,
                                pan_speed=_clamp(float(pan_cfg.get("speed", 15.0)) * 0.7, 0.5, max_speed),
                                tilt_speed=_clamp(float(tilt_cfg.get("speed", 12.0)) * 0.7, 0.5, max_speed),
                                sync=sync,
                            )
                    last_face_time = now

                time.sleep(0.01)
                continue

        # Safety: ensure pkt is still valid
        if pkt is None:
            time.sleep(0.01)
            continue

        last_face_time = now

        # Measurement filtering
        ex = float(pkt.x)
        ey = float(pkt.y)

        if not filt_init:
            ex_f, ey_f = ex, ey
            filt_init = True
        else:
            a = meas_ema
            ex_f = (a * ex_f) + ((1.0 - a) * ex)
            ey_f = (a * ey_f) + ((1.0 - a) * ey)

        # Delay compensation: predict how our own current motion changes the image error.
        # e_comp = e + (A * rate) * delay
        vx_self, vy_self = A_times(pan_rate, tilt_rate)
        ex_c = _clamp(ex_f + vx_self * delay_comp_s, -1.0, 1.0)
        ey_c = _clamp(ey_f + vy_self * delay_comp_s, -1.0, 1.0)

        # Deadzone handling: drive rates to zero and unwind integral.
        if abs(ex_c) < deadzone and abs(ey_c) < deadzone:
            if integral_leak > 1e-9:
                leak = _clamp(integral_leak * dt, 0.0, 1.0)
                int_ex *= (1.0 - leak)
                int_ey *= (1.0 - leak)
            else:
                int_ex *= max(0.0, 1.0 - 10.0 * dt)
                int_ey *= max(0.0, 1.0 - 10.0 * dt)

            # Ramp rates to zero (accel-limited)
            pan_rate += _clamp(-pan_rate, -max_pan_accel * dt, max_pan_accel * dt)
            tilt_rate += _clamp(-tilt_rate, -max_tilt_accel * dt, max_tilt_accel * dt)
        else:
            # Compute PI in image domain
            # Desired image velocity (normalized/sec)
            vx_cmd = (-kp * ex_c) - (ki * int_ex)
            vy_cmd = (-kp * ey_c) - (ki * int_ey)

            # Map to motor rates (deg/sec)
            pan_cmd, tilt_cmd = invA_times(vx_cmd, vy_cmd)

            if invert_pan:
                pan_cmd *= -1.0
            if invert_tilt:
                tilt_cmd *= -1.0

            # Saturate motor rate commands
            pan_cmd_sat = _clamp(pan_cmd, -max_pan_rate, max_pan_rate)
            tilt_cmd_sat = _clamp(tilt_cmd, -max_tilt_rate, max_tilt_rate)

            # Conditional integration (anti-windup):
            # If saturated and error pushes further into saturation, freeze integral.
            can_int_x = True
            can_int_y = True

            if pan_cmd != pan_cmd_sat and _sign(ex_c) == _sign(pan_cmd):
                can_int_x = False
            if tilt_cmd != tilt_cmd_sat and _sign(ey_c) == _sign(tilt_cmd):
                can_int_y = False

            if can_int_x:
                int_ex = _clamp(int_ex + ex_c * dt, -integral_limit, integral_limit)
            if can_int_y:
                int_ey = _clamp(int_ey + ey_c * dt, -integral_limit, integral_limit)

            # Accel-limit the actual executed rates toward the saturated targets
            pan_rate += _clamp(pan_cmd_sat - pan_rate, -max_pan_accel * dt, max_pan_accel * dt)
            tilt_rate += _clamp(tilt_cmd_sat - tilt_rate, -max_tilt_accel * dt, max_tilt_accel * dt)

        # Integrate rates to create the continuous target trajectory
        pan = _clamp(pan + pan_rate * dt, float(pan_cfg.get("min", -999.0)), float(pan_cfg.get("max", 999.0)))
        tilt = _clamp(tilt + tilt_rate * dt, float(tilt_cfg.get("min", -999.0)), float(tilt_cfg.get("max", 999.0)))

        # Decide whether to send a new short segment now
        if (now - last_sent_t) < segment_dt_s:
            time.sleep(max(0.0, loop_dt - (time.time() - now)))
            continue

        if move_cooldown_s > 1e-6 and (now - last_cmd_t) < move_cooldown_s:
            time.sleep(max(0.0, loop_dt - (time.time() - now)))
            continue

        if queue_limit_s > 1e-6:
            queued_s = max(0.0, commanded_until - now)
            if queued_s >= queue_limit_s:
                time.sleep(max(0.0, loop_dt - (time.time() - now)))
                continue

        # Compute speed so the move can execute within ~segment_dt_s (smoothly)
        step_pan = pan - motors.current_pan
        step_tilt = tilt - motors.current_tilt

        # If we're commanding extremely tiny deltas, skip to reduce HTTP spam.
        if abs(step_pan) < 1e-4 and abs(step_tilt) < 1e-4:
            last_sent_t = now
            time.sleep(max(0.0, loop_dt - (time.time() - now)))
            continue

        pan_speed = _clamp(max(abs(step_pan) / max(1e-3, segment_dt_s), abs(pan_rate)), min_speed, max_speed)
        tilt_speed = _clamp(max(abs(step_tilt) / max(1e-3, segment_dt_s), abs(tilt_rate)), min_speed, max_speed)

        if dry_run:
            if now - last_print > 0.5:
                print(
                    f"[DRY] e=({ex_f:+.3f},{ey_f:+.3f}) e*=( {ex_c:+.3f},{ey_c:+.3f} ) "
                    f"rate=({pan_rate:+.1f},{tilt_rate:+.1f}) deg/s "
                    f"pos=({pan:+.2f},{tilt:+.2f}) "
                    f"spd=({pan_speed:.1f},{tilt_speed:.1f})"
                )
                last_print = now
        else:
            if combine_moves:
                motors.move_pan_tilt(pan, tilt, pan_speed=pan_speed, tilt_speed=tilt_speed, sync=sync)
            else:
                motors.move_pan(pan, speed=pan_speed, sync=sync)
                motors.move_tilt(tilt, speed=tilt_speed, sync=sync)

        # Update queue horizon estimate
        est_t = max(abs(step_pan) / max(0.01, pan_speed), abs(step_tilt) / max(0.01, tilt_speed))
        last_cmd_t = now
        last_sent_t = now
        commanded_until = max(commanded_until, now) + est_t

        # Periodic status
        if now - last_print > 1.0:
            queued_s = max(0.0, commanded_until - now)
            print(
                f"[STAT] e=({ex_f:+.3f},{ey_f:+.3f}) rate=({pan_rate:+.1f},{tilt_rate:+.1f}) "
                f"pos=({pan:+.2f},{tilt:+.2f}) queued={queued_s:.2f}s conf={pkt.confidence:.2f} stab={pkt.stability:.2f}"
            )
            last_print = now

        # Sleep to respect loop rate
        time.sleep(max(0.0, loop_dt - (time.time() - now)))


def main() -> int:
    ap = argparse.ArgumentParser(description="Cascaded PI visual-servo head tracker")
    ap.add_argument(
        "--config",
        default=str(Path(__file__).parent.parent / "config.json"),
        help="Path to config.json",
    )
    ap.add_argument("--dry-run", action="store_true", help="Do not move motors")
    ap.add_argument("--sync", action="store_true", help="Use SYNC=1 (blocking moves)")

    args = ap.parse_args()
    return run(Path(args.config), dry_run=bool(args.dry_run), sync=bool(args.sync))


if __name__ == "__main__":
    raise SystemExit(main())
