#!/usr/bin/env python3
"""Apply tracking/motion tuning presets to config.json.

This avoids hard-to-copy multi-line terminal snippets.

Usage:
  python3 rpi/tune_presets.py glide
  python3 rpi/tune_presets.py balanced
  python3 rpi/tune_presets.py snappy

Optional:
  python3 rpi/tune_presets.py glide --config /path/to/config.json
  python3 rpi/tune_presets.py --show
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict


PRESETS: Dict[str, Dict[str, Any]] = {
    # Smoothest, may lag slightly.
    "glide": {
        "tracking": {
            "deadzone": 0.15,
        },
        "tracking_decoupled": {
            # Faster than the original "glide" (less lag), but still very smooth.
            "alpha": 0.75,
            "move_cooldown_s": 0.06,
            "speed_scaling": False,
            "cmd_smoothing": 0.88,
            "deadzone_hysteresis": 0.04,
            "require_outside_samples": 3,
            "max_step_pan_deg": 0.28,
            "max_step_tilt_deg": 0.11,
            "min_step_pan_deg": 0.03,
            "min_step_tilt_deg": 0.02,
            "pan_speed": 13.0,
            "tilt_speed": 9.0,
        },
        "motors": {
            "pan": {"accel": 25},
            "tilt": {"accel": 10},
        },
    },
    # Default recommendation.
    "balanced": {
        "tracking": {
            "deadzone": 0.15,
        },
        "tracking_decoupled": {
            # Faster convergence than glide, still smooth.
            "alpha": 0.82,
            "move_cooldown_s": 0.06,
            "speed_scaling": False,
            "cmd_smoothing": 0.80,
            "deadzone_hysteresis": 0.04,
            "require_outside_samples": 3,
            "max_step_pan_deg": 0.38,
            "max_step_tilt_deg": 0.15,
            "min_step_pan_deg": 0.04,
            "min_step_tilt_deg": 0.03,
            "pan_speed": 16.0,
            "tilt_speed": 12.0,
        },
        "motors": {
            "pan": {"accel": 30},
            "tilt": {"accel": 12},
        },
    },
    # Good speed like snappy, but reduces tiny shaky corrections.
    "fastsmooth": {
        "tracking": {
            # Slightly larger deadzone to prevent micro-dither.
            "deadzone": 0.17,
        },
        "tracking_decoupled": {
            # Force classic position control (no rate/Jetson-velocity influence)
            "control_mode": "position",
            "use_velocity_prediction": False,
            "lookahead_s": 0.0,
            # Disable sweep + anti-backlash experiments for this preset.
            "sweep_enabled": False,
            "reverse_deadband_pan_deg": 0.0,
            "reverse_deadband_tilt_deg": 0.0,

            "alpha": 0.95,
            # Small extra cooldown reduces tiny high-rate corrective pulses.
            "move_cooldown_s": 0.07,
            # NEW: hold measurements briefly after we command a move.
            # This prevents chasing the box while the camera is still moving.
            "hold_measurements_during_motion": True,
            "post_move_settle_s": 0.08,
            # Constant speeds (no per-step speed jumps) typically looks smoother.
            "speed_scaling": False,
            # A bit of command smoothing to glide, but not so much that it lags.
            "cmd_smoothing": 0.78,
            # More measurement smoothing reduces sign-flip chatter.
            "ema": 0.90,
            # Dither suppression.
            "deadzone_hysteresis": 0.06,
            "require_outside_samples": 5,
            # Allow larger steps so it recenters quickly.
            "max_step_pan_deg": 0.55,
            "max_step_tilt_deg": 0.22,
            # Ignore tiny micro-corrections that look like shaking.
            "min_step_pan_deg": 0.09,
            "min_step_tilt_deg": 0.07,
            "pan_speed": 18.0,
            "tilt_speed": 14.0,
        },
        "motors": {
            "pan": {"accel": 35},
            "tilt": {"accel": 14},
        },
    },
    # Fastest tracking; a bit less glide.
    "snappy": {
        "tracking": {
            "deadzone": 0.15,
        },
        "tracking_decoupled": {
            "alpha": 1.00,
            "move_cooldown_s": 0.05,
            "speed_scaling": True,
            "cmd_smoothing": 0.55,
            "deadzone_hysteresis": 0.04,
            "require_outside_samples": 3,
            "max_step_pan_deg": 0.60,
            "max_step_tilt_deg": 0.24,
            "min_step_pan_deg": 0.05,
            "min_step_tilt_deg": 0.04,
            "pan_speed": 20.0,
            "tilt_speed": 16.0,
        },
        "motors": {
            "pan": {"accel": 45},
            "tilt": {"accel": 16},
        },
    },

    # Prioritizes a single longer sweep when far off-center.
    "sweep": {
        "tracking": {
            "deadzone": 0.11,
        },
        "tracking_decoupled": {
            "use_velocity_prediction": False,
            "lookahead_s": 0.0,
            "sweep_enabled": True,
            "sweep_error": 0.24,
            # Dynamic boost only (non-blocking): bigger steps/speeds when far.
            "sweep_max_step_pan_deg": 1.6,
            "sweep_max_step_tilt_deg": 0.6,
            "sweep_pan_speed": 60.0,
            "sweep_tilt_speed": 40.0,

            # Normal tracking after the sweep.
            "alpha": 0.95,
            "ema": 0.75,
            "move_cooldown_s": 0.035,
            "speed_scaling": False,
            # Smooth near center, more responsive when far.
            "cmd_smoothing": 0.68,
            "cmd_smoothing_near": 0.72,
            "cmd_smoothing_far": 0.45,
            "deadzone_hysteresis": 0.02,
            "require_outside_samples": 2,
            "max_step_pan_deg": 0.8,
            "max_step_tilt_deg": 0.32,
            "min_step_pan_deg": 0.04,
            "min_step_tilt_deg": 0.03,
            "pan_speed": 60.0,
            "tilt_speed": 40.0,
            "max_pan_speed": 70.0,
            "max_tilt_speed": 48.0,
        },
        "motors": {
            "pan": {"accel": 20},
            "tilt": {"accel": 10},
        },
    },

    # Gimbal-like feel WITHOUT risking oscillation:
    # this uses the proven position-based decoupled controller plus the
    # non-blocking sweep scaling when far from center.
    "gimbal": {
        "tracking": {
            "deadzone": 0.11,
        },
        "tracking_decoupled": {
            "use_velocity_prediction": False,
            "lookahead_s": 0.0,
            "control_mode": "position",

            # Non-blocking dynamic sweep scaling when far.
            "sweep_enabled": True,
            "sweep_error": 0.24,
            "sweep_max_step_pan_deg": 1.6,
            "sweep_max_step_tilt_deg": 0.6,
            "sweep_pan_speed": 80.0,
            "sweep_tilt_speed": 55.0,

            # In rate mode we derive SPEED from step/dt; keep scaling off.
            "speed_scaling": False,

            # Keep your current smooth defaults (tweak later if desired).
            "alpha": 0.95,
            "ema": 0.75,
            "move_cooldown_s": 0.035,
            "hold_measurements_during_motion": True,
            "post_move_settle_s": 0.06,
            "cmd_smoothing": 0.68,
            "cmd_smoothing_near": 0.72,
            "cmd_smoothing_far": 0.45,
            "deadzone_hysteresis": 0.03,
            "require_outside_samples": 3,

            # Anti-backlash: prevent tiny sign-flip corrections.
            "reverse_deadband_pan_deg": 0.08,
            "reverse_deadband_tilt_deg": 0.05,

            # Allow decent step size so rate mode can flow.
            "max_step_pan_deg": 0.8,
            "max_step_tilt_deg": 0.32,
            "min_step_pan_deg": 0.04,
            "min_step_tilt_deg": 0.03,

            # IMPORTANT: override any previous preset speeds so gimbal can move
            # as a single smooth sweep rather than slow step-by-step.
            "pan_speed": 80.0,
            "tilt_speed": 55.0,
            "max_pan_speed": 90.0,
            "max_tilt_speed": 65.0,
        },

        # Motor accel caps strongly affect “step-step” feel.
        "motors": {
            "pan": {"accel": 20},
            "tilt": {"accel": 10},
        },
    },

    # Experimental: true rate-based control (can oscillate depending on latency).
    "gimbal_rate": {
        "tracking": {
            "deadzone": 0.20,
        },
        "tracking_decoupled": {
            "use_velocity_prediction": False,
            "lookahead_s": 0.0,
            "control_mode": "rate",
            "rate_kp": 1.2,
            "rate_kd": 0.0,
            "rate_use_velocity_feedback": False,
            "rate_motor_ema": 0.88,
            "rate_speed_dt_min_s": 0.08,
            "move_cooldown_s": 0.08,
            "speed_scaling": False,
            "cmd_smoothing": 0.75,
            "deadzone_hysteresis": 0.03,
            "require_outside_samples": 3,
            "max_step_pan_deg": 0.6,
            "max_step_tilt_deg": 0.24,
            "min_step_pan_deg": 0.05,
            "min_step_tilt_deg": 0.04,
        },
    },
}


def _ensure_dict(root: Dict[str, Any], key: str) -> Dict[str, Any]:
    val = root.get(key)
    if not isinstance(val, dict):
        val = {}
        root[key] = val
    return val


def _merge(dst: Dict[str, Any], src: Dict[str, Any]) -> None:
    for k, v in src.items():
        if isinstance(v, dict):
            child = _ensure_dict(dst, k)
            assert isinstance(child, dict)
            _merge(child, v)
        else:
            dst[k] = v


def _show(cfg: Dict[str, Any]) -> None:
    t = cfg.get("tracking", {}) if isinstance(cfg.get("tracking"), dict) else {}
    d = cfg.get("tracking_decoupled", {}) if isinstance(cfg.get("tracking_decoupled"), dict) else {}
    m = cfg.get("motors", {}) if isinstance(cfg.get("motors"), dict) else {}
    pan = m.get("pan", {}) if isinstance(m.get("pan"), dict) else {}
    tilt = m.get("tilt", {}) if isinstance(m.get("tilt"), dict) else {}

    print("Current tuning (selected fields):")
    print(f"  tracking.deadzone               = {t.get('deadzone')}")
    print(f"  tracking_decoupled.alpha        = {d.get('alpha')}")
    print(f"  tracking_decoupled.cmd_smoothing= {d.get('cmd_smoothing')}")
    print(f"  tracking_decoupled.speed_scaling= {d.get('speed_scaling')}")
    print(f"  tracking_decoupled.move_cooldown_s= {d.get('move_cooldown_s')}")
    print(f"  tracking_decoupled.active_preset= {d.get('active_preset')}")
    print(f"  tracking_decoupled.control_mode = {d.get('control_mode')}")
    print(f"  tracking_decoupled.rate_kp      = {d.get('rate_kp')}")
    print(f"  tracking_decoupled.rate_kd      = {d.get('rate_kd')}")
    print(f"  tracking_decoupled.max_step_pan_deg = {d.get('max_step_pan_deg')}")
    print(f"  tracking_decoupled.max_step_tilt_deg= {d.get('max_step_tilt_deg')}")
    print(f"  tracking_decoupled.pan_speed     = {d.get('pan_speed')}")
    print(f"  tracking_decoupled.tilt_speed    = {d.get('tilt_speed')}")
    print(f"  motors.pan.accel                 = {pan.get('accel')}")
    print(f"  motors.tilt.accel                = {tilt.get('accel')}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Apply smooth-tracking tuning presets")
    ap.add_argument(
        "preset",
        nargs="?",
        choices=sorted(PRESETS.keys()),
        help="Preset name: " + ", ".join(sorted(PRESETS.keys())),
    )
    ap.add_argument(
        "--config",
        default=str(Path(__file__).parent.parent / "config.json"),
        help="Path to config.json",
    )
    ap.add_argument("--show", action="store_true", help="Print current tuning fields")

    args = ap.parse_args()
    config_path = Path(args.config)

    cfg = json.loads(config_path.read_text())

    if args.show:
        _show(cfg)
        return 0

    if not args.preset:
        ap.error("preset is required unless --show is used")

    preset = PRESETS[args.preset]
    _merge(cfg, preset)

    # Stamp which preset was last applied (helps avoid confusion).
    td = _ensure_dict(cfg, "tracking_decoupled")
    td["active_preset"] = args.preset

    config_path.write_text(json.dumps(cfg, indent=2) + "\n")
    print(f"Applied preset: {args.preset}")
    print(f"Wrote: {config_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
