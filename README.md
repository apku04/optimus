# Optimus Face/Head Tracker — Config Reference

This repo uses **`config.json`** as the runtime configuration (strict JSON).

Because JSON does **not** support comments, the explanations live in:
- **`config.annotated.jsonc`**: commented reference copy (JSONC)
- **`docs/CONFIG_TUNING_GUIDE.txt`**: tuning notes

If you want to change behavior, edit **`config.json`** (or apply a preset with `rpi/tune_presets.py`).

---

## What the controller does (high level)

- Jetson runs face tracking and sends offsets (error) over UDP.
- Raspberry Pi receives those offsets and drives two Klipper `MANUAL_STEPPER`s (pan + tilt).
- The **decoupled** controller uses a calibrated 2×2 matrix to convert image error → motor degrees.

The goal is: keep the face centered **without oscillation**, by combining:
- deadzone + hysteresis (ignore tiny errors)
- smoothing (measurement + command)
- command pacing (cooldowns, hold/settle while moving)
- step limits and speed limits

---

## Editing rules

- **Don’t add comments to `config.json`** (it will break parsing).
- Keep values within reasonable ranges; speed/accel may also be capped by Klipper.

---

## Parameter reference (from `config.json`)

### `network`
- `network.jetson_ip` (string)
  - Jetson IP address (informational / documentation; used if your scripts read it).
- `network.rpi_ip` (string)
  - RPi IP address (informational / documentation).
- `network.udp_port` (int)
  - UDP port used for the Jetson → RPi telemetry.

### `motors`
- `motors.moonraker_url` (string)
  - Moonraker base URL (e.g. `http://localhost:7125`).

#### `motors.pan`
- `motors.pan.stepper` (string)
  - Klipper `MANUAL_STEPPER` name (example: `stepper_0`).
- `motors.pan.center` (float, degrees)
  - “Center” position used by centering logic / homing behavior in scripts.
- `motors.pan.min`, `motors.pan.max` (float, degrees)
  - Hard safety clamp for pan position.
- `motors.pan.speed` (float, deg/s)
  - Default speed used for non-tracking moves (center/calibration utilities). Tracking has its own `pan_speed`.
- `motors.pan.accel` (float)
  - Optional `ACCEL=` used for `MANUAL_STEPPER` moves.
  - Increase → snappier starts/stops; too high → shaking/overshoot.

#### `motors.tilt`
Same as pan:
- `motors.tilt.stepper`
- `motors.tilt.center`
- `motors.tilt.min`, `motors.tilt.max`
- `motors.tilt.speed`
- `motors.tilt.accel`

### `tracking` (high-level / some legacy)
- `tracking.smoothing` (float 0..1)
  - Generic smoothing knob (legacy). For the decoupled controller, the primary smoothing is `tracking_decoupled.ema` and `tracking_decoupled.cmd_smoothing`.
- `tracking.deadzone` (float, normalized)
  - Main “do nothing” zone in normalized image units.
  - Increase → less jitter, less precision.
  - Decrease → more precise centering, more micro-moves.
- `tracking.return_to_center_delay` (seconds)
  - Time without a detected face before returning to `motors.*.center`.
- `tracking.invert_pan`, `tracking.invert_tilt` (bool)
  - Flips axis direction if wiring/geometry is reversed.

### `camera` (informational)
- `camera.width`, `camera.height` (int)
  - Actual camera resolution.
- `camera.fps` (int)
  - Camera FPS (used for expectations / debugging).
- `camera.process_width`, `camera.process_height` (int)
  - Processing resolution (if used by the tracker pipeline).

---

## `tracking_decoupled` (main tuning section)

### Calibration matrix (the “decoupling”)
These values define the mapping from motor motion (degrees) to image offset change:

- `tracking_decoupled.dx_dpan`
- `tracking_decoupled.dy_dpan`
- `tracking_decoupled.dx_dtilt`
- `tracking_decoupled.dy_dtilt`

Interpretation:

\[x; y] \approx A \cdot [pan; tilt]\

If the system moves the *wrong way* or cross-coupling is weird, recalibrate (don’t try to hand-edit unless you know what you’re doing).

### Control mode
- `tracking_decoupled.control_mode` (`"position"` recommended)
  - `"position"`: uses current error and the calibrated inverse.
  - `"rate"`: experimental; more sensitive to latency/backlash.

### Gain / aggressiveness
- `tracking_decoupled.alpha` (float)
  - Main gain in position mode.
  - Higher → faster correction, but can overshoot/oscillate.
- `tracking_decoupled.alpha_near_mult` / `tracking_decoupled.alpha_far_mult`
  - Scales gain based on how far from center you are.
  - Lower `alpha_near_mult` helps prevent bouncing near center.

### Measurement smoothing (stability vs lag)
- `tracking_decoupled.ema` (float 0..0.98)
  - EMA on measured image error.
  - Higher → steadier/noisier suppression, but more lag.

### Deadzone behavior
- `tracking_decoupled.deadzone_hysteresis` (float)
  - Adds a “sticky” stop/start gap so it doesn’t toggle movement constantly.
- `tracking_decoupled.require_outside_samples` (int)
  - Requires N consecutive samples outside deadzone before moving.
  - Higher → less twitchy, more delay.

### Step sizing (how big each correction can be)
- `tracking_decoupled.max_step_pan_deg` / `tracking_decoupled.max_step_tilt_deg`
  - Max correction per update.
  - Higher → fewer, bigger moves (can look smoother), but may overshoot.
- `tracking_decoupled.min_step_pan_deg` / `tracking_decoupled.min_step_tilt_deg`
  - Ignore tiny corrections.
  - Higher → less jitter, but leaves small steady error.

### Command smoothing (reduces “step-step” feel)
- `tracking_decoupled.cmd_smoothing` (float 0..0.98)
  - Smooths commanded target positions.
  - Higher → more glide, but can feel “laggy”.
- `tracking_decoupled.cmd_smoothing_near` / `tracking_decoupled.cmd_smoothing_far`
  - Optional: different smoothing near center vs far.

### Loop cadence & pacing
- `tracking_decoupled.loop_hz` (int)
  - Target control-loop frequency.
  - Higher → more responsive, more commands.
  - Lower → more stable/less load, but more “steppy”.

- `tracking_decoupled.move_cooldown_s` (seconds)
  - Minimum time between sending commands.
  - Too low → command queueing + oscillation.
  - Too high → sluggish.

- `tracking_decoupled.hold_measurements_during_motion` (bool)
  - If true, ignore Jetson measurements while a move is underway (prevents “chasing” while camera is moving).

- `tracking_decoupled.post_move_settle_s` (seconds)
  - Extra time to ignore measurements after motion ends.
  - Increase → less oscillation; decrease → more responsive.

- `tracking_decoupled.combine_moves` (bool)
  - Sends pan+tilt in one Moonraker call. Usually smoother than separate commands.

### Speed and limits
- `tracking_decoupled.pan_speed` / `tracking_decoupled.tilt_speed` (deg/s)
  - Requested `SPEED=` for tracking moves.
- `tracking_decoupled.max_pan_speed` / `tracking_decoupled.max_tilt_speed` (deg/s)
  - Controller clamp.

If speed changes don’t matter, you’re probably limited by Klipper `manual_stepper` velocity/accel caps.

- `tracking_decoupled.speed_scaling` (bool)
  - If true, varies speed based on error/step size.

### Sweep boost (gimbal-like behavior when far)
- `tracking_decoupled.sweep_enabled` (bool)
  - Enables a stronger/faster behavior when error is large.
- `tracking_decoupled.sweep_error` (float)
  - Threshold where sweep boost begins.
- `tracking_decoupled.sweep_max_step_pan_deg` / `tracking_decoupled.sweep_max_step_tilt_deg`
  - Max step size during sweep.
- `tracking_decoupled.sweep_pan_speed` / `tracking_decoupled.sweep_tilt_speed`
  - Speed used during sweep.
- `tracking_decoupled.sweep_hold_s` (seconds)
  - Optional hold behavior in sweep mode (implementation-dependent).

### Backlash / direction flip suppression
- `tracking_decoupled.reverse_deadband_pan_deg` / `tracking_decoupled.reverse_deadband_tilt_deg`
  - If a requested move reverses direction and is smaller than this, skip it.
  - Increase to stop “left-right-left-right” jitter near center.

### Velocity prediction (normally keep OFF)
- `tracking_decoupled.use_velocity_prediction` (bool)
  - Predicts error ahead using reported velocities.
- `tracking_decoupled.lookahead_s` (seconds)
  - Prediction horizon.

This can reintroduce oscillation because moving the camera changes the measurement.

### Experimental rate controller (generally leave alone)
These are present for experiments; position mode is typically more stable.
- `tracking_decoupled.rate_kp`
- `tracking_decoupled.rate_kd`
- `tracking_decoupled.rate_use_velocity_feedback`
- `tracking_decoupled.rate_motor_ema`
- `tracking_decoupled.rate_speed_dt_min_s`

### Calibration bookkeeping (used by calibration tools)
- `tracking_decoupled.step_deg` (degrees)
- `tracking_decoupled.settle_s` (seconds)
- `tracking_decoupled.sample_s` (seconds)
- `tracking_decoupled.timestamp`
- `tracking_decoupled.notes`

### Preset marker
- `tracking_decoupled.active_preset` (string)
  - Name of last applied preset (for sanity checks).

---

## Practical tuning playbooks

### 1) Stop oscillation first (camera is still but it hunts)
1. Increase `tracking.deadzone` a bit.
2. Increase `tracking_decoupled.reverse_deadband_*` and/or `min_step_*`.
3. Increase `tracking_decoupled.require_outside_samples`.
4. Ensure `tracking_decoupled.use_velocity_prediction=false`.
5. Increase `tracking_decoupled.post_move_settle_s` if it still chases during/after moves.

### 2) Make it feel less “step-step-step”
1. Increase `tracking_decoupled.cmd_smoothing`.
2. Increase `tracking_decoupled.max_step_*` slightly.
3. Increase `tracking_decoupled.pan_speed` / `tilt_speed` (if Klipper allows).

### 3) Make it faster without destabilizing
1. Raise `pan_speed` / `tilt_speed`.
2. If no change: raise Klipper manual_stepper velocity/accel caps.
3. If it overshoots: reduce `alpha` or increase `post_move_settle_s`.

---

## Useful commands

- Show current preset/values:
  - `python3 rpi/tune_presets.py --show`
- Run the RPi controller:
  - `python3 rpi/head_tracker_decoupled.py`

---

## Notes

This README describes **what the knobs mean**, not what your “best values” are. Your best values depend on backlash, motor torque, camera latency, and your Klipper limits.
