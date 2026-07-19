# VR Teleoperation Handoff — 2026-07-15

## Current user goal and scope

The user is improving PICO VR teleoperation for the dual-arm robot. The current
work is deliberately staged:

1. Fix intermittent right-arm non-response caused by Servo activation timing.
2. Add only enough logging to distinguish bridge output from Servo rejection.
3. Next, replace the current per-frame pose deadband with a multi-frame,
   time-normalized velocity estimate for anti-jitter control.
4. Handle rotation-frame calibration separately and last.

Do not change the rotation mapping while working on the deadband task. Follow
the repository `AGENTS.md` risk policy and stop for user verification after a
working simulation result.

## Evidence and diagnosis already completed

The main recording examined was:

```text
/home/dell/dual_arm/vr_teleop_bridge/log/20260714_013541
```

The right-hand log contains five obvious lateral sweeps after 38 seconds. Each
sweep moved the controller roughly 0.20–0.33 m while the right end effector did
not move. Every sweep was performed immediately after pressing Grip and lasted
about 1.0–1.8 seconds.

The previous bridge behavior paused Servo on every Grip release, switched and
unpaused it on the next press, then discarded another second of commands via
`servo_command_settle_time`. The user's movement was usually finished before
commands became admissible.

The existing position deadband is also too large for a per-message comparison:

```yaml
publish_rate: 50.0
deadband_position: 0.005
deadband_rotation: 0.02
max_linear_speed: 0.3
max_angular_speed: 0.5
```

At 50 Hz, these are approximately 0.25 m/s and 1 rad/s instantaneous velocity
thresholds. In particular, the angular threshold is already above the maximum
configured angular speed, producing nearly binary rotation response.

## Completed and committed change

Commit:

```text
da6342e fix: keep VR Servo active across Grip cycles
```

Changed files:

```text
vr_teleop_bridge/vr_teleop_bridge/vr_pose_to_servo_node.py
vr_teleop_bridge/vr_teleop_bridge/trajectory_logger_node.py
vr_teleop_bridge/config/vr_teleop_bridge.yaml
vr_teleop_bridge/config/trajectory_logger.yaml
```

### Servo lifecycle now

- Both enabled Servo nodes start once after complete `/joint_states`, Servo
  service discovery, and the startup-only CurrentStateMonitor settle period.
- Servo remains active while Grip is released.
- Grip press clears the previous VR pose so the first pose becomes a fresh
  clutch reference.
- Grip release immediately publishes zero Twist and clears pose history. It no
  longer calls the Servo pause service.
- `servo_command_settle_time` was removed. There is no per-Grip delay.
- Stale or incomplete joint state remains a safety fault: the affected Servo is
  paused and may restart after state freshness recovers.
- A VR status message containing `disable` or `stop` stops motion but does not
  tear down the normal always-active Servo lifecycle.

### Minimal diagnostic logging now

Each log session has two additional files:

```text
control_left.csv
control_right.csv
```

Important columns:

```text
twist_messages
nonzero_twist_messages
linear_x linear_y linear_z
angular_x angular_y angular_z
servo_status_code
servo_status_message
```

Interpretation:

- `nonzero_twist_messages == 0`: the bridge did not issue a motion command in
  that logger interval. Check Grip, stale input, deadband, or bridge logic.
- `nonzero_twist_messages > 0` and nonzero Servo status: the bridge sent motion,
  while Servo reported a limitation or halt.
- `nonzero_twist_messages > 0`, Servo status `0 / No warnings`, and no end
  effector motion: investigate after the bridge, such as Servo robot state or
  the controller path.
- A blank Servo status means Servo had not published a status yet; it does not
  mean the bridge failed to publish.

Jazzy `moveit_msgs/msg/ServoStatus` codes used by the log are:

```text
-1 invalid
 0 no warning
 1 decelerate approaching singularity
 2 halt for singularity
 3 decelerate leaving singularity
 4 decelerate for collision
 5 halt for collision
 6 joint bound
```

## Verification already performed

Static and build checks passed:

```bash
python3 -m py_compile \
  vr_teleop_bridge/vr_teleop_bridge/vr_pose_to_servo_node.py \
  vr_teleop_bridge/vr_teleop_bridge/trajectory_logger_node.py

colcon build --symlink-install --packages-select vr_teleop_bridge
```

An end-to-end mock-hardware test published a synthetic right-hand trajectory.
Observed result after the repository's existing mock state kick:

```text
bridge Twist: approximately +0.295 m/s on linear x
Servo status: 0 / No warnings
right end-effector displacement: approximately 0.08 m
Grip release: zero Twist published immediately
Servo after release: remained active
```

All synthetic test scripts and generated test log directories were removed.
No test runtime was intentionally left running.

## Important mock-hardware caveat

MoveIt Servo 2.12.4 can remain at:

```text
Waiting to receive robot state update.
```

when mock joint states remain numerically static. This is an existing workspace
issue, not a regression from the always-active lifecycle. The repository already
contains the isolated workaround:

```bash
python3 tools/kick_servo_state_monitor.py
```

It injects and restores a 1e-6 rad mock JointState change. Do not fold this
workaround into the VR bridge or use it on real hardware without its explicit
real-hardware acknowledgement. In the verification run, Servo ignored valid
nonzero Twist before the kick; after the kick, both waiting messages stopped and
the right arm moved.

## Next task: multi-frame anti-jitter velocity deadband

The user now understands and accepts the multi-frame direction, but no exact
thresholds have been selected. The important clarification is:

- Velocity must still be estimated from pose differences. It cannot come from
  one pose sample.
- Merely changing the order from `compare delta, then divide by dt` to `divide
  by dt, then compare velocity` is mathematically equivalent when `dt` is fixed.
- The actual improvement comes from a time-normalized multi-frame estimate,
  noise averaging, and hysteresis.

Recommended initial design:

1. Add a short pose history per `ArmState`, covering about 80–100 ms.
2. While Grip is held, estimate average linear velocity using the current pose
   and an old sample near the window boundary:

   ```text
   v = mapped_position_delta / window_dt
   ```

3. Estimate angular velocity over the same window:

   ```text
   omega = mapped_rotvec(current_q * inverse(old_q)) / window_dt
   ```

4. Apply separate start and stop thresholds (hysteresis), for example a start
   threshold larger than the stop threshold. Do not treat the example numbers
   below as calibrated values:

   ```yaml
   linear_velocity_start: 0.02
   linear_velocity_stop: 0.01
   angular_velocity_start: 0.05
   angular_velocity_stop: 0.025
   velocity_window: 0.10
   ```

5. Keep updating the sample history while commands are filtered. Clear history
   and hysteresis state on Grip edges, stale input, Servo safety deactivation,
   and status stop.
6. Preserve the existing translation and rotation axis maps for this task.
7. Preserve current speed limits initially. Consider norm-based clamping as a
   later, separately verified refinement; current code clamps components.

Before settling thresholds, record 5–10 seconds with the controller held still
and measure the multi-frame estimated velocity noise. Set start thresholds above
the observed noise, then validate deliberately slow motion. Avoid guessing from
the old 5 Hz trajectory logger because it does not contain the bridge's raw 50 Hz
velocity estimate.

The tradeoff of an 80–100 ms window is corresponding smoothing latency. If that
feels too delayed, shorten the window only after confirming jitter remains
controlled.

## Rotation work explicitly deferred

Do not solve rotation calibration during the velocity-deadband change.

What is currently known:

- Unity converts controller pose using `ROSGeometry.To<FLU>()` before publish.
- Logged robot rotation components correlate most strongly with the same VR
  component, so the old recording does not support a simple x/y rotation swap.
- Translation currently maps `[VR y, VR x, VR z]`, while rotation remains
  identity. The translation map is a reflection rather than a valid rigid
  rotation, so a future unified tracking-origin-to-`base_link` calibration is
  still needed.
- The future rotation task should use labelled, single-axis recordings and a
  proper rotation matrix applied consistently to linear and angular vectors.

## Suggested start for the next conversation

1. Read `/home/dell/dual_arm/AGENTS.md`.
2. Read this handoff document completely.
3. Check `git status --short` and the two latest commits.
4. Do not redo the original CSV diagnosis.
5. Implement only the multi-frame velocity estimate and hysteresis in simulation.
6. Extend logging only if the new estimator itself needs one or two raw diagnostic
   fields; the user explicitly does not want a large logging expansion.
7. Build `vr_teleop_bridge`, run an isolated mock test using the existing state
   kick when necessary, and ask the user to verify before touching rotation.
