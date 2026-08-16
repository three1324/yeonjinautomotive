<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: ekf-sensor-fusion
description: 'Configure robot_localization EKF. Use when fusing odometry, IMU, GPS, or visual odometry with the extended Kalman filter.'
---

# EKF Sensor Fusion — robot_localization Extended Kalman Filter

## Overview

The `robot_localization` package provides `ekf_filter_node`, which fuses multiple odometry, IMU, pose, and twist sources into a single, smooth state estimate. It maintains a 15-dimensional state vector and publishes a filtered `nav_msgs/Odometry` message plus a TF transform.

---

## State Vector

The EKF tracks 15 state dimensions:

| Index | Dimension | Description |
|-------|-----------|-------------|
| 0 | x | Position X |
| 1 | y | Position Y |
| 2 | z | Position Z |
| 3 | roll | Orientation roll |
| 4 | pitch | Orientation pitch |
| 5 | yaw | Orientation yaw |
| 6 | vx | Linear velocity X |
| 7 | vy | Linear velocity Y |
| 8 | vz | Linear velocity Z |
| 9 | vroll | Angular velocity roll |
| 10 | vpitch | Angular velocity pitch |
| 11 | vyaw | Angular velocity yaw |
| 12 | ax | Linear acceleration X |
| 13 | ay | Linear acceleration Y |
| 14 | az | Linear acceleration Z |

Each input source specifies a **15-element boolean vector** declaring which dimensions it contributes.

---

## Input Naming Convention

Sources are numbered sequentially per type:

| Prefix | Message Type | Examples |
|--------|-------------|----------|
| `odom0`, `odom1`, ... | `nav_msgs/Odometry` | Wheel odometry, visual odometry |
| `imu0`, `imu1`, ... | `sensor_msgs/Imu` | IMU sensors |
| `pose0`, `pose1`, ... | `geometry_msgs/PoseWithCovarianceStamped` | AMCL pose, GPS converted to local |
| `twist0`, `twist1`, ... | `geometry_msgs/TwistWithCovarianceStamped` | External velocity estimates |

For each source, you configure:

| Parameter | Purpose |
|-----------|---------|
| `<source>` | Topic name |
| `<source>_config` | 15-element boolean vector (which dimensions to fuse) |
| `<source>_differential` | If true, converts absolute poses to velocity-like measurements |
| `<source>_relative` | If true, treats the first measurement as the origin |
| `<source>_queue_size` | Subscriber queue depth |
| `<source>_rejection_threshold` | Mahalanobis distance threshold to reject outliers |

---

## Differential Drive + IMU Configuration

For a typical differential-drive robot with wheel encoders and a single IMU:

### Wheel Odometry (odom0)

Fuse **velocities only** — not positions. The EKF integrates velocities into position internally. If you fuse position from wheel odom, the EKF fights between its internal integration and the raw odometry position.

```yaml
odom0: "odom/unfiltered"
odom0_config: [false, false, false,   # x, y, z — do NOT fuse position
               false, false, false,   # roll, pitch, yaw — not from wheel odom
               true,  false, false,   # vx — YES, vy — no (diff-drive), vz — no
               false, false, true,    # vroll — no, vpitch — no, vyaw — YES
               false, false, false]   # ax, ay, az — not from wheel odom
odom0_differential: false
odom0_relative: false
odom0_queue_size: 10
```

### IMU (imu0)

Fuse absolute yaw orientation, angular velocity about z, and forward linear acceleration:

```yaml
imu0: "imu/data"
imu0_config: [false, false, false,   # x, y, z — IMU has no position
              false, false, true,    # roll — no, pitch — no, yaw — YES (absolute heading)
              false, false, false,   # vx, vy, vz — not from IMU
              false, false, true,    # vroll — no, vpitch — no, vyaw — YES
              true,  false, false]   # ax — YES, ay — no, az — no
imu0_differential: false
imu0_relative: false
imu0_queue_size: 10
imu0_remove_gravitational_acceleration: true
```

### Critical Rule: No Double-Fusing

**Never fuse the same physical dimension from two sources both as absolute values.** If wheel odom provides vyaw and IMU provides vyaw, this is acceptable because they are independent measurements of the same quantity — the EKF will weight them by covariance. But if you fuse absolute yaw from both wheel odom AND IMU, the estimates will fight.

The safe pattern:
- Wheel odom → velocities (vx, vyaw)
- IMU → absolute orientation (yaw), angular velocity (vyaw), linear acceleration (ax)

---

## Key Parameters

### Frame Configuration

```yaml
map_frame: "map"              # Global fixed frame (only used if world_frame == map_frame)
odom_frame: "odom"            # Continuous odometry frame
base_link_frame: "base_link"  # Robot body frame
world_frame: "odom"           # Which frame the EKF operates in (odom for local, map for global)
```

Set `world_frame: "odom"` for a local-frame EKF (typical). The EKF then publishes the `odom → base_link` transform. If using a two-EKF pattern with a global EKF, set `world_frame: "map"` for the global instance.

### Timing and Rate

```yaml
frequency: 30.0                 # EKF update rate in Hz (30-50 typical)
sensor_timeout: 0.1             # Seconds before a sensor is considered stale
two_d_mode: true                # Constrain to 2D: z, roll, pitch forced to zero
transform_time_offset: 0.0      # Future-date the TF transform by this many seconds
transform_timeout: 0.0          # TF lookup timeout
publish_tf: true                # Publish the odom→base_link transform
publish_acceleration: false     # Publish acceleration in the output Odometry
predict_to_current_time: false  # Extrapolate state to current time (helps with TF latency)
```

**`two_d_mode: true`** is essential for ground robots. It locks z, roll, and pitch to zero, preventing the EKF from drifting in dimensions that a differential-drive robot doesn't traverse.

**`predict_to_current_time`**: Enable this if you see TF extrapolation warnings. It forward-predicts the state using the process model, reducing perceived TF lag.

### Process Noise Covariance

The `process_noise_covariance` is a 15×15 diagonal matrix (stored row-major) that controls how much the EKF trusts its process model vs. sensor measurements. Higher values = less trust in the model = more responsive to sensor inputs but noisier output.

```yaml
process_noise_covariance: [
  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015
]
```

### Initial Estimate Covariance

```yaml
initial_estimate_covariance: [
  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,
  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9
]
```

---

## Complete YAML Configuration

```yaml
ekf_filter_node:
  ros__parameters:
    use_sim_time: false
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true
    publish_acceleration: false
    predict_to_current_time: false
    transform_time_offset: 0.0
    transform_timeout: 0.0

    map_frame: "map"
    odom_frame: "odom"
    base_link_frame: "base_link"
    world_frame: "odom"

    # Wheel odometry — fuse forward velocity and yaw rate
    odom0: "odom/unfiltered"
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10

    # IMU — fuse absolute yaw, yaw rate, and forward acceleration
    imu0: "imu/data"
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  true,  false, false]
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: true

    process_noise_covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015]

    initial_estimate_covariance: [1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-9]
```

---

## Debugging Tips

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| EKF output oscillates rapidly | Double-fusing the same dimension from two absolute sources | Remove one source or switch one to `differential: true` |
| Position drifts even when stationary | IMU acceleration bias not removed, or gravity not subtracted | Set `imu0_remove_gravitational_acceleration: true`, calibrate IMU at rest |
| TF extrapolation warnings | EKF output lags behind sensor timestamps | Enable `predict_to_current_time: true` or increase `transform_time_offset` |
| Output ignores one sensor entirely | Topic name wrong, or sensor covariance extremely high | Check `ros2 topic hz <topic>`, verify covariance in the message |
| State jumps when sensor reconnects | Stale measurement accepted after timeout | Reduce `sensor_timeout` or increase rejection threshold |

### Diagnostic Topics

- `/diagnostics` — filter health, sensor status
- `/odometry/filtered` — the fused output odometry
- `/set_pose` — manually reset the EKF state (PoseWithCovarianceStamped)
