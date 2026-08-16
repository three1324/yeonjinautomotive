<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: ukf-sensor-fusion
description: 'Configure robot_localization UKF. Use when using unscented Kalman filter for non-linear sensor fusion.'
---

# UKF Sensor Fusion — robot_localization Unscented Kalman Filter

## Overview

The `robot_localization` package also provides `ukf_filter_node`, which implements an **Unscented Kalman Filter**. It uses the exact same configuration format, state vector, and input conventions as the EKF node. The difference is entirely in how the filter approximates the posterior distribution.

---

## EKF vs UKF: Algorithm Differences

| Aspect | EKF | UKF |
|--------|-----|-----|
| Approximation method | Linearizes via Jacobian matrices | Uses sigma points (deterministic sampling) |
| Jacobian computation | Required (analytical or numerical) | Not required |
| Nonlinearity handling | First-order Taylor expansion — poor for highly nonlinear systems | Captures up to second-order statistics — better for nonlinear dynamics |
| Computational cost | Lower (matrix operations on state) | Higher (propagates 2n+1 sigma points, where n=15 → 31 points) |
| Practical difference for ground robots | Excellent for diff-drive at low-moderate speeds | Marginal improvement unless dynamics are highly nonlinear |

### When to Choose UKF Over EKF

- **Aggressive maneuvering**: rapid turns, high angular velocities where linearization breaks down
- **Non-differential-drive robots**: Ackermann steering, mecanum wheels, or omnidirectional drives with complex kinematics
- **High update rates with noisy sensors**: UKF's sigma-point propagation handles nonlinear noise transforms better
- **If EKF diverges**: switching to UKF is a low-cost experiment since configuration is identical

For most indoor diff-drive robots moving at walking speed, EKF and UKF produce nearly identical results. EKF is the default recommendation.

---

## UKF-Specific Parameters

```yaml
alpha: 0.001   # Spread of sigma points around the mean (small = tight, large = wide)
kappa: 0.0     # Secondary scaling parameter (typically 0 or 3-n)
beta: 2.0      # Incorporates prior knowledge of the distribution (2 = optimal for Gaussian)
```

- **`alpha`**: Controls how far sigma points are spread from the mean. Smaller values keep points closer to the mean (better for mildly nonlinear systems). Larger values spread them further (better for highly nonlinear). Default 0.001 works for most cases.
- **`kappa`**: Secondary scaling, usually left at 0. Some literature suggests `3 - n` (where n is state dimension, so 3 - 15 = -12), but 0 is standard.
- **`beta`**: Encodes prior distribution knowledge. For Gaussian distributions, `beta = 2` is optimal.

---

## Complete UKF Configuration

The input structure is identical to EKF. Only the node name and UKF-specific parameters differ:

```yaml
ukf_filter_node:
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

    # UKF-specific sigma point parameters
    alpha: 0.001
    kappa: 0.0
    beta: 2.0

    map_frame: "map"
    odom_frame: "odom"
    base_link_frame: "base_link"
    world_frame: "odom"

    # Wheel odometry — identical config to EKF
    odom0: "odom/unfiltered"
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10

    # IMU — identical config to EKF
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
```

---

## Launch File Usage

Switch between EKF and UKF by changing the node executable:

```python
from launch_ros.actions import Node

# EKF version
ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[ekf_params_file],
)

# UKF version — drop-in replacement
ukf_node = Node(
    package='robot_localization',
    executable='ukf_node',
    name='ukf_filter_node',
    parameters=[ukf_params_file],
)
```

---

## Practical Comparison

Run both simultaneously on recorded data (different output topics) to compare:

```bash
ros2 bag play my_recording.db3
# In separate terminals:
ros2 run robot_localization ekf_node --ros-args -p "odom_frame:=odom_ekf" -r "odometry/filtered:=odom_ekf/filtered"
ros2 run robot_localization ukf_node --ros-args -p "odom_frame:=odom_ukf" -r "odometry/filtered:=odom_ukf/filtered"
```

Plot both `/odom_ekf/filtered` and `/odom_ukf/filtered` in RViz or PlotJuggler. For most diff-drive scenarios, the traces will nearly overlap — confirming that EKF is sufficient.
