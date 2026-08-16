<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: path-tracking-metrics
description: 'Measure path tracking performance. Use when computing cross-track error, heading error, or evaluating controller quality.'
---

# Path Tracking Metrics — Measuring Navigation Quality

## Key Metrics

### 1. Cross-Track Error (CTE)

Lateral distance from the robot's actual position to the nearest point on the planned path. This is the primary measure of path-following fidelity.

| Quality | CTE (m) | Interpretation |
|---------|---------|----------------|
| Excellent | < 0.10 | Tight path tracking, suitable for narrow corridors |
| Good | 0.10–0.20 | Normal indoor navigation |
| Needs attention | 0.20–0.30 | Noticeable deviation, may clip obstacles |
| Poor | > 0.30 | Significant wandering, likely critic weight issues |

### 2. Heading Error

Angle between the robot's actual heading and the tangent of the planned path at the nearest point. Measured in radians.

| Quality | Heading Error (rad) | Interpretation |
|---------|--------------------|-|
| Excellent | < 0.1 (~6°) | Robot faces along the path |
| Good | 0.1–0.3 (~6–17°) | Minor heading deviation |
| Poor | > 0.5 (~29°) | Robot is skewed relative to path, likely PathAngle/PathAlign issue |

### 3. Path Length Ratio

`actual_distance_traveled / planned_path_length`. Ratio of 1.0 means the robot followed the exact planned path. Greater than 1.0 means the robot traveled further (shortcuts, oscillations, detours).

| Quality | Ratio | Interpretation |
|---------|-------|----------------|
| Excellent | 1.0–1.05 | Nearly perfect tracking |
| Good | 1.05–1.15 | Minor deviations or smoothing |
| Needs attention | 1.15–1.30 | Significant deviations or oscillations |
| Poor | > 1.30 | Major detours or repeated recoveries |

### 4. Time to Completion

Wall-clock time from receiving the goal to reaching it. Compare against theoretical minimum (path_length / desired_linear_vel) to assess efficiency.

### 5. Recovery Event Count

Number of times recovery behaviors triggered during a single navigation goal. Ideal: 0. If consistently > 0, the controller or costmap needs tuning.

### 6. Average Velocity

Mean linear velocity along the path. Compare to `desired_linear_vel` / `vx_max` to assess whether regulation or obstacles are reducing speed.

## Data Collection

### Required Topics

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/odom` | `nav_msgs/msg/Odometry` | Robot's actual pose and velocity |
| `/plan` | `nav_msgs/msg/Path` | Global planned path |
| `/local_plan` | `nav_msgs/msg/Path` | Controller's local trajectory |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Commanded velocities |
| `/tf` | `tf2_msgs/msg/TFMessage` | Coordinate transforms |

### Recording with rosbag2

```bash
ros2 bag record -o nav_metrics_bag \
  /odom /plan /local_plan /cmd_vel /tf /tf_static \
  --max-bag-duration 300
```

Record during a typical patrol run. Include `/tf` and `/tf_static` for later replay.

## Analysis Script

```python
#!/usr/bin/env python3
"""Compute path tracking metrics from a rosbag2 recording."""

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import math


def nearest_point_on_path(robot_pos, path_points):
    """Find nearest point on path and return distance and index."""
    min_dist = float('inf')
    min_idx = 0
    for i, pp in enumerate(path_points):
        d = math.hypot(robot_pos[0] - pp[0], robot_pos[1] - pp[1])
        if d < min_dist:
            min_dist = d
            min_idx = i
    return min_dist, min_idx


def path_tangent_angle(path_points, idx):
    """Compute path tangent angle at index."""
    if idx >= len(path_points) - 1:
        idx = len(path_points) - 2
    dx = path_points[idx + 1][0] - path_points[idx][0]
    dy = path_points[idx + 1][1] - path_points[idx][1]
    return math.atan2(dy, dx)


def yaw_from_quaternion(q):
    """Extract yaw from quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q[3] * q[2] + q[0] * q[1])
    cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    return math.atan2(siny_cosp, cosy_cosp)


def analyze_bag(bag_path):
    cte_values = []
    heading_errors = []
    velocities = []
    path_points = None
    odom_positions = []

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/plan':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                path_points = [
                    (p.pose.position.x, p.pose.position.y)
                    for p in msg.poses
                ]

            elif connection.topic == '/odom' and path_points:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                pos = msg.pose.pose.position
                orient = msg.pose.pose.orientation
                robot_pos = (pos.x, pos.y)
                odom_positions.append(robot_pos)

                # Cross-track error
                cte, idx = nearest_point_on_path(robot_pos, path_points)
                cte_values.append(cte)

                # Heading error
                yaw = yaw_from_quaternion([
                    orient.x, orient.y, orient.z, orient.w])
                tangent = path_tangent_angle(path_points, idx)
                h_err = abs(math.atan2(
                    math.sin(yaw - tangent), math.cos(yaw - tangent)))
                heading_errors.append(h_err)

                # Velocity
                vel = msg.twist.twist.linear.x
                velocities.append(vel)

    if not cte_values:
        print("No data collected. Check bag contents.")
        return

    # Path length ratio
    actual_dist = sum(
        math.hypot(
            odom_positions[i + 1][0] - odom_positions[i][0],
            odom_positions[i + 1][1] - odom_positions[i][1])
        for i in range(len(odom_positions) - 1)
    )
    planned_dist = sum(
        math.hypot(
            path_points[i + 1][0] - path_points[i][0],
            path_points[i + 1][1] - path_points[i][1])
        for i in range(len(path_points) - 1)
    ) if path_points else 1.0

    print(f"=== Path Tracking Metrics ===")
    print(f"Samples: {len(cte_values)}")
    print(f"Cross-Track Error:")
    print(f"  Mean: {np.mean(cte_values):.3f} m")
    print(f"  Max:  {np.max(cte_values):.3f} m")
    print(f"  Std:  {np.std(cte_values):.3f} m")
    print(f"Heading Error:")
    print(f"  Mean: {np.mean(heading_errors):.3f} rad "
          f"({np.degrees(np.mean(heading_errors)):.1f}°)")
    print(f"  Max:  {np.max(heading_errors):.3f} rad")
    print(f"Path Length Ratio: {actual_dist / planned_dist:.3f}")
    print(f"Average Velocity: {np.mean(velocities):.3f} m/s")


if __name__ == '__main__':
    import sys
    analyze_bag(sys.argv[1] if len(sys.argv) > 1 else 'nav_metrics_bag')
```

## Using the Script

```bash
# Install dependencies
pip install rosbags numpy

# Run analysis
python3 analyze_nav_metrics.py nav_metrics_bag/
```

## Interpreting Results for Tuning

| Symptom | Likely Cause | Tuning Action |
|---------|-------------|---------------|
| High mean CTE | PathAlignCritic weight too low | Increase PathAlignCritic.cost_weight |
| High max CTE | Robot deviates at corners | Increase PathFollowCritic.cost_weight, check wz_max |
| High heading error | PathAngleCritic too weak | Increase PathAngleCritic.cost_weight or lower max_angle_to_furthest |
| Path length ratio > 1.2 | Oscillation or detours | Check Oscillation critic (DWB), or TwirlingCritic (MPPI) |
| Low average velocity | Over-regulation near obstacles | Reduce CostCritic.cost_weight, check costmap inflation |
| Frequent recoveries | Controller fails in tight spaces | Increase batch_size (MPPI), reduce obstacle critic sensitivity |

## Continuous Monitoring

For production patrol robots, log metrics per-goal and aggregate over time:

```python
# In your patrol node, subscribe to /odom and /plan
# Compute per-goal CTE and log to file
# Alert if mean CTE exceeds threshold for N consecutive goals
```

This enables early detection of:
- Costmap drift (SLAM map vs reality).
- Mechanical issues (wheel slip → high CTE).
- Environment changes (new furniture → frequent recoveries).
