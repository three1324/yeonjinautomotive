<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: regulated-pure-pursuit
description: 'Configure Nav2 Regulated Pure Pursuit controller. Use when setting lookahead distance, curvature regulation, or approach velocity scaling.'
---

# Regulated Pure Pursuit Controller (RPP)

## Algorithm Overview

Regulated Pure Pursuit is a path-tracking controller that follows a "carrot" (lookahead) point on the global path. The robot drives toward the carrot point using curvature-based steering. It adds regulation behaviors to slow the robot for tight turns, high-cost areas, and approach to the goal.

Plugin: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`

RPP is the simplest and most predictable controller in Nav2. It has low CPU cost and is easy to tune.

## Core Algorithm

1. Find the lookahead point on the global path at distance `lookahead_dist` ahead of the robot.
2. Compute the curvature to reach that point (pure pursuit geometry).
3. Command linear vel = `desired_linear_vel`, angular vel = `curvature × linear_vel`.
4. Apply regulation: slow for tight curvature, high costmap costs, proximity to goal.

## Lookahead Configuration

### Fixed Lookahead

```yaml
lookahead_dist: 0.6  # meters ahead on path
```

Robot always looks 0.6m ahead. Simple but limited — at high speed you want to look further ahead.

### Velocity-Scaled Lookahead (Recommended)

```yaml
use_velocity_scaled_lookahead_dist: true
min_lookahead_dist: 0.3
max_lookahead_dist: 0.9
lookahead_time: 1.5  # seconds; lookahead = current_vel × lookahead_time
```

The lookahead distance scales with the robot's current speed: at low speed, look nearby (tight control); at high speed, look further (smooth anticipation). Clamped to `[min_lookahead_dist, max_lookahead_dist]`.

## Velocity and Acceleration

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| `desired_linear_vel` | float | Target forward speed (m/s) | 0.5 |
| `max_angular_accel` | float | Angular acceleration limit (rad/s²) | 3.2 |

## Regulation Behaviors

### Curvature Regulation

```yaml
use_regulated_linear_velocity_scaling: true
regulated_linear_scaling_min_radius: 0.9  # meters
regulated_linear_scaling_min_speed: 0.25  # m/s
```

When the path curvature is tight (radius < `regulated_linear_scaling_min_radius`), the linear velocity is scaled down toward `regulated_linear_scaling_min_speed`. Prevents the robot from taking tight turns at full speed.

### Cost Regulation

```yaml
use_cost_regulated_linear_velocity_scaling: true
cost_scaling_dist: 0.6          # meters from obstacle at which to start slowing
cost_scaling_gain: 1.0          # proportional gain for slowdown
inflation_cost_scaling_factor: 3.0  # must match costmap inflation_layer factor
```

When the robot is near high-cost cells (obstacles/inflation), it slows down. The `inflation_cost_scaling_factor` must match the `cost_scaling_factor` in your costmap inflation layer, or the cost-to-distance mapping is wrong.

### Proximity to Goal

As the robot approaches the goal, it automatically decelerates. The deceleration profile is built in.

## Rotate to Heading

**Critical for differential-drive robots.** When the path direction changes significantly, RPP can rotate the robot in place before driving.

```yaml
use_rotate_to_heading: true
rotate_to_heading_min_angle: 0.785  # radians (~45°); angular error to trigger rotation
rotate_to_heading_angular_vel: 1.8  # rad/s during in-place rotation
max_angular_accel: 3.2              # acceleration limit during rotation
```

**Behavior**: If the heading error to the lookahead point exceeds `rotate_to_heading_min_angle`, the robot stops and rotates in place until aligned, then resumes driving.

**Important**: `use_rotate_to_heading` and `allow_reversing` are mutually exclusive. You cannot enable both.

```yaml
allow_reversing: false  # must be false when use_rotate_to_heading is true
```

## Complete YAML for Differential Drive

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      max_angular_accel: 3.2
      # Lookahead
      use_velocity_scaled_lookahead_dist: true
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      # Curvature regulation
      use_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      # Cost regulation
      use_cost_regulated_linear_velocity_scaling: true
      cost_scaling_dist: 0.6
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 3.0
      # Rotate to heading (diff-drive)
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      rotate_to_heading_angular_vel: 1.8
      max_angular_accel: 3.2
      allow_reversing: false
      # Tolerances
      transform_tolerance: 0.1
```

## When to Use RPP

**Advantages**:
- Very low CPU cost. Negligible compared to MPPI.
- Predictable behavior — always follows the path carrot.
- Easy to tune — fewer parameters, intuitive behavior.
- Reliable in simple environments.

**Disadvantages**:
- Does not optimize trajectories — follows the carrot naively.
- Less capable in narrow passages or around complex obstacles (no trajectory scoring).
- No dynamic obstacle reaction beyond what the planner provides.
- Cannot exploit gaps or find shortcuts.

**Best for**: Simple patrol tasks in open areas, robots with low computational resources, or as a baseline controller before upgrading to MPPI.

## Common Tuning Issues

- **Robot oscillates on straight paths**: `lookahead_dist` too small or `use_velocity_scaled_lookahead_dist` not enabled.
- **Robot cuts corners**: Increase `min_lookahead_dist` or `regulated_linear_scaling_min_radius`.
- **Robot stops at sharp turns**: `rotate_to_heading_min_angle` too low (triggers on small corrections). Increase to 0.785–1.0.
- **Robot too slow near walls**: Check `inflation_cost_scaling_factor` matches costmap; reduce `cost_scaling_gain`.
- **Robot jerks at start of path**: Needs RotationShimController to orient before RPP starts.
