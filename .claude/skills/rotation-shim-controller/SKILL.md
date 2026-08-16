<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: rotation-shim-controller
description: 'Configure Nav2 rotation shim controller. Use when the robot needs to rotate in place before following a path, setting angular_dist_threshold.'
---

# RotationShimController

## Purpose

The RotationShimController wraps another controller (MPPI, DWB, or RPP) and handles the **initial rotation** at the start of a new path. When the robot receives a new navigation goal, it often faces a different direction than the path requires. Without RotationShim, the primary controller must simultaneously rotate and translate, which can produce awkward or inefficient motion on differential-drive robots.

RotationShim solves this by:
1. Detecting that the robot's heading is misaligned with the path direction.
2. Commanding an in-place rotation to face the path.
3. Handing off to the primary controller once aligned.

Plugin: `nav2_rotation_shim_controller::RotationShimController`

## When to Use

- **Differential-drive robots**: Cannot translate laterally. Without RotationShim, the robot may drive sideways or backwards initially.
- **Any controller that doesn't handle initial orientation well**: RPP has `rotate_to_heading`, but MPPI and DWB may produce suboptimal initial motion.
- **Smooth start behavior**: The robot cleanly rotates, then drives forward.

## When NOT to Use

- **Omni-directional robots**: Can translate in any direction; initial rotation is unnecessary.
- **Very tight spaces**: In-place rotation may be blocked by nearby obstacles. The collision check helps but is not perfect.
- **RPP with `use_rotate_to_heading: true`**: RPP already handles this internally; RotationShim would be redundant.

## Key Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `angular_dist_threshold` | float | 0.785 | Minimum heading error (radians) to trigger in-place rotation |
| `forward_sampling_distance` | float | 0.5 | How far along the path to sample the target heading (meters) |
| `rotate_to_heading_angular_vel` | float | 1.8 | Angular velocity during in-place rotation (rad/s) |
| `max_angular_accel` | float | 3.2 | Angular acceleration limit during rotation (rad/s²) |
| `simulate_ahead_time` | float | 1.0 | Time to simulate ahead for collision checking during rotation |
| `primary_controller` | string | — | Plugin name of the wrapped controller |

### Parameter Details

**`angular_dist_threshold`**: If the angle between the robot's current heading and the direction toward the path point at `forward_sampling_distance` exceeds this threshold, RotationShim takes over and rotates. If below threshold, the primary controller handles immediately.

Typical values:
- 0.785 rad (~45°): Only rotate for large misalignments. Quick handoff to primary.
- 0.4 rad (~23°): Rotate for moderate misalignments. Smoother starts.
- 1.57 rad (~90°): Only rotate when nearly perpendicular to path.

**`forward_sampling_distance`**: Determines which path point to "aim" for during initial rotation. If too small, the target might be at the robot's current position (useless). If too large, the robot rotates toward a distant point that doesn't represent the immediate path direction (e.g., around a corner). Typically 0.5–1.0m.

**`simulate_ahead_time`**: During the rotation, RotationShim simulates the robot's footprint at `simulate_ahead_time` seconds into the future to check for collisions. If the rotation would sweep the robot into an obstacle, it aborts and passes control to the primary controller (which can handle the situation with trajectory optimization).

## Configuration Structure

RotationShimController is configured as the `FollowPath` plugin, and the primary controller is a parameter within it:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_mppi_controller::MPPIController"
      angular_dist_threshold: 0.785
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.8
      max_angular_accel: 3.2
      simulate_ahead_time: 1.0
      # All primary controller params go here as if MPPI were the top-level plugin:
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      # ... etc (all MPPI params)
```

## Complete YAML: RotationShim Wrapping MPPI

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_mppi_controller::MPPIController"
      # RotationShim params
      angular_dist_threshold: 0.785
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.8
      max_angular_accel: 3.2
      simulate_ahead_time: 1.0
      # MPPI params (passed through to primary controller)
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      iteration_count: 1
      temperature: 0.25
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: false
      vx_max: 0.5
      vx_min: -0.15
      vy_max: 0.0
      wz_max: 1.9
      ax_max: 3.0
      ax_min: -3.0
      az_max: 3.5
      vx_std: 0.2
      wz_std: 0.4
      prune_distance: 1.5
      transform_tolerance: 0.1
      critics:
        - "ConstraintCritic"
        - "CostCritic"
        - "GoalCritic"
        - "GoalAngleCritic"
        - "PathAlignCritic"
        - "PathFollowCritic"
        - "PathAngleCritic"
        - "PreferForwardCritic"
      # ... critic params as in mppi_controller.md
```

## Behavior During Operation

1. **New path received**: RotationShim checks heading error against `angular_dist_threshold`.
2. **Error exceeds threshold**: RotationShim takes control. Publishes `geometry_msgs/msg/TwistStamped` with `linear.x=0, angular.z=rotate_to_heading_angular_vel` (or `-rotate_to_heading_angular_vel` for the shorter rotation direction).
3. **Collision check**: Each cycle, simulates the rotation `simulate_ahead_time` into the future. If collision detected, aborts rotation and passes to primary controller immediately.
4. **Heading aligned**: Once the heading error drops below `angular_dist_threshold`, hands off to the primary controller.
5. **Primary controller runs**: From this point, RotationShim is transparent — the primary controller handles all velocity commands until a new path is received.

## Troubleshooting

- **Robot rotates but then the primary controller rotates again**: The `angular_dist_threshold` on RotationShim may be too loose (rotates to ~45° but MPPI wants exact alignment). Decrease `angular_dist_threshold` to match expected primary controller behavior.
- **Robot won't rotate in tight spaces**: The collision simulation prevents rotation. Decrease `simulate_ahead_time` or increase the obstacle clearance in the costmap.
- **Rotation is too slow**: Increase `rotate_to_heading_angular_vel` (respect motor limits).
- **Double rotation with RPP**: If using RPP as primary and RPP's `use_rotate_to_heading: true`, disable one of them. Either disable RPP's rotate-to-heading OR don't use RotationShim.

## RotationShim with Different Controllers

| Primary Controller | Recommended | Notes |
|-------------------|-------------|-------|
| MPPI | Yes | MPPI can handle initial rotation itself but RotationShim is cleaner |
| DWB | Yes | DWB's initial rotation is often suboptimal |
| RPP | Conditional | Only if RPP's `use_rotate_to_heading` is disabled |
| GracefulController | Optional | GracefulController handles approach angles internally |
