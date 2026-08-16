<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: graceful-controller
description: 'Configure Nav2 graceful motion controller. Use when needing smooth approach-to-pose behavior with curvature constraints.'
---

# GracefulController and Multiple Controller Plugins

## GracefulController Overview

The GracefulController implements a smooth control law for graceful approach to a goal. Instead of tracking a path point-by-point, it computes a smooth curve from the robot's current pose to the goal pose, producing natural, arc-like motion.

Plugin: `nav2_graceful_controller::GracefulController`

## Algorithm

The controller uses a Lyapunov-based control law that generates velocity commands to smoothly converge on the goal pose (position + orientation). The approach curve is determined by the `min_turning_radius` and the robot's geometric relationship to the goal.

## Key Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `transform_tolerance` | float | 0.1 | TF lookup tolerance (seconds) |
| `min_turning_radius` | float | 0.5 | Minimum curve radius (meters). Smaller = tighter curves |
| `max_robot_pose_search_dist` | float | 10.0 | Max distance to search for robot pose on path |
| `k_phi` | float | 3.0 | Proportional gain for heading error |
| `k_delta` | float | 2.0 | Proportional gain for steering angle |
| `beta` | float | 0.4 | Curvature gain weighting |
| `lambda` | float | 2.0 | Velocity decay gain as goal approaches |
| `v_linear_min` | float | 0.1 | Minimum forward velocity (m/s) |
| `v_linear_max` | float | 0.5 | Maximum forward velocity (m/s) |
| `v_angular_max` | float | 1.0 | Maximum angular velocity (rad/s) |
| `slowdown_radius` | float | 1.5 | Distance from goal to begin decelerating |
| `initial_rotation` | bool | true | Rotate in place to face goal direction before driving |
| `initial_rotation_min_angle` | float | 0.75 | Min heading error (rad) to trigger initial rotation |
| `final_rotation` | bool | true | Rotate in place at goal to match goal heading |
| `allow_backward` | bool | false | Allow reverse driving to reach the goal |

## When to Use GracefulController

- **Docking**: Precise approach to a charging station at a specific angle.
- **Final approach**: Smooth arrival at a goal pose where jerky motion is unacceptable.
- **Simple navigation**: Environments where a smooth curve to the goal is sufficient.

**Not suitable for**: Complex obstacle-dense environments (no costmap-aware trajectory scoring), long-distance navigation (only approaches a single goal).

## YAML Configuration

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_graceful_controller::GracefulController"
      transform_tolerance: 0.1
      min_turning_radius: 0.5
      max_robot_pose_search_dist: 10.0
      k_phi: 3.0
      k_delta: 2.0
      beta: 0.4
      lambda: 2.0
      v_linear_min: 0.1
      v_linear_max: 0.5
      v_angular_max: 1.0
      slowdown_radius: 1.5
      initial_rotation: true
      initial_rotation_min_angle: 0.75
      final_rotation: true
      allow_backward: false
```

---

## Multiple Controller Plugins

Nav2 supports registering multiple controller plugins. The behavior tree selects which controller to use for each path segment via the `controller_id` parameter in the `FollowPath` action.

### Configuration

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath", "DockApproach"]
    
    # Primary navigation controller
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_mppi_controller::MPPIController"
      angular_dist_threshold: 0.785
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.8
      max_angular_accel: 3.2
      # MPPI params...
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      motion_model: "DiffDrive"
      vx_max: 0.5
      vx_min: -0.15
      wz_max: 1.9
      # ... critics, etc.

    # Docking/precision approach controller
    DockApproach:
      plugin: "nav2_graceful_controller::GracefulController"
      min_turning_radius: 0.3
      v_linear_max: 0.2
      v_angular_max: 0.5
      slowdown_radius: 0.8
      initial_rotation: true
      final_rotation: true
```

### Behavior Tree Integration

In the behavior tree, use the `controller_id` field to select the controller:

```xml
<!-- Normal navigation: uses FollowPath (MPPI) -->
<FollowPath path="{path}" controller_id="FollowPath" 
            server_name="controller_server"/>

<!-- Precision docking: uses DockApproach (Graceful) -->
<FollowPath path="{dock_path}" controller_id="DockApproach" 
            server_name="controller_server"/>
```

### Use Case: Patrol with Docking

1. **Patrol phase**: BT sends navigation goals with `controller_id="FollowPath"`. MPPI handles complex indoor navigation.
2. **Return to charger**: BT computes path to charging station.
3. **Final approach**: Within 2m of charger, BT switches path segment to `controller_id="DockApproach"`. GracefulController provides smooth, precise approach.

### Practical Considerations

- Each controller plugin initializes its own resources (costmap subscription, etc.) but shares the same local costmap.
- Switching controllers mid-path is seamless — the new controller receives the remaining path via `setPlan()`.
- CPU cost: only the active controller runs `computeVelocityCommands()`. Idle controllers consume no CPU.
- All controllers share `controller_frequency`, `progress_checker`, and `goal_checker` settings.

## Controller Plugin List Reference

Available Nav2 controller plugins:

| Plugin | Class | Use Case |
|--------|-------|----------|
| MPPI | `nav2_mppi_controller::MPPIController` | Complex environments, optimal control |
| DWB | `dwb_core::DWBLocalPlanner` | Moderate environments, familiar from ROS1 |
| RPP | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | Simple paths, low CPU |
| Graceful | `nav2_graceful_controller::GracefulController` | Smooth approach, docking |
| RotationShim | `nav2_rotation_shim_controller::RotationShimController` | Wraps any controller for initial rotation |

## Combining Controllers: Recommendations

| Scenario | Primary | Secondary | Notes |
|----------|---------|-----------|-------|
| House patrol + docking | MPPI (via RotationShim) | GracefulController | Switch near charger |
| Warehouse navigation | MPPI | RPP | RPP for long straight aisles |
| Simple patrol only | RPP | None | Single controller suffices |
| Complex + tight goals | MPPI | GracefulController | Graceful for precise goals |
