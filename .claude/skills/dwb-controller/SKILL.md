<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: dwb-controller
description: 'Configure Nav2 DWB (Dynamic Window Based) controller. Use when setting velocity samples, forward simulation, or trajectory scoring.'
---

# DWB Controller — Dynamic Window-Based Local Planner

## Algorithm Overview

DWB (Dynamic Window approach, B-variant in Nav2) samples discrete velocity commands within the robot's dynamic window — the set of velocities reachable given current velocity and acceleration limits within one control cycle. Each sampled velocity is forward-simulated to produce a trajectory, and all trajectories are scored by critic plugins.

Plugin: `dwb_core::DWBLocalPlanner`

## How It Differs from MPPI

- DWB samples **velocities** (vx, vy, wz) and simulates forward. MPPI samples **trajectory perturbations** over the full horizon.
- DWB evaluates each trajectory independently. MPPI uses Boltzmann-weighted averaging.
- DWB is less computationally demanding but produces less-optimal trajectories in complex environments.
- DWB critic evaluation order can matter when `short_circuit_trajectory_evaluation: true`.

## Key Parameters

### Velocity Limits

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| `min_vel_x` | float | Minimum forward velocity (negative = reverse allowed) | -0.1 |
| `max_vel_x` | float | Maximum forward velocity (m/s) | 0.5 |
| `min_vel_y` | float | Minimum lateral velocity (0 for diff-drive) | 0.0 |
| `max_vel_y` | float | Maximum lateral velocity (0 for diff-drive) | 0.0 |
| `max_vel_theta` | float | Maximum angular velocity (rad/s) | 1.0 |
| `min_speed_xy` | float | Minimum translational speed to be considered moving | 0.0 |
| `max_speed_xy` | float | Maximum translational speed | 0.5 |
| `min_speed_theta` | float | Minimum rotational speed | 0.0 |

### Acceleration Limits

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| `acc_lim_x` | float | Forward acceleration limit (m/s²) | 2.5 |
| `acc_lim_y` | float | Lateral acceleration (0 for diff-drive) | 0.0 |
| `acc_lim_theta` | float | Angular acceleration limit (rad/s²) | 3.2 |
| `decel_lim_x` | float | Deceleration limit (negative, m/s²) | -2.5 |
| `decel_lim_y` | float | Lateral deceleration | 0.0 |
| `decel_lim_theta` | float | Angular deceleration (negative) | -3.2 |

### Trajectory Generation

| Parameter | Type | Description | Typical |
|-----------|------|-------------|---------|
| `sim_time` | float | Forward simulation time (seconds) | 1.7 |
| `vx_samples` | int | Number of linear velocity samples | 20 |
| `vy_samples` | int | Number of lateral velocity samples (1 for diff-drive) | 1 |
| `vtheta_samples` | int | Number of angular velocity samples | 20 |
| `transform_tolerance` | float | TF tolerance (seconds) | 0.1 |
| `short_circuit_trajectory_evaluation` | bool | Stop scoring a trajectory once it's clearly worse than best so far | true |

**Total trajectory count** = `vx_samples × vy_samples × vtheta_samples`. For 20×1×20 = 400 trajectories. More samples = better coverage of velocity space but more CPU. DWB typically evaluates 200–600 trajectories per cycle.

### Trajectory Generators

The default generator is `dwb_plugins::StandardTrajectoryGenerator`. Alternative: `dwb_plugins::LimitedAccelGenerator` which strictly enforces acceleration constraints on samples.

```yaml
trajectory_generator_name: "dwb_plugins::StandardTrajectoryGenerator"
```

## Critics

Critics are specified as a list. Each critic scores all evaluated trajectories. The order matters when `short_circuit_trajectory_evaluation: true` — place collision-checking critics first to reject infeasible trajectories early.

```yaml
critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

See `dwb_critics.md` for details on each critic plugin.

## Complete YAML for Differential Drive

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.0
    min_theta_velocity_threshold: 0.001
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: true
      # Velocity limits
      min_vel_x: 0.0
      max_vel_x: 0.5
      min_vel_y: 0.0
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      # Acceleration limits
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      # Trajectory generation
      sim_time: 1.7
      vx_samples: 20
      vy_samples: 1
      vtheta_samples: 20
      transform_tolerance: 0.1
      short_circuit_trajectory_evaluation: true
      # Trajectory generator
      trajectory_generator_name: "dwb_plugins::StandardTrajectoryGenerator"
      # Critics
      critics:
        - "RotateToGoal"
        - "Oscillation"
        - "BaseObstacle"
        - "GoalAlign"
        - "PathAlign"
        - "PathDist"
        - "GoalDist"
      BaseObstacle:
        scale: 0.02
        sum_scores: false
      PathDist:
        scale: 32.0
      GoalDist:
        scale: 24.0
      PathAlign:
        scale: 32.0
        forward_point_distance: 0.325
      GoalAlign:
        scale: 24.0
        forward_point_distance: 0.325
      RotateToGoal:
        scale: 32.0
        slowing_factor: 5.0
        lookahead_time: -1.0
      Oscillation:
        scale: 1.0
```

## Tuning Tips

- **Robot wobbles/oscillates**: Increase `Oscillation` scale, reduce `vtheta_samples`, or lower `acc_lim_theta`.
- **Robot clips corners**: Increase `PathDist` scale or `PathAlign` scale.
- **Robot won't enter narrow spaces**: Reduce `BaseObstacle` scale; check inflation radius.
- **Robot is sluggish**: Increase `vx_samples`, increase `max_vel_x`, or reduce `sim_time` (shorter lookahead can be more agile).
- **CPU too high**: Reduce `vx_samples × vtheta_samples`, enable `short_circuit_trajectory_evaluation`.

## When to Use DWB

- Middle ground between MPPI (heavy, optimal) and RPP (light, simple).
- Good for environments that are moderately complex.
- Familiar to users of the ROS1 `dwa_local_planner`.
- Easier to reason about than MPPI because trajectories correspond to single velocity commands.
