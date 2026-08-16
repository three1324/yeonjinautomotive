<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: mppi-controller
description: 'Tune MPPI controller for Nav2. Use when configuring model_predictive_path_integral, trajectory rollouts, batch_size, time_steps, or velocity limits.'
---

# MPPI Controller — Model Predictive Path Integral Control

## Algorithm Overview

MPPI (Model Predictive Path Integral) is a sampling-based optimal controller. Each control cycle:

1. **Sample** `batch_size` random trajectory rollouts by injecting Gaussian noise into the control space.
2. **Forward-simulate** each trajectory for `time_steps` steps, each of duration `model_dt`.
3. **Score** every trajectory by summing costs from all active critic plugins.
4. **Weight** trajectories using a softmin (Boltzmann) distribution controlled by `temperature`.
5. **Average** the weighted control sequences to produce the optimal velocity command.

The prediction horizon in seconds = `time_steps × model_dt`. For example, 56 steps × 0.05s = 2.8s lookahead.

## Key Parameter Reference

### Trajectory Sampling

| Parameter | Type | Description | Typical |
|-----------|------|-------------|---------|
| `time_steps` | int | Lookahead steps. Higher = longer horizon, smoother plans, more CPU | 30–80 |
| `model_dt` | float | Time between steps (seconds). Lower = finer resolution | 0.05–0.1 |
| `batch_size` | int | Number of sampled trajectories per iteration | 1000–3000 |
| `iteration_count` | int | Optimization iterations per control cycle. Usually 1 suffices | 1–2 |
| `temperature` | float | Boltzmann temperature. Lower = greedier (exploits best), higher = explores more | 0.1–0.5 |
| `gamma` | float | Discount factor. Higher values weight near-term costs more heavily | 0.015 |

### Motion Model

| Parameter | Description |
|-----------|-------------|
| `motion_model` | `"DiffDrive"` (constrains vy=0), `"Omni"`, or `"Ackermann"` |

### Velocity Limits

| Parameter | Description | Example |
|-----------|-------------|---------|
| `vx_max` | Maximum forward velocity (m/s) | 0.5 |
| `vx_min` | Minimum linear velocity; negative allows reverse | -0.15 |
| `vy_max` | Maximum lateral velocity (0.0 for DiffDrive) | 0.0 |
| `wz_max` | Maximum angular velocity (rad/s) | 1.9 |

### Acceleration Limits

| Parameter | Description | Example |
|-----------|-------------|---------|
| `ax_max` | Max forward acceleration (m/s²) | 3.0 |
| `ax_min` | Max deceleration (negative, m/s²) | -3.0 |
| `ay_max` | Max lateral acceleration | 0.0 |
| `az_max` | Max angular acceleration (rad/s²) | 3.5 |

### Noise (Sampling Width)

| Parameter | Description | Typical |
|-----------|-------------|---------|
| `vx_std` | Std dev of Gaussian noise on linear velocity | 0.2 |
| `vy_std` | Std dev of lateral velocity noise (0 for DiffDrive) | 0.0 |
| `wz_std` | Std dev of angular velocity noise | 0.4 |

If `vx_std` / `wz_std` are **too small**, sampled trajectories cluster tightly and the controller gets trapped in local minima. If **too large**, trajectories scatter wildly and cost averages become noisy.

### Other Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `prune_distance` | 1.5 | Distance (m) ahead of robot at which path points start being considered |
| `transform_tolerance` | 0.1 | TF lookup tolerance (seconds) |
| `regenerate_noises` | true | Re-randomize Gaussian noise each iteration |
| `retry_attempt_limit` | 1 | Retries before declaring failure |

## Parameter Interactions

- **Horizon vs CPU**: Increasing `time_steps` or `batch_size` linearly increases computation. Doubling `batch_size` from 1000→2000 roughly doubles CPU time.
- **Horizon vs smoothness**: A longer horizon (`time_steps × model_dt > 2s`) produces smoother trajectories but reacts more slowly to sudden obstacles.
- **Noise vs convergence**: Wider noise (`vx_std`, `wz_std`) explores more but needs higher `batch_size` to ensure good trajectories are found. Narrow noise converges faster but misses alternatives.
- **Temperature**: At `temperature=0.01`, only the single best trajectory matters (greedy). At `temperature=1.0`, all trajectories contribute nearly equally (pure averaging). For most indoor use, 0.15–0.3 balances well.
- **Iteration count**: With sufficient `batch_size` (≥1500), a single iteration usually finds a good solution. A second iteration refines around the first solution and costs ~2× the CPU.

## CPU Cost Model

Per control cycle cost ≈ `iteration_count × batch_size × time_steps × (critic_eval_cost)`.

For 1 iteration, 2000 trajectories, 56 time steps, approximately:
- Without footprint checking: ~15–25ms on a modern x86 CPU.
- With `consider_footprint: true` on CostCritic: ~40–80ms (scales with footprint polygon vertices).
- Visualization enabled: +5–10ms overhead.

Target: keep total compute under `1 / controller_frequency` (e.g., under 50ms for 20Hz).

## Complete YAML Configuration

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.0
    min_theta_velocity_threshold: 0.001
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      iteration_count: 1
      temperature: 0.25
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: false
      regenerate_noises: true
      retry_attempt_limit: 1
      prune_distance: 1.5
      transform_tolerance: 0.1
      # Velocity limits
      vx_max: 0.5
      vx_min: -0.15
      vy_max: 0.0
      wz_max: 1.9
      # Acceleration limits
      ax_max: 3.0
      ax_min: -3.0
      ay_max: 0.0
      az_max: 3.5
      # Sampling noise
      vx_std: 0.2
      vy_std: 0.0
      wz_std: 0.4
      # Critics (see mppi_critics.md for details)
      critics:
        - "ConstraintCritic"
        - "CostCritic"
        - "GoalCritic"
        - "GoalAngleCritic"
        - "PathAlignCritic"
        - "PathFollowCritic"
        - "PathAngleCritic"
        - "PreferForwardCritic"
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        critical_cost: 300.0
        consider_footprint: false
        collision_cost: 1000000.0
        near_goal_distance: 1.0
        trajectory_point_step: 2
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.07
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
```

## Tuning Workflow

1. Start with the YAML above as a baseline.
2. Set `visualize: true` and observe trajectories in RViz2.
3. If robot hugs walls or oscillates near obstacles: increase `CostCritic.cost_weight` or reduce `PathAlignCritic.cost_weight`.
4. If robot overshoots goals: increase `GoalCritic.cost_weight` and reduce `threshold_to_consider`.
5. If robot is sluggish or takes wide turns: increase `wz_std` and `batch_size`.
6. If CPU is too high: reduce `batch_size`, increase `trajectory_point_step` on critics, set `consider_footprint: false`.
7. Once stable, set `visualize: false` for production.
