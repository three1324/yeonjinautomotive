<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: controller-comparison
description: 'Compare Nav2 controllers (MPPI, DWB, RPP, Graceful). Use when choosing a local controller for your robot type and use case.'
---

# Controller Comparison — MPPI vs DWB vs RPP

## Decision Matrix

| Feature | MPPI | DWB | RPP |
|---------|------|-----|-----|
| **Algorithm** | Sampling-based optimal control | Dynamic window velocity sampling | Pure pursuit with regulation |
| **CPU cost** | High (15–80ms/cycle) | Medium (5–20ms/cycle) | Low (<2ms/cycle) |
| **Path quality** | Best — optimizes over full horizon | Good — best of discrete samples | Adequate — follows carrot |
| **Tuning complexity** | High — many critics/weights | Medium — fewer critics | Low — intuitive params |
| **Narrow spaces** | Excellent — trajectory optimization finds gaps | Good — samples may miss tight gaps | Fair — no obstacle-aware planning |
| **Dynamic obstacles** | Excellent — re-optimizes each cycle | Good — samples around obstacles | Fair — relies on replanning |
| **Predictability** | Medium — weighted average can shift | Medium — best sample may jump | High — always follows carrot |
| **Reverse handling** | Configurable via critics | Configurable via velocity limits | `allow_reversing` flag |
| **In-place rotation** | Via critics or RotationShim | Needs external handling | Built-in `rotate_to_heading` |
| **Obstacle avoidance** | CostCritic with optional footprint | BaseObstacle/ObstacleFootprint | Cost regulation (slows down) |
| **Goal approach** | GoalCritic + GoalAngleCritic | RotateToGoal + GoalDist | Built-in deceleration |

## CPU Budget Analysis

For a controller running at 20Hz (50ms budget per cycle):

| Controller | Typical compute | Margin at 20Hz | Max feasible frequency |
|------------|----------------|-----------------|----------------------|
| MPPI (2000 batch, no footprint) | 20ms | 30ms | ~50Hz |
| MPPI (2000 batch, with footprint) | 50ms | 0ms | ~20Hz |
| MPPI (3000 batch, no footprint) | 30ms | 20ms | ~33Hz |
| DWB (400 trajectories) | 8ms | 42ms | >100Hz |
| DWB (600 trajectories) | 12ms | 38ms | ~83Hz |
| RPP | 1ms | 49ms | >100Hz |

On ARM-based single-board computers (Raspberry Pi 4, Jetson Nano), expect 2–3× longer compute times.

## Scenario Recommendations

### House Patrol Robot (typical indoor robot use case)

**Recommended: MPPI** with RotationShimController.

Justification:
- Indoor environments have doorways (0.8m wide), furniture edges, narrow corridors — MPPI's trajectory optimization excels here.
- Multiple rooms with sharp 90° turns — MPPI handles these smoothly.
- Obstacle avoidance is critical (pets, humans, objects left on floor).
- CPU is available (typical patrol robot uses Intel/ARM64 with headroom).

### Simple Corridor Patrol

**Recommended: RPP** — simple straight corridors with infrequent turns.

Justification:
- Low CPU.
- Path is mostly straight; no need for trajectory optimization.
- Predictable, auditable behavior.

### Warehouse/Large Open Space

**Recommended: DWB** — open areas with scattered obstacles.

Justification:
- Good balance of path quality and CPU cost.
- Obstacle avoidance via BaseObstacle is sufficient for sparse obstacles.
- Easier to tune than MPPI for straightforward environments.

### Competition/Performance-Critical

**Recommended: MPPI** with high `batch_size` and `consider_footprint: true`.

Justification:
- Maximum path quality and obstacle negotiation.
- Worth the CPU cost for competitive or safety-critical applications.

## Hybrid Approaches

### RotationShim + Any Controller

Wraps any controller with clean initial rotation. Best with MPPI or DWB. See `rotation_shim_controller.md`.

### Multiple Controller Plugins

Nav2 supports multiple controller plugins in the `controller_plugins` list. Different behavior tree tasks can request different controllers:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath", "DockApproach"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      # ... MPPI params
    DockApproach:
      plugin: "nav2_graceful_controller::GracefulController"
      # ... Graceful params
```

The behavior tree uses the `controller_id` field in the `FollowPath` action to select which controller handles each path segment.

### Migration Path

1. **Start with RPP** — get basic navigation working, verify costmaps and planning.
2. **Switch to MPPI** — use the baseline MPPI configuration from `mppi_controller.md`.
3. **Tune MPPI critics** — adjust weights iteratively with visualization enabled.
4. **Add RotationShim** — if initial rotation behavior is unsatisfactory.
5. **Add VelocityDeadbandCritic** — if motor deadband causes issues.

## Parameter Cross-Reference

Parameters with similar roles across controllers:

| Concept | MPPI | DWB | RPP |
|---------|------|-----|-----|
| Max forward vel | `vx_max` | `max_vel_x` | `desired_linear_vel` |
| Max angular vel | `wz_max` | `max_vel_theta` | N/A (curvature-based) |
| Forward accel | `ax_max` | `acc_lim_x` | N/A |
| Angular accel | `az_max` | `acc_lim_theta` | `max_angular_accel` |
| Lookahead time | `time_steps × model_dt` | `sim_time` | `lookahead_time` |
| Obstacle weight | `CostCritic.cost_weight` | `BaseObstacle.scale` | `cost_scaling_gain` |
| Path tracking | `PathAlignCritic.cost_weight` | `PathDist.scale` | `lookahead_dist` |
| Goal attraction | `GoalCritic.cost_weight` | `GoalDist.scale` | Built-in deceleration |
| Heading at goal | `GoalAngleCritic.cost_weight` | `RotateToGoal.scale` | `use_rotate_to_heading` |

## Switching Controllers

When switching from one controller to another, the primary changes are in the `FollowPath` namespace. The `controller_server` configuration, costmap setup, and behavior tree remain largely the same. Only the plugin name and controller-specific parameters change.
