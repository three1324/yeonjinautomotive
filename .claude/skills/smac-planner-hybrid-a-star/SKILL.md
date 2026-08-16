<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: smac-planner-hybrid-a-star
description: 'Configure Nav2 SMAC Hybrid-A* planner. Use when planning kinematically feasible paths with minimum turning radius constraints.'
---

# SMAC Hybrid-A* Planner

## Plugin
```
nav2_smac_planner::SmacPlannerHybridAstar
```

## Algorithm
Hybrid-A* extends A* into the SE2 state space (x, y, theta), producing paths that respect non-holonomic motion constraints. The search graph discretizes heading into angular bins and expands nodes using Dubins or Reeds-Shepp motion primitives. An analytic expansion periodically attempts to connect the current node directly to the goal using a closed-form curve, dramatically reducing search time when a clear path exists.

The planner searches in a 3D space: (x_cell, y_cell, heading_bin). Each expansion applies a kinematically feasible motion primitive, ensuring the output path can be physically executed by the robot.

## Critical: Minimum Turning Radius

```yaml
minimum_turning_radius: 0.20  # meters - for differential drive robots
```

**For differential drive robots**, the true minimum turning radius is effectively zero (the robot can spin in place). However, setting it to exactly 0 produces degenerate motion primitives. Use a small value like **0.1–0.4 meters** to produce smooth, natural-looking arcs while still allowing tight turns. Larger values (0.5+) force wider arcs and may cause failures in tight spaces.

For Ackermann (car-like) robots, set this to the actual mechanical minimum turning radius.

## Motion Models

| Model | Description | Use Case |
|-------|-------------|----------|
| `DUBIN` | Forward-only arcs (left, straight, right) | Robots that cannot reverse (e.g., car-like robots without reverse gear) |
| `REEDS_SHEPP` | Forward and reverse arcs | Robots that can reverse. Produces more flexible paths, especially in constrained spaces like parking/docking. |

For differential drive robots, **Reeds-Shepp** is preferred since the robot can move backward. For Ackermann robots, choose based on whether reversing is allowed in your application.

## Key Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `minimum_turning_radius` | double | 0.40 | Minimum turning radius in meters. |
| `max_iterations` | int | 1000000 | Maximum SE2 node expansions. |
| `max_planning_time` | double | 5.0 | Hard time limit in seconds. |
| `motion_model_for_search` | string | "DUBIN" | "DUBIN" or "REEDS_SHEPP". |
| `angle_quantization_bins` | int | 72 | Number of heading bins. 72 = 5° resolution, 36 = 10°. More bins = finer heading control but larger search space. |
| `lookup_table_size` | double | 20.0 | Size of the precomputed heuristic lookup table in meters. Should be ≥ the expected map size. |
| `allow_unknown` | bool | true | Plan through unknown space. |
| `allow_reverse_expansion` | bool | false | Only used with REEDS_SHEPP. Allow the planner to expand reverse motions. |

## Analytic Expansion Parameters

The analytic expansion attempts to shortcut the search by computing a closed-form Dubins/Reeds-Shepp curve from the current node to the goal.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `analytic_expansion_ratio` | double | 3.5 | How often (relative to search progress) to attempt analytic expansion. Lower = more frequent attempts. |
| `analytic_expansion_max_length` | double | 3.0 | Maximum length of the analytic expansion in meters. |

## Penalty Parameters

These penalties shape path characteristics by adding cost to certain types of motion.

| Parameter | Type | Default | Effect |
|-----------|------|---------|--------|
| `change_penalty` | double | 0.0 | Cost for switching between forward and reverse motion. Higher values reduce oscillation. |
| `non_straight_penalty` | double | 1.20 | Cost multiplier for turning vs going straight. Higher = straighter paths. |
| `cost_penalty` | double | 2.0 | Multiplier for costmap cell costs. **Higher values make the planner strongly avoid high-cost areas** (near obstacles). Critical for safety margin tuning. |
| `reverse_penalty` | double | 2.0 | Cost multiplier for reverse motion. Higher = prefers forward motion. |

**Penalty tuning strategy:**
- Robot hugs walls → increase `cost_penalty` (try 3.0–5.0).
- Path has too many unnecessary turns → increase `non_straight_penalty`.
- Robot reverses when it shouldn't → increase `reverse_penalty` or disable `allow_reverse_expansion`.
- Robot oscillates forward/reverse → increase `change_penalty`.

## Complete YAML Configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybridAstar"
      tolerance: 0.25
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      minimum_turning_radius: 0.20
      motion_model_for_search: "REEDS_SHEPP"
      angle_quantization_bins: 72
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      lookup_table_size: 20.0
      allow_unknown: true
      allow_reverse_expansion: false
      cost_travel_multiplier: 2.0
      change_penalty: 0.0
      non_straight_penalty: 1.20
      cost_penalty: 2.0
      reverse_penalty: 2.0
      downsample_costmap: false
      downsampling_factor: 1
      cost_penalty: 2.0
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
```

## When To Use

**Good fit:**
- You want kinematically feasible paths directly from the planner, reducing reliance on the controller to fix infeasible segments.
- Ackermann steering robots with a real minimum turning radius.
- Differential drive robots in constrained environments where smooth, arc-based paths reduce controller effort.
- Docking or parking maneuvers where approach angle matters.

**Poor fit:**
- Simple open environments where NavFn + controller smoothing suffices (Hybrid-A* is slower).
- Real-time replanning at high frequency (the 3D search is inherently more expensive).
- Holonomic robots that don't benefit from non-holonomic motion planning.

## Performance Considerations

Hybrid-A* is significantly slower than 2D planners because the search space is O(X × Y × θ) instead of O(X × Y). Mitigation strategies:
- **Downsample the costmap** (`downsample_costmap: true`, `downsampling_factor: 2`).
- **Reduce `angle_quantization_bins`** to 36 (10° resolution) if fine heading control isn't needed.
- **Lower `max_planning_time`** and accept occasional planning failures that trigger replanning.
- **Reduce `lookup_table_size`** if your map is smaller.

## Troubleshooting

- **Planner times out in open space:** The lookup table or analytic expansion may be misconfigured. Increase `analytic_expansion_ratio` to attempt shortcuts more often.
- **Paths have unnecessary reversal segments:** Set `allow_reverse_expansion: false` or increase `reverse_penalty`.
- **Planner fails in narrow corridors:** Decrease `minimum_turning_radius`. Check that costmap inflation isn't closing off the corridor.
- **Path doesn't match robot's actual capability:** Ensure `minimum_turning_radius` matches your robot. For diff-drive, 0.1–0.4m is typical.
