<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: smac-planner-2d
description: 'Configure Nav2 SMAC 2D planner. Use when using 8-connected grid search, setting maximum_iterations, or cost penalties.'
---

# SMAC Planner 2D

## Plugin
```
nav2_smac_planner::SmacPlanner2D
```

## Algorithm
A* search on an 8-connected 2D costmap grid with a Wavefront (breadth-first) heuristic. Operates directly on the occupancy grid without considering robot kinematics. Produces grid-aligned paths that follow cell centers, then applies an optional internal smoother.

The Wavefront heuristic precomputes exact grid distances from the goal, accounting for obstacles. This makes the heuristic admissible and consistent, guaranteeing optimal paths on the grid.

## Key Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tolerance` | double | 0.25 | Goal tolerance in **costmap cells** (not meters). If the exact goal cell is occupied, the planner accepts a goal within this many cells. |
| `max_iterations` | int | 1000000 | Maximum A* expansion iterations before failure. Increase for very large maps. |
| `max_on_approach_iterations` | int | 1000 | After first finding a path within tolerance, how many more iterations to try improving it. |
| `max_planning_time` | double | 2.0 | Hard time limit in seconds. Planning fails if exceeded. |
| `allow_unknown` | bool | true | If true, treats unknown cells (NO_INFORMATION=255) as free space for planning. Set false if unknown space should block the robot. |
| `use_final_approach_orientation` | bool | false | If true, the final path segment orients the robot toward the goal position rather than using the goal's orientation. Useful for tasks where arrival direction doesn't matter (e.g., "go to room center"). |
| `cost_travel_multiplier` | double | 2.0 | Multiplier applied to costmap cell costs during A* expansion. Higher values make the planner more averse to high-cost areas. |

## Downsampling

Reduces the costmap resolution before planning, trading precision for speed.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `downsample_costmap` | bool | false | Enable costmap downsampling. |
| `downsampling_factor` | int | 1 | Factor by which to reduce resolution. A factor of 2 on a 0.05m costmap plans at 0.10m resolution. |

Downsampling is effective on large outdoor maps where cell-level precision is unnecessary. For indoor robots with tight corridors, keep it disabled or use factor 2 at most.

## Smoother Parameters

The built-in smoother runs after path planning to reduce jaggedness.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `smoother.max_iterations` | int | 1000 | Maximum smoothing iterations. |
| `smoother.w_smooth` | double | 0.3 | Smoothing weight. Higher = smoother path but may deviate from optimal. |
| `smoother.w_data` | double | 0.2 | Data fidelity weight. Higher = stay closer to the original A* path. |
| `smoother.tolerance` | double | 1e-10 | Convergence tolerance for the smoother. |

The smoother minimizes a cost function balancing path smoothness against fidelity to the original planned path. If `w_smooth` >> `w_data`, paths become very smooth but may cut corners near obstacles.

## Complete YAML Configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      tolerance: 0.25
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 2.0
      cost_travel_multiplier: 2.0
      allow_unknown: true
      use_final_approach_orientation: false
      downsample_costmap: false
      downsampling_factor: 1
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
```

## When To Use

**Good fit:**
- Simple indoor environments with wide corridors.
- Fast replanning needed (operates on 2D grid, no angle dimension).
- Robot kinematic constraints are handled by the controller (DWB/MPPI), not the planner.
- Holonomic or omni-directional robots where any-direction movement is valid.

**Poor fit:**
- Robots with non-holonomic constraints where kinematically feasible paths from the planner matter (use Hybrid-A* instead).
- Environments where path smoothness is critical and controller-level smoothing is insufficient.

## Comparison with NavFn

Both operate on the 2D costmap grid. Key differences:
- SMAC 2D uses A* with a Wavefront heuristic; NavFn uses Dijkstra or A* with a potential field approach.
- SMAC 2D includes a built-in smoother; NavFn requires a separate path smoother plugin.
- SMAC 2D supports costmap downsampling for large maps.
- SMAC 2D is generally faster for point-to-point planning due to the focused A* search with a strong heuristic.

## Troubleshooting

- **Planner fails in cluttered space:** Increase `max_iterations` and `max_planning_time`. Check that the costmap inflation radius isn't making passages impassable.
- **Paths hug walls:** Increase `cost_travel_multiplier` to penalize high-cost cells more heavily.
- **Goal unreachable but looks reachable in RViz:** The goal may fall inside an inflated obstacle cell. Increase `tolerance` or check costmap inflation parameters.
- **Jerky paths from the controller:** Enable the smoother or increase `w_smooth`. Consider adding the `nav2_constrained_smoother` or `nav2_smoother_server` downstream.
