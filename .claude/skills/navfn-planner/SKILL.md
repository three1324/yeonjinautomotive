<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: navfn-planner
description: 'Configure Nav2 NavFn planner. Use when using Dijkstra or A* global planning, setting tolerance, or allowing_unknown.'
---

# NavFn Planner

## Plugin
```
nav2_navfn_planner::NavfnPlanner
```

## Algorithm
NavFn computes a **navigation potential field** across the entire costmap using either Dijkstra's algorithm or A*. The potential represents the cost-to-go from every cell to the goal. Once the potential field is computed, the planner traces a **gradient descent** path from the start to the goal, following decreasing potential values.

### Dijkstra vs A*

| Mode | `use_astar` | Behavior |
|------|-------------|----------|
| Dijkstra | `false` (default) | Expands cells in order of total cost from the goal outward, like a flood fill. Explores more cells but **guarantees the globally optimal cost path** on the grid. |
| A* | `true` | Uses a heuristic (Euclidean distance) to focus the search toward the start. Faster for long-distance planning but path quality depends on heuristic accuracy. May produce slightly suboptimal paths in complex environments. |

**Recommendation:** Use Dijkstra (`use_astar: false`) for reliability. Switch to A* only if planning time is a bottleneck on very large maps, and verify path quality doesn't degrade.

### Potential Field Behavior

The potential field naturally routes around obstacles and through low-cost regions. The gradient descent path follows the steepest descent through this field. This approach has a useful property: the potential field can be reused for multiple start points with the same goal, though Nav2 recomputes it each cycle.

Because the path follows a continuous gradient, it tends to be smoother than cell-center A* paths, but it can still have sharp turns at obstacle corners or narrow passages.

## Key Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_astar` | bool | false | Use A* instead of Dijkstra. |
| `tolerance` | double | 0.5 | Goal tolerance in **meters**. If the exact goal is unreachable (inside an obstacle), accept any cell within this distance. Note: unlike SMAC planners, this is in meters, not cells. |
| `allow_unknown` | bool | true | Allow planning through unknown space (NO_INFORMATION cells). |

## Planner Server Parameters

These affect all planners, not just NavFn:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `expected_planner_frequency` | double | 20.0 | Expected rate of planning requests. Used to detect if the planner is falling behind. Set to 0.0 to disable the warning. |
| `costmap_update_timeout` | double | — | How long to wait for a costmap update before planning. |

## Complete YAML Configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

## Why NavFn Is Still a Solid Default

1. **Simplicity:** Three parameters. Almost nothing to tune.
2. **Speed:** Dijkstra on a 2D grid is fast. For typical indoor maps (< 50m × 50m at 0.05m resolution = 1M cells), planning completes in milliseconds.
3. **Reliability:** The Dijkstra mode guarantees optimal grid paths. It has been battle-tested across thousands of ROS 1 and ROS 2 deployments.
4. **Controller handles kinematics:** Modern controllers (MPPI, DWB, RPP) are sophisticated enough to follow a grid-planned path while respecting kinematic constraints. The planner doesn't need to produce kinematically feasible paths.
5. **Low memory:** Only the 2D potential field is stored. No angle dimension, no lookup tables.

## Path Characteristics

NavFn paths have these properties:
- **Grid-aligned:** Paths follow the costmap grid, resulting in staircase-like segments on diagonals.
- **Optimal cost:** With Dijkstra, the path minimizes total traversal cost (obstacle proximity + distance).
- **Sharp turns:** At corridor junctions or obstacle corners, paths can have abrupt direction changes.
- **No kinematic constraints:** The planner doesn't know the robot's turning radius or max velocity.

### Improving Path Quality

To smooth NavFn output, add the **smoother server**:

```yaml
smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true
```

Or use the **constrained smoother** for more control:

```yaml
smoother_server:
  ros__parameters:
    smoother_plugins: ["constrained_smoother"]
    constrained_smoother:
      plugin: "nav2_constrained_smoother::ConstrainedSmoother"
      w_smooth: 0.3
      w_cost: 0.015
      w_data: 0.2
```

## Multiple Planner Configuration

You can configure multiple planners and select between them at runtime:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased", "HybridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
    HybridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybridAstar"
      minimum_turning_radius: 0.20
      # ... other params
```

The `NavigateToPose` action can specify which planner to use via the `planner_id` field.

## Troubleshooting

- **"Failed to compute plan" for reachable goals:** The goal may be inside an inflated cell. Increase `tolerance` or check the inflation radius. Use `rviz2` to inspect the costmap at the goal location.
- **Planner is slow on large maps:** Switch to A* (`use_astar: true`). Or downsample the global costmap by using a coarser resolution for the global costmap.
- **Path cuts through narrow gaps the robot can't fit:** The costmap inflation radius is too small. Increase `cost_scaling_factor` or `inflation_radius` in the inflation layer.
- **Path goes through unknown space when it shouldn't:** Set `allow_unknown: false`.
- **"Planner frequency is lower than expected" warning:** Either increase `expected_planner_frequency` limit, set it to 0.0 to disable, or investigate why planning is slow (large map, high costmap update rate).

## Migration Notes

If migrating from NavFn to an SMAC planner:
- NavFn `tolerance` is in **meters**; SMAC `tolerance` is in **costmap cells**.
- NavFn has no smoother parameters; SMAC planners have built-in smoothers.
- NavFn paths may need external smoothing; SMAC paths are typically smoother out of the box.
- SMAC planners have more parameters to tune but offer more control over path characteristics.
