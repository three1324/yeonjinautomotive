<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: planner-comparison
description: 'Compare Nav2 planners (NavFn, SMAC 2D, Hybrid-A*, Lattice, Theta*). Use when choosing a planner for your robot type and environment.'
---

# Planner Comparison and Selection Guide

## Decision Matrix

| Feature | NavFn | SMAC 2D | SMAC Hybrid-A* | SMAC Lattice | Theta* |
|---------|-------|---------|-----------------|--------------|--------|
| **Algorithm** | Dijkstra/A* + potential field | A* + Wavefront heuristic | Hybrid-A* in SE2 | Lattice search in SE2 | A* + line-of-sight |
| **Search space** | 2D (x, y) | 2D (x, y) | 3D (x, y, θ) | 3D (x, y, θ) | 2D (x, y) |
| **Computation speed** | ★★★★★ | ★★★★☆ | ★★★☆☆ | ★★☆☆☆ | ★★★★☆ |
| **Path smoothness** | ★★☆☆☆ | ★★★☆☆ | ★★★★★ | ★★★★★ | ★★★★☆ |
| **Kinematic feasibility** | None | None | Dubins/Reeds-Shepp | Custom primitives | None |
| **Configuration complexity** | ★☆☆☆☆ (3 params) | ★★☆☆☆ (~10 params) | ★★★☆☆ (~15 params) | ★★★★☆ (params + lattice file) | ★★☆☆☆ (5 params) |
| **Memory usage** | Low | Low | High (3D + lookup table) | High (3D + primitives) | Low |
| **Built-in smoother** | No | Yes | Yes | Yes | No (inherently smoother) |
| **Handles reverse** | N/A | N/A | Yes (Reeds-Shepp) | Yes (if primitives include it) | N/A |
| **Goal orientation** | No | `use_final_approach_orientation` | Yes (SE2 goal) | Yes (SE2 goal) | No |
| **Costmap downsampling** | No | Yes | Yes | Yes | No |

## Recommendations by Robot Type

### Differential Drive (Indoor) — e.g., the robot

**Best choice: NavFn or SMAC 2D**

Differential drive robots can spin in place, so kinematic feasibility from the planner is rarely necessary. The controller (MPPI, DWB, or RPP) handles turning in place and arc following.

- **NavFn** when simplicity is paramount. Add a smoother server if paths are too jagged.
- **SMAC 2D** when you want built-in smoothing and costmap downsampling for larger maps.
- **Theta*** if NavFn paths cause controller oscillation due to frequent direction changes.

**Upgrade to SMAC Hybrid-A*** only if:
- You need the planner to produce smooth, arc-based paths (e.g., for aesthetic patrol routes).
- You're doing docking maneuvers where approach angle matters.
- The controller struggles to smooth NavFn paths in constrained spaces.

If using Hybrid-A* with diff-drive, set `minimum_turning_radius: 0.15–0.30` and `motion_model_for_search: "REEDS_SHEPP"`.

### Ackermann Steering (Car-like)

**Best choice: SMAC Hybrid-A* or SMAC Lattice**

Ackermann robots have a real minimum turning radius and cannot spin in place. The planner must produce kinematically feasible paths.

- **Hybrid-A*** is the standard choice. Set `minimum_turning_radius` to the actual mechanical limit.
- **Lattice** if you need exact kinematic fidelity (e.g., the robot has different forward/reverse turning characteristics).

### Holonomic / Omnidirectional

**Best choice: SMAC 2D or Theta***

Omnidirectional robots can move in any direction, so 2D planners are ideal. The robot can follow any 2D path regardless of heading.

- **SMAC 2D** for grid-optimal paths with smoothing.
- **Theta*** for straighter paths with fewer waypoints.

## When the Planner Isn't the Problem

Before switching planners, verify that the real issue isn't elsewhere:

### Costmap Problems (More Common Than Planner Problems)
- **Path cuts through obstacles:** Costmap isn't being updated (sensor topic not publishing, transform broken).
- **Path goes too close to walls:** Inflation radius too small. Increase `inflation_radius` and tune `cost_scaling_factor`.
- **Path avoids open areas:** Stale obstacles in costmap. Check `observation_sources` clearing parameters.
- **Robot can't fit through passages it should:** Inflation radius too large for the robot's actual footprint.

### Controller Problems
- **Robot oscillates along the path:** Controller gains too aggressive, or path has too many direction changes (try Theta* or add smoother).
- **Robot doesn't follow the path tightly:** Controller lookahead distance too large.
- **Robot stops before reaching goal:** Goal tolerance mismatch between planner and controller/BT.

### Transform / Timing Problems
- **Intermittent planning failures:** TF tree has delays. Check `transform_tolerance`.
- **Plan is offset from the map:** Frame mismatch between planner costmap and map.

## Switching Planners at Runtime

Configure multiple planners in the planner server:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["NavFnPlanner", "SmacPlanner"]
    NavFnPlanner:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
    SmacPlanner:
      plugin: "nav2_smac_planner::SmacPlannerHybridAstar"
      minimum_turning_radius: 0.20
      motion_model_for_search: "REEDS_SHEPP"
      angle_quantization_bins: 72
      max_planning_time: 5.0
      cost_penalty: 2.0
```

Select at runtime via the BT action node:

```xml
<ComputePathToPose goal="{goal}" path="{path}" planner_id="SmacPlanner"/>
```

Or in the `NavigateToPose` action goal:

```python
goal = NavigateToPose.Goal()
goal.pose = target_pose
goal.planner_id = "SmacPlanner"  # or "NavFnPlanner"
```

## Upgrade Path for the robot

Current: **NavFnPlanner** (Dijkstra mode)

Recommended evaluation order if considering a switch:
1. **Add smoother server** to the existing NavFn setup. This addresses most path quality complaints with zero planner changes.
2. **Try Theta*** as a drop-in replacement. Same 2D search space, minimal parameter changes, naturally smoother paths.
3. **Evaluate SMAC 2D** if you want built-in smoothing and potential costmap downsampling.
4. **SMAC Hybrid-A*** only if the above don't satisfy path quality requirements, or for specific maneuvers (docking, precise approach angles).

Each step adds complexity. Only proceed to the next if the current step doesn't solve the problem.

## Quick Benchmark Comparison (Typical Indoor 20m × 20m Map)

| Planner | Planning Time | Path Length | Direction Changes |
|---------|--------------|-------------|-------------------|
| NavFn (Dijkstra) | 1–5 ms | Optimal | Many (grid-aligned) |
| NavFn (A*) | 0.5–3 ms | Near-optimal | Many |
| SMAC 2D | 1–8 ms | Optimal | Fewer (smoother) |
| Theta* | 2–10 ms | Near-optimal | Few (any-angle) |
| SMAC Hybrid-A* | 10–100 ms | Longer (arcs) | Smooth curves |
| SMAC Lattice | 20–200 ms | Depends on primitives | Smooth curves |

*Times are approximate for a 400×400 cell costmap with moderate obstacle density.*
