<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: theta-star-planner
description: 'Configure Nav2 Theta* planner. Use when generating any-angle smooth paths with line-of-sight optimization.'
---

# Theta* Planner

## Plugin
```
nav2_theta_star_planner::ThetaStarPlanner
```

## Algorithm
Theta* is an **any-angle path planning** algorithm that extends A* with line-of-sight checks. In standard A*, paths are constrained to grid edges (4 or 8-connected neighbors), producing staircase-like paths on diagonal trajectories. Theta* removes this constraint: when a cell expansion occurs, it checks whether the parent of the current node has direct line-of-sight to the neighbor being expanded. If so, it updates the neighbor's parent to skip intermediate nodes, producing **straighter paths with fewer unnecessary turns**.

### How Line-of-Sight Works

During expansion of node `s`:
1. For each neighbor `s'` of `s`, compute the tentative cost through `s` (standard A*).
2. Also check: does `parent(s)` have line-of-sight to `s'`? (Bresenham line check through the costmap.)
3. If yes, and if the path `parent(s) → s'` is cheaper than `s → s'`, set `parent(s') = parent(s)`.

This simple modification produces dramatically straighter paths because the planner can "see through" multiple cells at once.

### Compared to A*

```
A* path (8-connected):        Theta* path:
  S─┐                          S
    └─┐                         \
      └─┐                        \
        └─G                       G
```

The A* path follows grid edges step by step. The Theta* path takes a direct diagonal if the line of sight is clear.

## Key Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `how_many_corners` | int | 8 | Number of neighbors to explore per cell. **8** = 8-connected grid (diagonal movement), **4** = 4-connected (cardinal only). Use 8 for diagonal movement. |
| `w_euc_cost` | double | 1.0 | Weight for Euclidean distance component in the cost function. Higher values produce shorter paths. |
| `w_traversal_cost` | double | 2.0 | Weight for costmap traversal cost. Higher values make the planner avoid high-cost cells more aggressively. |
| `terminal_checking_interval` | int | 5000 | How often (in node expansions) to check if planning time has exceeded the limit. Lower values = more responsive timeout but slightly more overhead. |
| `allow_unknown` | bool | true | Plan through unknown cells. |

### Cost Function

The total cost for reaching a cell is:

```
f(s) = g(s) + h(s)
g(s) = parent_cost + w_euc_cost * euclidean_distance + w_traversal_cost * costmap_cost
h(s) = w_euc_cost * euclidean_distance_to_goal
```

**Tuning `w_euc_cost` vs `w_traversal_cost`:**
- `w_euc_cost` = 1.0, `w_traversal_cost` = 2.0 (default): Good balance. Paths avoid high-cost areas while remaining reasonably short.
- Increase `w_traversal_cost` to 5.0+: Paths strongly avoid costmap obstacles (stay far from walls), at the cost of longer paths.
- Increase `w_euc_cost` to 3.0+: Paths prioritize shortness over obstacle avoidance. May cut closer to walls.
- Both high: Slower planning (higher heuristic values cause more expansions).

## Complete YAML Configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_theta_star_planner::ThetaStarPlanner"
      how_many_corners: 8
      w_euc_cost: 1.0
      w_traversal_cost: 2.0
      terminal_checking_interval: 5000
      allow_unknown: true
```

## When To Use

**Good fit:**
- Want straighter paths than NavFn or SMAC 2D without the computational cost of Hybrid-A*.
- Indoor environments with open areas where diagonal and straight-line paths are natural.
- Robot kinematic constraints are handled by the controller, not the planner.
- Environments where grid-aligned staircase paths cause unnecessary controller oscillation.

**Poor fit:**
- Robots with non-holonomic constraints where kinematically feasible paths from the planner matter (use Hybrid-A*).
- Very cluttered environments where line-of-sight checks rarely succeed (Theta* degrades to A* with extra overhead).
- When you need guaranteed smoothness — Theta* paths have fewer turns but can still have abrupt direction changes at obstacle edges.

## Comparison with Other Planners

| Aspect | NavFn | SMAC 2D | Theta* |
|--------|-------|---------|--------|
| Path alignment | Grid-aligned | Grid-aligned + smoother | Any-angle (straight lines) |
| Turn count | High (staircase) | Medium (with smoother) | Low (line-of-sight) |
| Speed | Fast | Fast | Moderate (line-of-sight checks add overhead) |
| Kinematic feasibility | None | None | None |
| Configuration | 3 params | ~10 params | 5 params |

Theta* sits between the simplicity of NavFn and the sophistication of SMAC Hybrid-A*. It produces naturally smoother paths without requiring a separate smoother, while remaining in the 2D planning space.

## Path Smoothing

Theta* paths are smoother by construction than grid-based planners, but they still consist of straight-line segments with angular discontinuities. For additional smoothness, you can add the smoother server:

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

However, the benefit of a separate smoother is less pronounced with Theta* than with NavFn because the line-of-sight shortcuts already eliminate most unnecessary turns.

## Troubleshooting

- **Paths still look grid-aligned:** Check that `how_many_corners: 8` is set. With `how_many_corners: 4`, line-of-sight checks are limited to cardinal directions.
- **Planner is slower than expected:** In very cluttered environments, most line-of-sight checks fail, adding overhead without benefit. Consider SMAC 2D or NavFn for such cases.
- **Paths cut too close to obstacles:** Increase `w_traversal_cost` to penalize cells near obstacles more. Or increase the costmap inflation radius.
- **Planning fails for reachable goals:** Check `allow_unknown` and ensure the goal isn't inside an inflated obstacle. The planner has no explicit tolerance parameter like NavFn — the goal cell itself must be reachable.
- **Angular discontinuities at obstacle corners:** This is inherent to Theta*. The path connects straight segments at obstacle edges. Use a downstream smoother if the controller struggles with sharp turns.

## Implementation Notes

The line-of-sight check uses a Bresenham-like line drawing algorithm through the costmap. If any cell along the line has cost ≥ LETHAL_OBSTACLE, line-of-sight is blocked. This means inflated costs do NOT block line-of-sight — only lethal cells do. The cost of traversing through inflated cells is still accumulated in the path cost via `w_traversal_cost`.
