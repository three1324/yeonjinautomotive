<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: loop-closure
description: 'Understand and tune loop closure. Use when debugging SLAM drift, configuring loop closure detection, or handling false loop closures.'
---

# Loop Closure in SLAM

## What Is Loop Closure?

Loop closure detects when the robot returns to a previously visited location and corrects the accumulated odometry/scan-matching drift. Without loop closure, a map of a closed loop (e.g., driving around a building) will show a gap or overlap where the path should close. Loop closure aligns the revisited area with the original, then runs a **global optimization** to redistribute the error across the entire trajectory.

## The Process

1. **Detection**: The current scan is compared against distant (non-sequential) nodes in the pose graph.
2. **Verification**: A match score must exceed a threshold to avoid false positives.
3. **Constraint addition**: A new edge is added to the pose graph connecting the current node to the matched distant node.
4. **Global optimization**: The entire graph is re-optimized with the new constraint, redistributing accumulated drift.

## Loop Closure in Google Cartographer

### Constraint Builder Parameters

```lua
POSE_GRAPH.constraint_builder.min_score = 0.65
-- Minimum scan-to-submap match score to accept a loop closure constraint.
-- Range: 0.0-1.0. Higher = fewer but more reliable closures.
-- Start at 0.55, increase if you see false closures.

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
-- Stricter threshold for matching against non-adjacent submaps.
-- Should be >= min_score. Prevents far-away false matches.

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
-- Fraction of finished nodes to try matching against each submap.
-- 1.0 = try all nodes (thorough but slow). 0.3 = try 30%.

POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
-- Maximum distance (m) between nodes to consider for loop closure.
-- Reduce for indoor environments to avoid matching across walls.

POSE_GRAPH.constraint_builder.log_matches = true
-- Log successful and failed matches. Useful for debugging.
```

### Scan-to-Submap Matching

Cartographer uses a two-step process:
1. **Branch-and-bound search**: A fast, hierarchical search over the submap to find candidate poses.
2. **Ceres refinement**: The best candidate is refined using nonlinear optimization.

The match score reflects how well the current scan aligns with the submap. A score of 1.0 means perfect alignment.

### Global Optimization

```lua
POSE_GRAPH.optimize_every_n_nodes = 90
-- Run global optimization every N new nodes. Set to 0 to disable automatic optimization.
-- Lower = more frequent optimization, smoother map but higher CPU.

POSE_GRAPH.optimization_problem.huber_scale = 1e1
-- Robust loss function scale. Reduces the influence of outlier constraints.
-- Increase if loop closures cause visible map distortion.

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
-- How much to trust local SLAM poses during optimization.
-- Higher = local SLAM poses are more rigid.
```

### Visualization in Cartographer

In RViz, subscribe to `/constraint_list` (MarkerArray):
- **Green lines**: Intra-submap constraints (sequential scan matching).
- **Blue lines**: Inter-submap constraints (loop closures).
- **Absence of blue lines** in areas you've revisited means loop closure isn't triggering—lower `min_score`.
- **Blue lines connecting wrong areas** means false positives—raise `min_score`.

## Loop Closure in SLAM Toolbox

### Chain Size and Response Thresholds

```yaml
do_loop_closing: true
loop_match_minimum_chain_size: 10
# Minimum chain of connected nodes before considering a loop closure.
# A chain is a sequence of consecutive nodes. Prevents premature closures
# when the robot has barely moved.

loop_match_maximum_variance_coarse: 3.0
# Maximum allowed variance in the coarse match. Rejects matches with
# high uncertainty.

loop_match_minimum_response_coarse: 0.35
# Minimum response from the coarse correlation scan matcher.
# Range: 0.0-1.0. Higher = more selective.

loop_match_minimum_response_fine: 0.45
# Minimum response from the fine matcher (after coarse match passes).
# Must be >= coarse threshold.
```

### Correlation Scan Matching for Loop Detection

```yaml
loop_search_space_dimension: 8.0
# Size of the search window (m) for loop closure candidates.
# Larger window finds closures even with significant drift, but is slower.

loop_search_space_resolution: 0.05
# Resolution of the loop closure search grid.
# Finer resolution = more accurate but slower.

loop_search_space_smear_deviation: 0.03
# Gaussian smear applied to the search space. Helps robustness.

loop_search_maximum_distance: 3.0
# Maximum distance between candidate nodes for loop closure consideration.
```

## False Positive Loop Closures

The most damaging SLAM failure. Two areas that look similar (long corridors, symmetric rooms) are incorrectly matched, and the optimizer warps the map to force them together.

### Symptoms
- Sudden map distortion after the robot traverses a specific area.
- Walls bending or rooms collapsing.
- "Map tearing"—a previously correct area is pulled out of alignment.

### Causes
- `min_score` / `minimum_response` too low.
- Symmetric environments (identical corridors, repeating office layouts).
- Featureless areas (long blank walls).
- Lidar seeing through glass into areas that resemble other parts of the map.

### Mitigation Strategies

| Strategy | Cartographer | SLAM Toolbox |
|----------|-------------|--------------|
| Raise match threshold | `min_score: 0.70+` | `loop_match_minimum_response_fine: 0.55+` |
| Reduce search distance | `max_constraint_distance: 10.0` | `loop_search_maximum_distance: 2.0` |
| Increase chain requirement | N/A | `loop_match_minimum_chain_size: 20` |
| Reduce sampling | `sampling_ratio: 0.1` | N/A |
| Increase Huber scale | `huber_scale: 5e1` | N/A (use Ceres loss function) |
| Disable loop closure entirely | `optimize_every_n_nodes: 0` | `do_loop_closing: false` |

### Visual Verification

Before trusting a map, verify loop closures:
1. In RViz, overlay the constraints on the map.
2. Check that blue/loop-closure constraints connect areas that are actually the same place.
3. If a closure looks wrong, increase thresholds and re-map.

## Error Redistribution

After a loop closure, the optimizer minimizes the total error across all constraints. The drift accumulated over the loop is distributed proportionally across all nodes in the path:

```
Before closure:
  Start ---[drift accumulating]--→ End (gap between start and end)

After closure:
  Start ---[error spread evenly]--→ End (aligned with start)
```

Nodes near the middle of the loop path are adjusted the most. This is why intermediate scans may shift slightly—it's normal and expected behavior. The `huber_scale` parameter controls how aggressively outlier constraints are downweighted during this redistribution.

## Tuning Workflow

1. Start with default thresholds and map the environment.
2. Inspect the map for distortions. If present, check constraint visualization.
3. If false closures exist, **increase thresholds** and re-map.
4. If legitimate closures are missed (map has gaps), **decrease thresholds** carefully.
5. Test with multiple laps. A robust configuration should produce identical maps across 3+ laps.
