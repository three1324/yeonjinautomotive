<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: dwb-critics
description: 'Configure DWB critic plugins. Use when tuning GoalDist, PathDist, ObstacleFootprint, GoalAlign, PathAlign, or RotateToGoal critics.'
---

# DWB Critics — Plugin Reference

## Scoring Model

Each DWB critic plugin has a `scale` parameter that directly multiplies its raw score. The total trajectory score is the sum of all critics' weighted scores. The trajectory with the **lowest** total score wins.

Unlike MPPI's `cost_weight` / `cost_power` model, DWB uses a simple linear scale:
```
trajectory_score = Σ (critic_i.scale × critic_i.raw_score(trajectory))
```

Higher `scale` = more influence from that critic. The absolute values matter less than the ratios between critics.

---

## PathDist

**Purpose**: Scores each trajectory based on distance to the nearest point on the global path. The primary path-tracking critic.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 32.0 | Weight multiplier |

**Behavior**: For each trajectory point, computes distance to the closest point on the global plan. Aggregates (typically sums or averages) across all points. Lower distance = lower score = better.

**Tuning**: Highest-scaled critic for path following. Increase to 40+ for tight corridor tracking.

---

## GoalDist

**Purpose**: Scores distance from the trajectory endpoint to the goal. Provides goal attraction.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 24.0 | Weight multiplier |

**Behavior**: Measures Euclidean distance from the last simulated position in the trajectory to the goal. Ensures the robot makes progress toward the goal.

**Tuning**: Keep slightly lower than PathDist so the robot tracks the path rather than cutting directly to the goal.

---

## GoalAlign

**Purpose**: Penalizes trajectories whose endpoint heading doesn't align with the goal orientation.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 24.0 | Weight multiplier |
| `forward_point_distance` | float | 0.325 | Distance ahead of robot to project for alignment check |

**Behavior**: Projects a point `forward_point_distance` ahead of the trajectory endpoint and measures angular deviation from the goal heading. Helps the robot arrive at the goal facing the correct direction.

**Tuning**: Increase if the robot consistently arrives with wrong heading. The `forward_point_distance` should roughly match the robot's length.

---

## PathAlign

**Purpose**: Penalizes trajectories whose heading deviates from the path tangent direction. Keeps the robot oriented along the path.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 32.0 | Weight multiplier |
| `forward_point_distance` | float | 0.325 | Projection distance for alignment |

**Behavior**: At each trajectory point, measures angular deviation between the robot's heading and the tangent of the nearest path segment. Prevents the robot from crabbing (moving laterally) along the path.

**Tuning**: High scale keeps the robot aligned. Lower for open areas where strict alignment isn't necessary.

---

## BaseObstacle

**Purpose**: Checks trajectory points against the costmap for obstacle proximity.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 0.02 | Weight multiplier (intentionally low) |
| `sum_scores` | bool | false | If true, sum costs across all trajectory points; if false, use max |

**Behavior**: For each trajectory point, looks up the costmap value. With `sum_scores: false`, the trajectory's obstacle score is the maximum costmap value encountered along it. With `sum_scores: true`, it accumulates — penalizing trajectories that spend more time near obstacles.

**Critical**: A low `scale` (0.02) already provides strong obstacle avoidance because costmap values (0–254) are large numbers. Scale of 0.02 × costmap value 200 = 4.0, which is significant relative to other critics.

**Tuning**: Increase very cautiously. Too high and the robot refuses to navigate near any inflated cells.

---

## ObstacleFootprint

**Purpose**: Full robot footprint collision checking against the costmap. More accurate than BaseObstacle's point check but significantly more expensive.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 0.02 | Weight multiplier |

**Behavior**: At each trajectory point, stamps the robot's full footprint polygon onto the costmap and checks all cells underneath. If any lethal cell is under the footprint, the trajectory is rejected (infinite cost). Inscribed cells receive high but finite cost.

**Use when**: Robot has a non-circular or large footprint where point-based checking misses clipping corners on obstacles.

**CPU cost**: Scales with footprint polygon vertices × trajectory points. For a 6-vertex polygon over 40 trajectory points, it's ~240 costmap lookups per trajectory × number of trajectories.

---

## RotateToGoal

**Purpose**: Near the goal, encourages the robot to rotate to match the goal heading.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 32.0 | Weight multiplier |
| `slowing_factor` | float | 5.0 | How aggressively to slow linear velocity near goal |
| `lookahead_time` | float | -1.0 | Time ahead to check; -1 = disabled |
| `xy_goal_tolerance` | float | 0.25 | Distance within which to start final rotation |
| `trans_stopped_velocity` | float | 0.25 | Velocity threshold for "stopped" |

**Behavior**: When the robot is within `xy_goal_tolerance` of the goal, this critic strongly favors trajectories that rotate to match the goal heading while slowing forward motion by `slowing_factor`.

**Tuning**: If robot overshoots the goal before rotating: decrease `xy_goal_tolerance` or increase `slowing_factor`.

---

## Oscillation

**Purpose**: Detects and penalizes oscillatory behavior — the robot alternating between opposite velocity commands.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 1.0 | Penalty weight |

**Behavior**: Tracks recent velocity command history. If the robot has been alternating between positive and negative vx (forward/backward) or vtheta (left/right) for several consecutive cycles, trajectories continuing the oscillation are penalized.

**Tuning**: Usually leave at 1.0. The oscillation detector can be too sensitive in environments with multiple narrow gaps. Decrease scale if the robot hesitates at openings.

---

## PreferForward

**Purpose**: Penalizes reverse and lateral motion. Essential for differential-drive robots.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 1.0 | Weight (raw penalty is already significant) |
| `penalty` | float | 1.0 | Direct penalty for backward trajectories |
| `strafe_penalty` | float | 1.0 | Penalty for lateral motion |
| `strafe_x` | float | 0.1 | Lateral velocity threshold |
| `strafe_theta` | float | 0.2 | Angular component of strafe |
| `theta_scale` | float | 10.0 | Weight on rotational component |

**Behavior**: Assigns penalties to trajectories with negative linear velocity (reverse) and trajectories with significant lateral velocity components.

---

## TwirlingCritic

**Purpose**: Prevents excessive in-place spinning during transit.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 1.0 | Weight |

**Behavior**: Penalizes trajectories with high angular velocity, especially when combined with low linear velocity. Keeps the robot from spinning unnecessarily.

---

## MapGrid

**Purpose**: General-purpose grid-based scoring using a precomputed navigation function on the costmap.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scale` | float | 1.0 | Weight |

**Behavior**: Computes a wavefront propagation from the goal (or path) to create a cost grid. Each trajectory point is scored by its position in this grid. PathDist and GoalDist are special cases of MapGrid.

---

## Recommended Scales for Indoor Differential Drive

```yaml
# Balanced indoor configuration
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
Oscillation:
  scale: 1.0
```

## Critic Ordering with Short-Circuit

When `short_circuit_trajectory_evaluation: true`, the controller stops evaluating a trajectory as soon as its partial score exceeds the current best total score. This means critic order affects performance:

1. **Place collision critics first** (`BaseObstacle` or `ObstacleFootprint`): quickly rejects infeasible trajectories.
2. **Place expensive critics last**: they only run on trajectories that survived cheaper checks.

Recommended ordering:
```yaml
critics:
  - "BaseObstacle"    # Reject collisions immediately
  - "Oscillation"     # Cheap, eliminates oscillating
  - "RotateToGoal"    # Active near goal
  - "PathDist"        # Main path tracking
  - "GoalDist"        # Goal attraction
  - "PathAlign"       # Heading alignment with path
  - "GoalAlign"       # Final heading alignment
```
