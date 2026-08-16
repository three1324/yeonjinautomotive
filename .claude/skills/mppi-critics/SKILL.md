<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: mppi-critics
description: 'Configure MPPI critic plugins. Use when tuning ConstraintCritic, CostCritic, GoalCritic, GoalAngleCritic, PathAlignCritic, PathFollowCritic, PathAngleCritic, PreferForwardCritic, or VelocityDeadbandCritic weights and parameters.'
---

# MPPI Critics — Exhaustive Reference

## How Critics Work

Every MPPI critic plugin receives all `batch_size` sampled trajectories and assigns a cost to each. Costs are summed across all active critics. The total cost per trajectory determines its weight in the Boltzmann-weighted average that produces the final velocity command.

Each critic has:
- `cost_power` (int): Exponent applied to the raw cost. Power=2 penalizes large deviations quadratically.
- `cost_weight` (float): Multiplicative weight on the final critic cost. This is the PRIMARY tuning lever.
- `enabled` (bool): Toggle the critic on/off without removing from the list.

Effective cost contribution = `cost_weight × (raw_cost ^ cost_power)`.

---

## ConstraintCritic

**Purpose**: Safety net that penalizes trajectories violating kinematic constraints (velocity or acceleration limits).

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cost_power` | int | 1 | Exponent on constraint violation magnitude |
| `cost_weight` | float | 4.0 | Penalty weight |

**Behavior**: For each trajectory point, checks if vx, vy, wz, or accelerations exceed configured limits. Adds a proportional penalty for each violation. This should always be enabled as a hard constraint enforcer.

**Tuning**: Rarely needs adjustment. If you see trajectories that violate limits in visualization, increase `cost_weight`.

---

## CostCritic

**Purpose**: Evaluates trajectories against the local costmap. This is the primary obstacle avoidance critic.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cost_power` | int | 1 | Exponent on costmap cost |
| `cost_weight` | float | 3.81 | Base weight |
| `critical_cost` | float | 300.0 | Costmap value that causes trajectory rejection |
| `collision_cost` | float | 1000000.0 | Penalty added when a point is in lethal collision |
| `consider_footprint` | bool | false | Use full robot footprint instead of point check |
| `near_goal_distance` | float | 1.0 | Distance from goal where cost sensitivity relaxes |
| `trajectory_point_step` | int | 2 | Evaluate every Nth trajectory point (1=all) |
| `near_collision_cost` | float | 253.0 | Costmap value considered "near collision" |

**Behavior**:
- For each evaluated trajectory point, looks up the costmap cell value at that (x,y).
- Costmap values: 0=free, 1-252=increasing cost (from inflation), 253=inscribed, 254=lethal, 255=unknown.
- Points with cost ≥ `critical_cost` get the `collision_cost` penalty.
- Points between `near_collision_cost` and `critical_cost` get a scaled penalty.
- Within `near_goal_distance`, penalty is reduced so the robot can approach goals near walls.

**`consider_footprint`**: When `true`, instead of checking a single point, the critic checks all cells under the robot's footprint polygon at each trajectory point. This is **3-10× more expensive** depending on footprint complexity. Use for robots with large or non-circular footprints. For circular robots, use `false` with proper inflation.

**`trajectory_point_step`**: Setting to 2 means only every other point is checked. Cuts CostCritic CPU in half. Safe for smooth trajectories at moderate speed. Set to 1 in tight environments.

**Tuning**:
- Too high `cost_weight`: Robot refuses to enter narrow passages, stops far from walls.
- Too low: Robot clips corners, brushes obstacles.
- If robot won't approach goals near walls: decrease `near_goal_distance` or reduce `cost_weight`.

---

## GoalCritic

**Purpose**: Attracts trajectories toward the navigation goal. Scores based on the Euclidean distance between each trajectory's terminal point and the goal.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cost_power` | int | 1 | Exponent on distance-to-goal |
| `cost_weight` | float | 5.0 | Attraction strength |
| `threshold_to_consider` | float | 1.4 | Distance (m) from goal to activate this critic |

**Behavior**: Only activates when the robot is within `threshold_to_consider` meters of the goal. Computes Euclidean distance from the terminal state of each trajectory to the goal position. Higher weight = more aggressive goal-seeking.

**Tuning**:
- If robot orbits the goal without reaching it: increase `cost_weight` or increase `threshold_to_consider`.
- If robot rushes to the goal ignoring path quality: decrease `cost_weight`.

---

## GoalAngleCritic

**Purpose**: Penalizes trajectories whose terminal heading deviates from the desired goal orientation.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cost_power` | int | 1 | Exponent on angular error |
| `cost_weight` | float | 3.0 | Heading alignment strength |
| `threshold_to_consider` | float | 0.5 | Distance (m) from goal to activate |

**Behavior**: Only active within `threshold_to_consider` meters. Computes angular difference between the trajectory's terminal heading and the goal's orientation (from the goal pose quaternion). Essential for tasks requiring precise final heading (docking, facing a door).

**Tuning**:
- If robot reaches goal position but wrong heading: increase `cost_weight` or `threshold_to_consider`.
- If robot takes excessive time rotating at goal: decrease `cost_weight`.

---

## PathAlignCritic

**Purpose**: Keeps the robot ON the planned path. Penalizes lateral deviation from the global plan. **This is typically the highest-weighted critic.**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cost_power` | int | 1 | Exponent on lateral deviation |
| `cost_weight` | float | 14.0 | Path-following fidelity weight |
| `max_path_occupancy_ratio` | float | 0.07 | If more than this fraction of path is in collision, disable critic |
| `trajectory_point_step` | int | 4 | Evaluate every Nth trajectory point |
| `threshold_to_consider` | float | 0.5 | Distance from goal to deactivate (let GoalCritic take over) |
| `offset_from_furthest` | int | 20 | Skip this many path points from the furthest reached |
| `use_path_orientations` | bool | false | Also penalize heading deviation from path tangent |

**Behavior**:
- For each evaluated trajectory point, computes the minimum perpendicular distance to the global path.
- Higher weight forces the robot to track the path closely.
- `offset_from_furthest` prevents the robot from "chasing" a point too far ahead on the path; it looks at a point near the current furthest reached.
- `max_path_occupancy_ratio`: Safety valve — if the global path passes through many obstacles (e.g., costmap updated after planning), the path is probably stale and the critic should not force the robot to follow a bad path.
- `use_path_orientations`: When true, also penalizes heading deviation from the path tangent direction at each point. This is more expensive and can cause issues at sharp turns but gives tighter tracking.

**Tuning**:
- This is THE critic to adjust first for path-tracking fidelity.
- Too high: Robot refuses to deviate even when obstacles block the path → recovery behaviors trigger excessively.
- Too low: Robot takes shortcuts, cuts corners, wanders off path.
- For narrow hallways: increase to 16–20.
- For open areas with obstacles: 10–14 is fine.

---

## PathFollowCritic

**Purpose**: Encourages the robot to make forward progress along the path. Penalizes trajectories that don't advance the "furthest reached point" index on the path.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cost_power` | int | 1 | Exponent |
| `cost_weight` | float | 5.0 | Progress incentive weight |
| `offset_from_furthest` | int | 5 | Target this many points ahead of furthest reached |
| `threshold_to_consider` | float | 1.4 | Deactivate within this distance to goal |

**Behavior**: Computes how far along the path each trajectory would advance. Trajectories that stall or regress receive penalties. The `offset_from_furthest` sets a target ahead of the current position, creating a "pull" forward.

**Tuning**:
- If robot lingers in one spot: increase `cost_weight` or `offset_from_furthest`.
- If robot rushes forward ignoring alignment: decrease `cost_weight` relative to `PathAlignCritic`.

---

## PathAngleCritic

**Purpose**: Penalizes trajectories where the robot's heading deviates significantly from the direction toward the next path waypoint.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cost_power` | int | 1 | Exponent |
| `cost_weight` | float | 2.0 | Heading-toward-path weight |
| `offset_from_furthest` | int | 4 | Target path point index offset |
| `threshold_to_consider` | float | 0.5 | Deactivate near goal |
| `max_angle_to_furthest` | float | 1.0 | Angular threshold (radians) beyond which penalty applies |
| `mode` | int | 0 | 0=forward only, 1=also consider reverse approaches |

**Behavior**: Computes the angle between the robot's current heading and the vector pointing toward the target path point. If this angle exceeds `max_angle_to_furthest`, a penalty is applied. Prevents the robot from driving sideways or backwards along the path.

**`mode=1` (reverse)**: For robots that frequently need to back up (e.g., in dead ends), mode=1 also considers whether the trajectory approaches the target point in reverse, reducing the penalty for well-aimed reverse motion.

**Tuning**:
- If robot turns excessively before moving: decrease `cost_weight` or increase `max_angle_to_furthest`.
- If robot drives at skewed angles: increase `cost_weight`.

---

## PreferForwardCritic

**Purpose**: Directly penalizes negative linear velocity (reverse motion). Essential for differential-drive robots.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cost_power` | int | 1 | Exponent |
| `cost_weight` | float | 5.0 | Reverse penalty strength |
| `threshold_to_consider` | float | 0.5 | Distance from goal to deactivate |

**Behavior**: For each trajectory, if the average or terminal linear velocity is negative, applies a penalty proportional to `cost_weight`. Near the goal (within `threshold_to_consider`), the critic deactivates so the robot can back into tight goal positions if needed.

**Tuning**:
- Essential for diff-drive; always enable.
- If robot should never reverse: set very high (20+) and `threshold_to_consider: 0.0`.
- If occasional reverse is OK (backing out of dead ends): moderate weight (3–5) with threshold.

---

## TwirlingCritic

**Purpose**: Penalizes excessive rotational velocity during transit. Prevents unnecessary spinning.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `twirling_cost_power` | int | 1 | Exponent on angular velocity |
| `twirling_cost_weight` | float | 10.0 | Spin penalty weight |

**Behavior**: Proportional to the absolute angular velocity in each trajectory. High angular velocities during forward motion get penalized. This prevents the MPPI optimizer from finding "spin while moving" trajectories that technically satisfy other critics.

**Tuning**: Usually 10.0 is fine. Reduce if the robot needs to execute tight turns at speed.

---

## VelocityDeadbandCritic

**Purpose**: Penalizes trajectories in the motor deadband — velocities too small for actuators to respond.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cost_power` | int | 1 | Exponent |
| `cost_weight` | float | 35.0 | Deadband penalty weight |
| `deadband_velocities` | list | [0.05, 0.05, 0.05] | `[vx_db, vy_db, wz_db]` thresholds |

**Behavior**: If a trajectory commands a velocity below the deadband threshold (but above zero), the motors receive a command they cannot execute. This causes the robot to stall or twitch. The critic penalizes such trajectories, forcing MPPI to choose either full-stop or above-deadband commands.

**Tuning**: Set `deadband_velocities` to match your motor controller's actual deadband. For a robot with RoboClaw, measure the minimum velocity that produces actual wheel movement.

---

## Critical Interactions Between Critics

### PathAlignCritic vs GoalCritic Near the Goal

**Problem**: As the robot approaches the goal, PathAlignCritic wants it to stay on the path, while GoalCritic wants it to converge on the goal point. If the goal is slightly off-path (common with costmap replanning), the critics fight.

**Solution**: Set `PathAlignCritic.threshold_to_consider` and `PathFollowCritic.threshold_to_consider` to a nonzero distance (0.5–1.4m). When the robot is within this distance, PathAlignCritic disengages and GoalCritic takes over. GoalAngleCritic also has its own threshold for final heading alignment.

### CostCritic vs PathAlignCritic in Narrow Passages

**Problem**: In a narrow corridor, the costmap has high costs near walls (from inflation). CostCritic penalizes being near walls, but PathAlignCritic forces the robot to follow a path that goes through the corridor center where inflation costs exist.

**Solution**: Balance `CostCritic.cost_weight` (3–5) against `PathAlignCritic.cost_weight` (10–14). PathAlign should dominate so the robot tracks the path through the corridor rather than refusing to enter. Ensure inflation radius is not excessive for your corridors.

### PreferForwardCritic vs Recovery

**Problem**: When the robot needs to back out of a dead end, PreferForwardCritic fights recovery behaviors that command reverse.

**Solution**: Set `threshold_to_consider` on PreferForwardCritic so it disengages near the goal. For general reverse capability, keep cost_weight moderate (3–5) rather than extreme.

---

## Recommended Weight Profiles

### Open Indoor (living room, wide hallways)

```yaml
ConstraintCritic:
  cost_weight: 4.0
CostCritic:
  cost_weight: 3.81
  consider_footprint: false
GoalCritic:
  cost_weight: 5.0
  threshold_to_consider: 1.4
GoalAngleCritic:
  cost_weight: 3.0
  threshold_to_consider: 0.5
PathAlignCritic:
  cost_weight: 10.0
  threshold_to_consider: 0.5
PathFollowCritic:
  cost_weight: 5.0
PathAngleCritic:
  cost_weight: 2.0
PreferForwardCritic:
  cost_weight: 5.0
```

### Narrow Hallway (< 1m wide)

```yaml
ConstraintCritic:
  cost_weight: 4.0
CostCritic:
  cost_weight: 5.0
  consider_footprint: true
  trajectory_point_step: 1
GoalCritic:
  cost_weight: 5.0
  threshold_to_consider: 1.0
GoalAngleCritic:
  cost_weight: 3.0
  threshold_to_consider: 0.4
PathAlignCritic:
  cost_weight: 18.0
  threshold_to_consider: 0.4
  offset_from_furthest: 10
PathFollowCritic:
  cost_weight: 7.0
  offset_from_furthest: 3
PathAngleCritic:
  cost_weight: 3.0
PreferForwardCritic:
  cost_weight: 5.0
```

### Cluttered Room (furniture, dynamic obstacles)

```yaml
ConstraintCritic:
  cost_weight: 4.0
CostCritic:
  cost_weight: 6.0
  consider_footprint: true
  trajectory_point_step: 1
  near_goal_distance: 0.5
GoalCritic:
  cost_weight: 6.0
  threshold_to_consider: 1.0
GoalAngleCritic:
  cost_weight: 3.5
  threshold_to_consider: 0.5
PathAlignCritic:
  cost_weight: 12.0
  threshold_to_consider: 0.5
  max_path_occupancy_ratio: 0.15
PathFollowCritic:
  cost_weight: 4.0
PathAngleCritic:
  cost_weight: 2.5
PreferForwardCritic:
  cost_weight: 5.0
TwirlingCritic:
  twirling_cost_weight: 15.0
VelocityDeadbandCritic:
  cost_weight: 35.0
  deadband_velocities: [0.05, 0.05, 0.05]
```

---

## Tuning Methodology

1. **Start with the "Open Indoor" profile** — it is the safest baseline.
2. **Enable visualization** (`visualize: true`) and watch trajectories in RViz2.
3. **Observe pathologies**:
   - Robot cuts corners → increase `PathAlignCritic.cost_weight`.
   - Robot won't enter narrow spaces → decrease `CostCritic.cost_weight` or reduce inflation.
   - Robot oscillates near goal → increase `GoalCritic.cost_weight`, check `threshold_to_consider` handoff.
   - Robot drives backwards → increase `PreferForwardCritic.cost_weight`.
   - Robot spins in place → add/increase `TwirlingCritic`.
   - Robot stops with "all red" trajectories → `batch_size` too low, or critics too punishing.
4. **Adjust one critic at a time**, in increments of ~20%.
5. **Test in the hardest scenario** (tightest doorway, most cluttered room).
6. **Disable visualization** for deployment.
