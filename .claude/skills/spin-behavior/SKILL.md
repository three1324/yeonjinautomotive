<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: spin-behavior
description: 'Configure Nav2 spin recovery behavior. Use when tuning spin_dist, max_rotational_vel, or in-place rotation for stuck recovery.'
---

# Spin Recovery Behavior

## Overview

The Spin behavior rotates the robot in place by a specified angle. It is the most common first-line recovery action when the robot gets stuck due to costmap artifacts or minor planning failures. Plugin: `nav2_behaviors::Spin`.

## BT Node

```xml
<Spin spin_dist="1.57" server_name="behavior_server" server_timeout="10"/>
```

`spin_dist` is in **radians**. Positive = counterclockwise. `1.57` ≈ 90°, `3.14` ≈ 180°, `6.28` ≈ full rotation.

## How It Works

1. The behavior server receives the goal angle from the BT action node.
2. Each cycle, it commands an angular velocity (`cmd_vel.angular.z`) while publishing zero linear velocity.
3. Before commanding rotation, it performs a **simulated forward projection** using `simulate_ahead_time` — it checks whether the robot footprint at the projected future pose would collide with the local costmap.
4. The robot accelerates up to `max_rotational_vel` respecting `rotational_acc_lim`, then decelerates as it approaches the target angle.
5. Returns `SUCCESS` when the accumulated rotation reaches `spin_dist`, or `FAILURE` if collision is predicted and rotation cannot proceed.

## YAML Configuration

```yaml
behavior_server:
  ros__parameters:
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    # Spin-specific parameters
    simulate_ahead_time: 1.0        # seconds of forward simulation for collision check
    max_rotational_vel: 1.0          # rad/s — maximum spin speed
    min_rotational_vel: 0.4          # rad/s — minimum to overcome friction
    rotational_acc_lim: 3.2          # rad/s² — angular acceleration limit
    transform_tolerance: 0.1
```

## simulate_ahead_time Interaction

This parameter is the most frequent source of "spin won't start" problems. The collision checker projects the robot footprint forward by `simulate_ahead_time × current_velocity` and checks for costmap lethal cells. If the inflation radius in the local costmap extends close to the robot footprint, the simulated pose may overlap inflated cells even though the robot is only rotating in place.

**Symptoms**: Spin immediately returns FAILURE without any rotation. Logs show `"Collision Ahead"`.

**Fixes**:
- Reduce `simulate_ahead_time` from the default `2.0` to `0.5`–`1.0`.
- Reduce the local costmap `inflation_layer` `inflation_radius` (e.g., from 0.55 to 0.35).
- Verify the robot footprint is accurate — an oversized footprint causes phantom collisions.

## Common Issues

| Problem | Cause | Fix |
|---|---|---|
| Spin never starts | `simulate_ahead_time` too high or inflation too wide | Lower `simulate_ahead_time` to 0.5–1.0 |
| Spin oscillates without progressing | `min_rotational_vel` too low, static friction wins | Increase `min_rotational_vel` to 0.4+ |
| Spin overshoots target angle | `rotational_acc_lim` too low for deceleration | Increase `rotational_acc_lim` or reduce `max_rotational_vel` |
| Spin completes but robot still stuck | Costmap artifacts persist after rotation | Chain with a `ClearEntireCostmap` before retrying navigation |

## Typical BT Recovery Usage

```xml
<RecoveryNode number_of_retries="3" name="NavigateRecovery">
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
      <ComputePathToPose goal="{goal}" path="{path}"/>
    </RateController>
    <FollowPath path="{path}"/>
  </PipelineSequence>
  <RoundRobin name="RecoveryActions">
    <Spin spin_dist="1.57"/>
    <Wait wait_duration="3"/>
    <BackUp backup_dist="0.15" backup_speed="0.05"/>
    <Spin spin_dist="3.14"/>
  </RoundRobin>
</RecoveryNode>
```

## Debugging

```bash
ros2 topic echo /cmd_vel          # verify angular.z commands during spin
ros2 action list                   # should show /spin action
ros2 param get /behavior_server simulate_ahead_time
```

Set `behavior_server` log level to DEBUG to see collision check details:
```bash
ros2 service call /behavior_server/set_logger_level rcl_interfaces/srv/SetLoggerLevel \
  "{logger_name: 'behavior_server', level: 'DEBUG'}"
```
