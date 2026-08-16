<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: backup-behavior
description: 'Configure Nav2 backup recovery behavior. Use when tuning backup_dist, backup_speed, or reverse motion for stuck recovery.'
---

# BackUp Recovery Behavior

## Overview

The BackUp behavior drives the robot backward by a specified distance at a controlled speed. It is used when the robot is stuck and spinning alone cannot free it — typically when the robot has driven into a tight space and needs to retreat. Plugin: `nav2_behaviors::BackUp`.

## BT Node

```xml
<BackUp backup_dist="0.30" backup_speed="0.05" server_name="behavior_server" server_timeout="10"/>
```

- `backup_dist`: distance in **meters** (positive value, despite moving backwards). Default: 0.15.
- `backup_speed`: linear speed in **m/s**. Keep slow (0.05–0.15) for safety.

## How It Works

1. On receiving the goal, the behavior records the robot's starting pose via TF.
2. Each cycle, it publishes a negative linear velocity (`cmd_vel.linear.x = -backup_speed`) with zero angular.
3. Before each command, it checks the **costmap behind the robot** by projecting the footprint along the reverse trajectory using `simulate_ahead_time`.
4. The behavior monitors accumulated distance by comparing current pose to start pose.
5. Returns `SUCCESS` when `backup_dist` is reached, `FAILURE` if rearward collision is detected.

**Duration** ≈ `backup_dist / backup_speed`. At 0.30 m / 0.05 m/s = 6 seconds.

## YAML Configuration

```yaml
behavior_server:
  ros__parameters:
    cycle_frequency: 10.0
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    behavior_plugins: ["spin", "backup", "wait"]
    backup:
      plugin: "nav2_behaviors::BackUp"
    simulate_ahead_time: 1.0
    transform_tolerance: 0.1
```

`backup_dist` and `backup_speed` are set in the BT node, not the YAML. The behavior server YAML controls the collision checking and TF parameters.

## Why BackUp Fails — Rear Sensor Self-Occlusion

The most common failure mode: the robot's own rear-facing sensors (LIDAR, depth cameras) mark the area immediately behind the robot as occupied in the local costmap. When the behavior checks for rearward clearance, it finds lethal cells and refuses to move.

**Root cause**: sensor `min_range` is set too low, or the sensor publishes returns from the robot's own body or mounting hardware.

**Fixes**:

1. **Set appropriate `min_range`** on obstacle layers for rear sensors:
   ```yaml
   local_costmap:
     local_costmap:
       ros__parameters:
         obstacle_layer:
           observation_sources: scan rear_scan
           rear_scan:
             topic: /rear_scan
             min_obstacle_height: 0.05
             max_obstacle_height: 0.5
             clearing: true
             marking: true
             min_range: 0.20   # ignore returns closer than 20 cm
   ```

2. **Enable `footprint_clearing_enabled`** on the obstacle layer so the robot's own footprint cells are always cleared:
   ```yaml
   obstacle_layer:
     footprint_clearing_enabled: true
   ```

3. **Adjust the robot footprint** to precisely match the physical chassis so that self-returns fall inside the cleared footprint zone.

## transform_tolerance

The `transform_tolerance` parameter on `behavior_server` controls how stale a TF lookup can be before the behavior aborts. If your TF tree has jitter or occasional delays (common with hardware drivers), increase from the default `0.1` to `0.2`–`0.5`.

Symptom of too-tight tolerance: BackUp returns FAILURE immediately with TF lookup errors in the log.

## Common Issues

| Problem | Cause | Fix |
|---|---|---|
| BackUp immediately fails | Rear area marked occupied by own sensors | Increase sensor `min_range`, enable `footprint_clearing_enabled` |
| BackUp fails with TF error | `transform_tolerance` too tight | Increase to 0.2–0.5 |
| Robot backs into wall | `simulate_ahead_time` too low | Increase to 1.5–2.0 |
| BackUp too slow / times out in BT | `backup_speed` too conservative | Increase to 0.10–0.15 m/s |
| Robot veers while backing up | IMU drift or wheel slip | Reduce `backup_dist` to small increments (0.10–0.15 m) |

## BT Recovery Pattern

A common escalating recovery sequence:

```xml
<RoundRobin name="RecoveryActions">
  <Sequence name="WaitThenRetry">
    <Wait wait_duration="3"/>
  </Sequence>
  <Sequence name="SpinThenRetry">
    <Spin spin_dist="1.57"/>
  </Sequence>
  <Sequence name="BackUpThenSpin">
    <BackUp backup_dist="0.20" backup_speed="0.08"/>
    <Spin spin_dist="1.57"/>
  </Sequence>
  <Sequence name="AggressiveRecovery">
    <ClearEntireCostmap name="ClearLocal" service_name="local_costmap/clear_entirely_local_costmap"/>
    <BackUp backup_dist="0.30" backup_speed="0.10"/>
  </Sequence>
</RoundRobin>
```

## Debugging

```bash
ros2 topic echo /cmd_vel            # verify negative linear.x during backup
ros2 action send_goal /backup nav2_msgs/action/BackUp \
  "{target: {x: -0.20, y: 0.0, z: 0.0}, speed: 0.05}"
```

Check local costmap behind the robot in RViz — look for unexpected lethal cells at the rear of the footprint.
