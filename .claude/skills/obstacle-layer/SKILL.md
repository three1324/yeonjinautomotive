<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: obstacle-layer
description: 'Configure Nav2 obstacle layer. Use when adding laser or pointcloud obstacles, setting raytrace/obstacle ranges, or clearing parameters.'
---

# ObstacleLayer (2D)

## Purpose

ObstacleLayer marks cells as lethal (254) based on live sensor data and clears cells via raycasting. This is the primary layer for detecting dynamic obstacles (people, furniture, doors) in 2D.

## Observation Sources

The `observation_sources` parameter lists named sources. Each source subscribes to a ROS topic and has independent marking/clearing/range settings. The source list is **NOT order-dependent** — all sources are processed in the same update cycle.

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: true
  combination_method: 1
  footprint_clearing_enabled: true
  max_obstacle_height: 2.0
  observation_sources: scan bump_scan
```

## Per-Source Configuration

```yaml
scan:
  topic: /scan
  data_type: "LaserScan"          # "LaserScan" or "PointCloud2"
  marking: true                    # Add lethal marks from this source
  clearing: true                   # Raytrace to clear cells from this source
  obstacle_range: 3.0              # Max range (m) at which to mark obstacles
  raytrace_range: 4.0              # Max range (m) at which to raytrace-clear
  max_obstacle_height: 2.0         # Ignore points above this height
  min_obstacle_height: 0.0         # Ignore points below this height
  expected_update_rate: 0.0        # 0 = no staleness check. Set > 0 to warn on stale data
  observation_persistence: 0.0     # Seconds to keep observations in buffer (0 = current only)
  inf_is_valid: false              # Treat inf range readings as max range for clearing
  sensor_frame: ""                 # Override frame_id from message (leave empty to use message frame)
```

## CRITICAL: raytrace_range Must Be >= obstacle_range

```
raytrace_range: 4.0   # Clears cells up to 4m via raytracing
obstacle_range: 3.0   # Marks obstacles up to 3m
```

If `raytrace_range < obstacle_range`, obstacles marked at the outer range are NEVER cleared by raycasting when the robot moves away. They persist as phantom obstacles, eventually corrupting the costmap.

**Rule**: Always set `raytrace_range` at least 1m greater than `obstacle_range`.

## Marking and Clearing

- **Marking**: When a sensor reports a hit at distance D ≤ obstacle_range, the cell at that position is set to lethal (254).
- **Clearing (raycasting)**: A ray is traced from the sensor origin through each endpoint. All cells along the ray up to raytrace_range are set to free (0). This clears previously marked cells when an obstacle moves away.

A source can be marking-only (`marking: true, clearing: false`) for sensors that detect obstacles but shouldn't clear them (e.g., bumper sensors). Or clearing-only (`marking: false, clearing: true`) for a sensor used solely to clear stale marks.

## footprint_clearing_enabled

When `true`, cells under the robot's footprint are cleared to free on each update. This prevents the robot from marking itself as an obstacle if a sensor partially sees the robot body. Recommended: `true`.

## Multi-Sensor Example

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: true
  combination_method: 1
  footprint_clearing_enabled: true
  max_obstacle_height: 2.0
  observation_sources: front_scan rear_scan depth_camera

  front_scan:
    topic: /scan_front
    data_type: "LaserScan"
    marking: true
    clearing: true
    obstacle_range: 5.0
    raytrace_range: 6.5
    max_obstacle_height: 2.0
    min_obstacle_height: 0.0
    expected_update_rate: 5.0

  rear_scan:
    topic: /scan_rear
    data_type: "LaserScan"
    marking: true
    clearing: true
    obstacle_range: 3.0
    raytrace_range: 4.0
    max_obstacle_height: 2.0
    min_obstacle_height: 0.0

  depth_camera:
    topic: /camera/depth/points
    data_type: "PointCloud2"
    marking: true
    clearing: false           # Don't clear with camera - lidar handles clearing
    obstacle_range: 4.0
    raytrace_range: 5.0
    max_obstacle_height: 1.5
    min_obstacle_height: 0.05  # Ignore ground plane noise
```

## combination_method

Set to `1` (Maximum) so that obstacle marks from this layer don't overwrite higher costs from other layers. If using Overwrite (0), this layer will erase inflation costs from previous layers in the plugin order.

## Height Filtering

- **max_obstacle_height**: Points above this are ignored. Set to ~2.0m to filter ceiling reflections.
- **min_obstacle_height**: Points below this are ignored. Set to ~0.05m to filter ground plane noise from depth cameras. For 2D lidar mounted at a fixed height, ground filtering is usually not needed.

## expected_update_rate

When set to a positive value (Hz), the layer checks if new data arrived within that period. If not, it logs a warning:

```
Sensor data is stale for source "scan", expected update rate is 5.0 Hz
```

This catches disconnected sensors or topic remapping errors. Set to `0.0` to disable.

## Debugging

- **Obstacles don't clear**: Check `raytrace_range >= obstacle_range`. Check `clearing: true` on at least one source.
- **Phantom obstacles after turning**: The sensor's FOV sweep may not cover all previously marked cells. Increase raytrace_range or add a clearing-only source with wide FOV.
- **No obstacles appear**: Check that the sensor topic is publishing in the correct frame. Run `ros2 topic echo /scan --once` and verify `header.frame_id` is in the TF tree.
- **Stale data warning**: Verify sensor driver is running. Check `expected_update_rate` matches actual sensor rate.
