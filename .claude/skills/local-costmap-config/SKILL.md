<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: local-costmap-config
description: 'Configure Nav2 local costmap. Use when setting rolling window size, resolution, update frequency, or sensor sources for local planning.'
---

# Local Costmap Configuration

## Purpose

The local costmap is a rolling window around the robot used by the controller (DWB, RPP, MPPI) for reactive obstacle avoidance. It does NOT use the static map — it builds entirely from live sensor data.

## Key Rules

- **rolling_window**: Always `true`. The costmap moves with the robot.
- **global_frame**: Must be `"odom"` — NOT `"map"`. Using `"map"` causes TF jitter as localization corrections shift the frame. The local costmap needs the smooth, continuous odom frame for stable controller behavior.
- **No static_layer**: A rolling window doesn't load from the static map. Sensor layers populate it directly.
- **width/height**: Define the window size in meters. These ARE used (unlike global costmap with static map).

## Sizing the Local Costmap

The costmap size directly affects CPU load:

```
cells = (width / resolution) * (height / resolution)
```

| Width×Height | Resolution | Cells | Notes |
|---|---|---|---|
| 3m × 3m | 0.05 | 3,600 | Tight, fast. Good for slow indoor robots |
| 5m × 5m | 0.05 | 10,000 | Good balance for indoor differential drive |
| 10m × 10m | 0.05 | 40,000 | Large, expensive. Only for fast outdoor robots |
| 6m × 6m | 0.025 | 57,600 | High-res. Expensive but precise for narrow corridors |

**Rule of thumb**: The local costmap should extend at least 2× the robot's stopping distance at max velocity. For a differential drive at 0.3 m/s, 3-5m is sufficient.

**Common mistake**: Making the local costmap too large. A 20m × 20m local costmap wastes CPU on cells the controller will never evaluate. The controller only plans ~1-3 seconds ahead.

## Complete YAML Example

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      robot_radius: 0.18
      transform_tolerance: 0.3

      plugins: ["voxel_layer", "obstacle_layer", "range_sensor_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        combination_method: 1
        publish_voxel_map: false
        z_voxels: 10
        z_resolution: 0.05
        origin_z: 0.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true
          obstacle_range: 3.0
          raytrace_range: 4.0
          max_obstacle_height: 0.5
          min_obstacle_height: 0.0

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        combination_method: 1
        observation_sources: bump
        bump:
          topic: /bump_scan
          data_type: "LaserScan"
          marking: true
          clearing: false
          obstacle_range: 0.5
          raytrace_range: 0.6

      range_sensor_layer:
        plugin: "nav2_costmap_2d::RangeSensorLayer"
        enabled: true
        topics: ["/range/front_left", "/range/front_right", "/range/rear"]
        phi: 0.087
        inflate_cone: 1.0
        no_readings_timeout: 1.0
        clear_threshold: 0.2
        mark_threshold: 0.8
        clear_on_max_reading: true
        combination_method: 1

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Update Frequency Guidance

- **update_frequency: 10.0** — Process sensor data 10 times/second. Increase to 15 for fast-moving robots or dynamic environments. Decrease to 5 if CPU-constrained.
- **publish_frequency: 5.0** — For RViz visualization only. In production, set to 1.0 or 0.0 to save CPU.

The local costmap update is the most CPU-intensive part of the navigation stack on most robots. Profile with `ros2 topic hz /local_costmap/costmap` to verify actual rates match configured rates.

## Multiple Sensor Layers

The example above uses three layers for different sensor modalities:
- **voxel_layer**: Primary lidar with 3D clearing
- **obstacle_layer**: Secondary sensor (bump sensor) with marking only
- **range_sensor_layer**: Proximity sensors for close-range detection

Each layer's `combination_method: 1` (Maximum) ensures marks from one layer aren't cleared by another layer's raytracing. This is critical when sensors have different fields of view.

## Common Mistakes

1. **global_frame: "map"** — Causes the local costmap to jump when AMCL corrects pose. Controller becomes unstable.
2. **rolling_window: false** — Without this, the local costmap is fixed in space and doesn't follow the robot.
3. **Including static_layer** — Adds the full map into the rolling window. Wastes CPU and can cause stale obstacle data.
4. **Costmap too large with high resolution** — A 20m×20m at 0.025m resolution = 640,000 cells. The update loop will lag.
5. **Low update_frequency** — Below 5 Hz, the robot reacts slowly to new obstacles. Dangerous for dynamic environments.
