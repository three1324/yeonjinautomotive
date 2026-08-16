<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: cartographer-tuning
description: 'Tune Google Cartographer for ROS 2. Use when configuring 2D SLAM, setting submaps, scan matching, motion filter, or loop closure parameters.'
---

# Google Cartographer Tuning for ROS 2

## Overview

Google Cartographer performs 2D/3D SLAM using a graph-based approach with two distinct subsystems: **local SLAM** (trajectory builder) builds submaps from consecutive scans, and **global SLAM** (pose graph) connects submaps via loop closures and optimizes the full graph. Configuration is done via a `.lua` file passed as a launch parameter.

## Local SLAM: Trajectory Builder 2D

Local SLAM inserts laser scans into the current submap using scan matching. Each submap accumulates a fixed number of scans, then is frozen and handed to the pose graph.

### Key Parameters

```lua
TRAJECTORY_BUILDER_2D.min_range = 0.12          -- Ignore returns closer than this (m). Set to your lidar's minimum reliable range.
TRAJECTORY_BUILDER_2D.max_range = 8.0            -- Ignore returns beyond this (m). For indoor, cap to room diagonals.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- Length assigned to no-return rays. Keep <= max_range.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- Scans to accumulate before matching. 1 for single-beam lidar.

TRAJECTORY_BUILDER_2D.use_imu_data = true        -- Use IMU for gravity and rotation. true if IMU is calibrated.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Brute-force initial guess before Ceres optimization.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90     -- Scans per submap. More = denser submap, slower creation.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- Submap cell size (m). 0.05 = 5cm.
```

### Real-Time Correlative Scan Matcher

This brute-force matcher provides a robust initial pose estimate before the Ceres-based refinement. Critical for environments with long featureless corridors.

```lua
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1   -- meters
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)  -- radians
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
```

### Ceres Scan Matcher (local refinement)

```lua
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.   -- Higher = trust odometry more.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.      -- Higher = trust gyro/odom heading more.
```

## Global SLAM: Pose Graph

The pose graph runs in a background thread, adding inter-submap constraints and running optimization.

```lua
POSE_GRAPH.optimize_every_n_nodes = 90            -- Trigger global optimization every N nodes. 0 = disable.
POSE_GRAPH.constraint_builder.min_score = 0.65    -- Minimum scan match score to accept a loop closure. Range 0-1.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- Fraction of finished nodes to try matching against each submap.
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- Stricter threshold for non-consecutive submaps.

POSE_GRAPH.optimization_problem.huber_scale = 1e1  -- Robust loss to downweight outlier constraints.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
```

## Submap Creation Flow

1. Scans are inserted into the active submap via scan matching.
2. After `num_range_data` scans, the submap is **finished** (frozen).
3. A new empty submap begins. Two submaps briefly overlap.
4. Finished submaps become nodes in the pose graph.
5. The constraint builder tries to match recent scans against older submaps to find loop closures.

## Indoor Tuning Tips

| Issue | Fix |
|-------|-----|
| Lidar sees through windows | Reduce `max_range` to 6-8m |
| False loop closures (similar rooms) | Increase `min_score` to 0.7+ |
| Map drift with bad IMU | Set `use_imu_data = false`, increase `rotation_weight` |
| Map tearing at loop closure | Increase `huber_scale`, decrease `sampling_ratio` |
| Slow on Raspberry Pi | Increase `optimize_every_n_nodes`, reduce `submaps.num_range_data` |

## Complete .lua for Indoor Differential Drive

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",          -- Or "base_link" if no IMU.
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,           -- false when your robot publishes odom.
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimization_problem.huber_scale = 1e1

return options
```

## Launch Integration (ROS 2)

```python
Node(
    package='cartographer_ros',
    executable='cartographer_node',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=[
        '-configuration_directory', FindPackageShare('my_pkg').find('my_pkg') + '/config',
        '-configuration_basename', 'my_cartographer.lua',
    ],
    remappings=[('scan', '/scan')],
),
Node(
    package='cartographer_ros',
    executable='cartographer_occupancy_grid_node',
    parameters=[{'use_sim_time': use_sim_time, 'resolution': 0.05}],
),
```

## Diagnostics

Monitor `/constraint_list` in RViz to visualize inter-submap and intra-submap constraints. Green = intra-submap (sequential), blue = inter-submap (loop closures). Red/absent indicates rejected matches. Watch for excessive blue lines connecting distant submaps—this usually means false loop closures and requires raising `min_score`.
