<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: voxel-layer
description: 'Configure Nav2 voxel layer for 3D obstacle detection. Use when using depth cameras or 3D lidar, setting z_voxels, mark/clear heights.'
---

# VoxelLayer (3D)

## Purpose

VoxelLayer extends ObstacleLayer with a 3D voxel grid for proper volumetric raycasting. Instead of projecting all points into 2D and raytracing in the plane, it maintains a 3D column of voxels at each (x, y) cell and raytraces in 3D space.

## When to Use VoxelLayer vs ObstacleLayer

**Use VoxelLayer when:**
- You have sensors at different heights (e.g., lidar at 15cm, depth camera at 80cm)
- You need 3D clearing — an object on a table (detected by camera) should be cleared when the table is removed, even though the lidar never saw it
- You want to handle overhanging obstacles (shelves, table edges) that lidar misses
- Your sensor is tilted and sees the ground at some ranges (3D raycasting avoids false ground marks)

**Use ObstacleLayer when:**
- You only have a single 2D lidar at a fixed height
- CPU is very constrained (VoxelLayer uses more memory and processing)
- The environment is simple and 2D projection is sufficient

## Z-Axis Parameters

```yaml
voxel_layer:
  plugin: "nav2_costmap_2d::VoxelLayer"
  z_voxels: 10           # Number of voxels in the Z column
  z_resolution: 0.05     # Height of each voxel in meters
  origin_z: 0.0          # Z origin of the voxel grid (meters above ground)
  mark_threshold: 0       # Min marked voxels in a column to project as occupied in 2D
```

The total monitored height = `z_voxels * z_resolution`. With the defaults above: 10 × 0.05 = 0.5m above `origin_z`.

### mark_threshold

Controls the 3D → 2D projection. A column of voxels at (x, y) is projected as occupied in the 2D costmap only if the number of marked voxels ≥ `mark_threshold`.

- **mark_threshold: 0** — Any single marked voxel in the column makes it occupied (most sensitive)
- **mark_threshold: 2** — Need at least 2 marked voxels (filters noise, but may miss thin obstacles)

For most robots, `mark_threshold: 0` is appropriate. Increase only if you have noise issues.

## 3D Raycasting Explained

When clearing, the VoxelLayer traces a ray in 3D from the sensor origin through each measured point. All voxels along the 3D ray are cleared. This matters because:

1. An obstacle detected by a high camera is stored in upper voxels
2. A lidar at the same (x, y) reads through at a lower height
3. The 3D raytrace from the lidar clears ONLY the lower voxels — the upper obstacle remains
4. If the obstacle is removed, the next camera frame's raytrace clears the upper voxels

This is fundamentally different from 2D ObstacleLayer, where the lidar clear would erase the camera's mark.

## Complete Configuration

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        combination_method: 1
        footprint_clearing_enabled: true
        max_obstacle_height: 0.5
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 10
        mark_threshold: 0
        publish_voxel_map: true     # Enable for debugging
        observation_sources: lidar depth

        lidar:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true
          obstacle_range: 3.5
          raytrace_range: 5.0
          max_obstacle_height: 0.5
          min_obstacle_height: 0.0
          expected_update_rate: 5.0

        depth:
          topic: /camera/depth/points
          data_type: "PointCloud2"
          marking: true
          clearing: true
          obstacle_range: 3.0
          raytrace_range: 4.0
          max_obstacle_height: 0.5
          min_obstacle_height: 0.05
          expected_update_rate: 5.0
```

## publish_voxel_map

When `true`, publishes the 3D voxel grid as a `PointCloud2` on `<costmap_name>/voxel_grid`. Visualize in RViz:

1. Add a PointCloud2 display
2. Set topic to `/local_costmap/voxel_grid`
3. Set Color Transformer to "FlatColor" or "AxisColor" (Z-axis)
4. Set size to ~0.05 to match z_resolution

This shows exactly which voxels are marked and helps debug sensor coverage, clearing issues, and height filtering. **Disable in production** — it's expensive to publish.

## Memory Usage

The voxel grid uses 2 bits per voxel (marked, cleared, or unknown). Memory per layer:

```
memory = (width/resolution) * (height/resolution) * z_voxels * 2 bits
```

For a 5m × 5m costmap at 0.05m resolution with 10 z-voxels:
```
100 * 100 * 10 * 2 bits = 200,000 bits ≈ 24 KB
```

This is negligible. Even large costmaps with many z-voxels use modest memory.

## Observation Source Settings

All observation source parameters from ObstacleLayer apply (see obstacle_layer.md). The same **raytrace_range ≥ obstacle_range** rule is critical.

Additional consideration for VoxelLayer: `max_obstacle_height` should match `origin_z + z_voxels * z_resolution`. Points above the voxel grid are silently dropped — they won't be marked OR cleared.

## Common Issues

- **Ground hits marking as obstacles**: Set `min_obstacle_height: 0.05` on depth camera sources. Or increase `origin_z` above the ground plane.
- **Obstacles not clearing after removal**: Check that `clearing: true` is set on a source that covers the obstacle. Check the voxel map in RViz to see if upper voxels are stuck.
- **Too many voxel layers**: Each VoxelLayer has its own independent voxel grid. Having multiple VoxelLayers is usually wrong — put multiple sources in ONE voxel_layer's `observation_sources`.
