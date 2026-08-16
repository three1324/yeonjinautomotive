<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: denoise-layer
description: 'Configure Nav2 denoise costmap layer. Use when filtering spurious single-cell obstacles or noise in the costmap.'
---

# DenoiseLayer

## Purpose

DenoiseLayer removes salt-and-pepper noise from the costmap — isolated single-pixel or small-cluster lethal marks that aren't real obstacles. These appear from sensor noise, multipath reflections, or transient electrical interference, and cause unnecessary path deviations and replanning.

## Plugin Configuration

```yaml
denoise_layer:
  plugin: "nav2_costmap_2d::DenoiseLayer"
  enabled: true
  minimal_group_size: 2
```

## Parameters

### minimal_group_size

The minimum number of connected lethal cells that must form a cluster to survive denoising. Groups smaller than this threshold are removed (set to free).

| Value | Effect |
|---|---|
| 1 | No denoising (every cell survives) |
| 2 | Removes isolated single-pixel obstacles (most common setting) |
| 3 | Removes single pixels AND pairs |
| 4+ | Aggressive — may remove real small obstacles |

**Recommended**: `2` for most robots. This catches single-pixel noise without masking real obstacles.

### enabled

`true`/`false` to enable/disable without removing from plugins list.

## Algorithm

The denoising algorithm performs connected-component analysis on lethal cells:

1. Scan the costmap for all cells with cost = 254 (lethal)
2. Group connected lethal cells using 4-connectivity (up, down, left, right neighbors)
3. For each connected group with fewer than `minimal_group_size` cells, set those cells to free (0)
4. Larger groups are left untouched

This is done during the layer's `updateCosts()` phase, before inflation runs.

## Plugin Order

DenoiseLayer MUST be placed:
- **AFTER** obstacle/voxel/range layers (it needs to see their marks)
- **BEFORE** inflation_layer (so inflation only inflates real obstacles)

```yaml
plugins: ["voxel_layer", "obstacle_layer", "denoise_layer", "inflation_layer"]
```

If denoise is placed after inflation, it would try to remove lethal cells that are already surrounded by inflation gradient — and the inflation wouldn't be recalculated.

## When to Use

**Use DenoiseLayer when:**
- You see scattered single-pixel obstacles in RViz that cause the robot to swerve
- Your lidar has occasional noise spikes (common with cheap lidars)
- Multipath reflections create phantom dots (glass, chrome, mirrors)
- The robot replans excessively due to transient noise marks

**Don't use DenoiseLayer when:**
- Your sensors are clean and you don't see noise in the costmap
- You need to detect small real obstacles (cables, thin posts)
- You've already filtered noise at the sensor driver level

## Complete Example

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "denoise_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        combination_method: 1
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true
          obstacle_range: 3.5
          raytrace_range: 5.0

      denoise_layer:
        plugin: "nav2_costmap_2d::DenoiseLayer"
        enabled: true
        minimal_group_size: 2

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Risks

- **minimal_group_size too high (≥ 4)**: Real thin obstacles like chair legs or cables may be denoised away. The robot could collide with objects it no longer sees.
- **Masking sensor issues**: If noise is caused by a broken sensor, denoising hides the problem instead of fixing it. Always investigate persistent noise at the source.
- **CPU cost**: Connected-component analysis is O(n) in the number of lethal cells. For typical costmaps, this is negligible. For very large costmaps with many obstacles, there's a measurable but small CPU impact.

## Debugging

- **Still seeing noise**: Increase `minimal_group_size` cautiously (try 3). But first check sensor data quality.
- **Real obstacles disappearing**: Decrease `minimal_group_size` or disable the layer. Visualize the raw costmap (before inflation) to see which marks are being removed.
- **To see the effect**: Compare RViz costmap visualization with denoise_layer enabled vs disabled. Toggle with dynamic reconfigure or set `enabled: false` temporarily.

## Alternative: Sensor-Level Filtering

Before adding DenoiseLayer, consider filtering noise at the source:
- **Lidar**: Use `laser_filters` package to remove outliers before publishing `/scan`
- **Depth camera**: Apply temporal/spatial filters in the camera driver
- **Range sensors**: Average multiple readings in the sensor driver

Sensor-level filtering is preferred when possible because it reduces noise before it enters the costmap pipeline. DenoiseLayer is the fallback when sensor-level filtering is insufficient or impractical.
