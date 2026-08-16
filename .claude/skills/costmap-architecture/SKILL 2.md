<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: costmap-architecture
description: 'Understand Nav2 costmap architecture and layer ordering. Use when configuring costmap plugins list (ORDER DEPENDENT), observation_sources, or master costmap.'
---

# Costmap Architecture & Layer Processing Order

## CRITICAL: Plugin Order Is Strictly Sequential

The `plugins` list in costmap YAML is **order-dependent**. Each layer writes into the master costmap grid sequentially. The combination_method on each layer controls HOW it writes.

```yaml
# CORRECT order - inflation LAST
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "obstacle_layer", "range_sensor_layer", "inflation_layer"]
```

```yaml
# WRONG - inflation before obstacle means dynamic obstacles are NOT inflated
plugins: ["inflation_layer", "voxel_layer", "obstacle_layer"]  # BUG!
```

**RULE**: `inflation_layer` MUST be the LAST plugin. It reads all lethal/inscribed cells and generates the cost gradient. Any layer added after inflation will overwrite the gradient.

**Global costmap order**: `static_layer` → obstacle/voxel layers → `inflation_layer`
**Local costmap order**: obstacle/voxel layers → `range_sensor_layer` → `inflation_layer`

The `observation_sources` list within a single layer is NOT order-dependent. All sources are processed during the same layer update cycle.

## Master Grid Combination Methods

Each layer plugin has a `combination_method` parameter controlling how it merges into the master grid:

| Value | Name | Behavior |
|-------|------|----------|
| 0 | Overwrite | Layer output replaces master grid cells entirely |
| 1 | Maximum | Each cell = max(master, layer). Preserves higher costs from earlier layers |
| 2 | MaxWithoutUnknownOverwrite | Like Maximum, but unknown (255) cells in the layer don't overwrite known values |

**When to use each:**
- **Overwrite (0)**: Static layer in global costmap (it provides the base map).
- **Maximum (1)**: Most dynamic layers (obstacle, voxel, range). Ensures lethal marks from other layers aren't cleared.
- **MaxWithoutUnknownOverwrite (2)**: When a layer has large unknown regions and you don't want them clobbering known-free space from other layers. Useful for voxel_layer when sensor coverage is limited.

## Costmap2DROS Wrapper

`Costmap2DROS` is the ROS 2 lifecycle node that owns the costmap. It:

1. Creates the `LayeredCostmap` (the master grid + ordered layer stack)
2. Manages the update loop on a timer at `update_frequency`
3. Publishes the costmap on a separate timer at `publish_frequency`
4. Handles TF lookups for robot pose in the costmap frame
5. Exposes the costmap to planners/controllers via a shared pointer

The update cycle per tick:
1. Look up robot pose via TF (`robot_base_frame` → `global_frame`)
2. Compute the update bounds (region around robot that needs refresh)
3. Call `updateBounds()` on each layer in order — layers report which cells they'll modify
4. Call `updateCosts()` on each layer in order — layers write into the master grid
5. If `publish_frequency` timer fires, serialize and publish as `nav2_msgs/Costmap`

## Key Timing Parameters

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0    # Hz - how often layers process sensor data and update the grid
      publish_frequency: 5.0    # Hz - how often the costmap is published for visualization
      transform_tolerance: 0.3  # seconds - max age of TF data before considered stale
```

- **update_frequency**: Controls responsiveness to new sensor data. Higher = more CPU. The costmap is always current internally for the planner/controller regardless of publish rate.
- **publish_frequency**: Only affects RViz visualization and external consumers. Set lower than update_frequency to save bandwidth. Set to 0.0 to disable publishing entirely in production.
- **transform_tolerance**: If the TF lookup for robot pose is older than this, the costmap update is skipped. Too tight (< 0.1s) causes skipped updates on loaded systems. Too loose (> 1.0s) uses stale pose data.

## Sensor Data Flow

```
Sensor → ROS topic → observation_sources in layer → layer updateBounds/updateCosts → master grid
```

Each observation source buffers incoming messages. During `updateBounds()`, the layer reads buffered observations and determines which cells to mark/clear. During `updateCosts()`, those marks are written to the master grid using the layer's `combination_method`.

Stale data detection: if a source's `expected_update_rate` is set and no message arrives within that period, a warning is logged and the source is considered stale. The layer continues operating with remaining sources.

## Costmap Cell Values

| Value | Meaning |
|-------|---------|
| 0 | Free space |
| 1-252 | Increasing cost (from inflation gradient) |
| 253 | Inscribed cost — robot center here means collision |
| 254 | Lethal — obstacle cell |
| 255 | Unknown / no information |

Planners treat 254 as impassable. Most planners also treat 253 as impassable (inscribed radius collision). The inflation gradient (1-252) biases paths away from obstacles without blocking them.
