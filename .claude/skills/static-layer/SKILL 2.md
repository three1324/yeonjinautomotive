<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: static-layer
description: 'Configure Nav2 static layer. Use when loading a map into the costmap, setting map_topic, or handling map updates.'
---

# StaticLayer Plugin

## Purpose

StaticLayer loads the pre-built map from `map_server` and writes it as the base layer of the global costmap. This provides the structural layout (walls, permanent furniture) that the planner uses for global pathfinding.

## Key Parameters

```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  enabled: true
  map_subscribe_transient_local: true
  map_topic: "/map"
  subscribe_to_updates: false
  trinary_costmap: true
```

### map_subscribe_transient_local

**Set to `true`** (critical). This sets the QoS subscription to transient-local durability, meaning the static layer will receive the last published map even if it subscribed after `map_server` published. Without this, if the costmap node starts before `map_server`, it may never receive the map.

### map_topic

Default: `"/map"`. Change only if your map_server publishes on a different topic (e.g., when running multiple maps).

### subscribe_to_updates

When `true`, the static layer also subscribes to `map_topic + "_updates"` (e.g., `/map_updates`). This is used during live SLAM when the map is being updated incrementally. For pre-built maps, leave `false`.

### trinary_costmap

Controls how map cell values translate to costmap costs:

**trinary_costmap: true** (default, recommended):
| Map value | Costmap value | Meaning |
|-----------|---------------|---------|
| 0 (free) | 0 | Free space |
| 100 (occupied) | 254 | Lethal obstacle |
| -1 (unknown) | 255 | Unknown |

All intermediate values in the OccupancyGrid are collapsed into these three states.

**trinary_costmap: false**:
The full 0-100 range from the map is scaled to costmap values 0-254. This preserves soft cost information from the map (e.g., areas that are "probably free" but uncertain). Useful with probabilistic maps from SLAM but less common in practice.

## Map Value to Costmap Cost Mapping

The `lethal_cost_threshold` parameter (default: 100) sets the minimum OccupancyGrid value that becomes lethal (254) in the costmap. Values at or above this threshold are lethal.

The `unknown_cost_value` parameter (default: -1) specifies which OccupancyGrid value maps to unknown (255). Set to a non-negative value if your map uses a different convention.

## When Static Layer Is NOT Needed

- **Local costmap with rolling_window: true**: The local costmap builds from live sensor data. Adding a static layer is wasteful — it loads the full map into the rolling window, and the data becomes stale relative to the robot's current surroundings.
- **Pure SLAM exploration** (no pre-built map): Use obstacle/voxel layers instead. There IS no static map yet.

## Integration with SLAM

When running SLAM (e.g., slam_toolbox), the static layer can subscribe to the live map:

```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: true
  subscribe_to_updates: true  # Receive incremental map updates
```

The map_server must be configured to republish SLAM updates. With slam_toolbox, the map is published on `/map` and updates on `/map_updates` automatically.

## Complete Global Costmap with Static Layer

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.18
      resolution: 0.05
      track_unknown_space: true
      always_send_full_costmap: false

      plugins: ["static_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        enabled: true
        map_subscribe_transient_local: true
        map_topic: "/map"
        subscribe_to_updates: false
        trinary_costmap: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

This minimal configuration is sufficient for a robot with good lidar coverage in the local costmap. The global costmap only needs walls from the static map plus inflation for path planning.

## Debugging

- **Costmap appears empty**: Check `ros2 topic echo /map --once` — is the map being published? Check `map_subscribe_transient_local: true`.
- **Unknown areas block planning**: Verify `track_unknown_space` in the parent costmap config. Set to `false` to treat unknown as free.
- **Map doesn't update during SLAM**: Enable `subscribe_to_updates: true` and verify `/map_updates` topic is active.
- **Wrong resolution**: The static layer adopts the resolution of the OccupancyGrid message, NOT the costmap's `resolution` parameter. If they differ, the static layer resamples, which can blur walls.

## PGM to Costmap Value Chain

```
PGM pixel (0-255) → map_server → OccupancyGrid (0-100, -1) → StaticLayer → Costmap cell (0, 254, or 255)
```

In the PGM: white (255) = free, black (0) = occupied, gray (205) = unknown (configurable via map YAML `free_thresh` and `occupied_thresh`).
