<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: global-costmap-config
description: 'Configure Nav2 global costmap. Use when setting global costmap size, resolution, update frequency, or static map integration.'
---

# Global Costmap Configuration

## Purpose

The global costmap covers the entire known map. Planners (NavFn, Smac, Theta*) use it to compute full paths from start to goal. It is built from the static map plus any dynamic obstacle layers and inflation.

## Key Settings

- **global_frame**: Must be `"map"`. The global costmap operates in the map frame.
- **robot_base_frame**: `"base_link"` (or your robot's base frame).
- **resolution**: Typically `0.05` (5cm). Must match or be coarser than the static map resolution.
- **Width/height**: NOT set when a static map is loaded — the costmap inherits dimensions from the map server.
- **track_unknown_space**: `true` means cells marked unknown (205 in PGM) in the static map remain cost 255 (unknown) in the costmap. The planner will not route through them. Set `false` if you want unknown = free.
- **always_send_full_costmap**: `true` publishes the entire costmap each cycle (useful for debugging). `false` sends only updates (saves bandwidth). Use `false` in production.

## Robot Footprint

Two options — use ONE, not both:

```yaml
# Option 1: Circular robot (simpler, faster collision checking)
robot_radius: 0.18

# Option 2: Polygon footprint (for non-circular robots)
# Points are [x, y] in base_link frame, counterclockwise
footprint: "[[0.20, 0.15], [-0.20, 0.15], [-0.20, -0.15], [0.20, -0.15]]"
```

The inscribed radius (largest circle fitting inside the footprint) and circumscribed radius (smallest circle enclosing the footprint) are computed automatically. These affect inflation layer behavior.

## When to Include Obstacle/Voxel Layers in Global Costmap

- **Short-range sensors only (< 3m)**: Skip obstacle layers in global costmap. The static map provides sufficient context for global planning. Dynamic obstacles are handled by the local costmap.
- **Long-range sensors (lidar > 5m)**: Include a voxel or obstacle layer to catch dynamic obstacles not on the static map (people, furniture moved since mapping).
- **SLAM mode** (no static map): You MUST have obstacle layers in the global costmap or it will be empty.

## Complete YAML Example

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.18
      resolution: 0.05
      track_unknown_space: true
      always_send_full_costmap: false
      transform_tolerance: 0.5

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        enabled: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        combination_method: 1
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true
          obstacle_range: 6.0
          raytrace_range: 8.0
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Common Mistakes

1. **Setting width/height with a static map**: The costmap ignores them and uses the map dimensions. Explicitly setting them causes confusion.
2. **global_frame: "odom"**: Wrong for global costmap. Must be `"map"` so the costmap aligns with the static map and goals.
3. **High update_frequency**: The global costmap doesn't need fast updates (1-2 Hz is fine). The local costmap handles reactive obstacle avoidance.
4. **Missing track_unknown_space**: Defaults to `false`, treating unknown as free. This can cause the planner to route through unmapped areas.
5. **robot_radius AND footprint both set**: `footprint` takes precedence. Remove `robot_radius` if using polygon footprint to avoid confusion.

## Debugging Tips

- Visualize in RViz: add a `Map` display on `/global_costmap/costmap` topic.
- Check layer activation: `ros2 service call /global_costmap/global_costmap/get_parameters rcl_interfaces/srv/GetParameters "{names: ['plugins']}"`.
- If the costmap appears empty, verify the static map is being published: `ros2 topic echo /map --once`.
- If obstacles don't appear, check TF: the sensor frame must be connected to `map` via TF chain.
