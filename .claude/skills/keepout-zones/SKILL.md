<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: keepout-zones
description: 'Set up Nav2 keepout zones. Use when defining no-go areas, creating filter mask maps, or excluding regions from navigation.'
---

# Keepout Zones

## Purpose

Keepout zones prevent the robot from planning or navigating through designated areas. These areas become lethal (254) in the costmap, making them impassable. Use cases: staircases, fragile equipment areas, private rooms, hazardous zones.

## How It Works

1. A **mask image** (PGM) marks keepout areas
2. A **map_server** node serves this mask as an OccupancyGrid
3. A **CostmapFilterInfoServer** publishes metadata (type=0 for keepout)
4. A **KeepoutFilter** plugin in the costmap reads the mask and marks keepout cells as lethal

## Creating the Mask

### Step 1: Get the base map PGM

Use the same PGM file from your map YAML as a starting template. This ensures the mask is perfectly aligned.

### Step 2: Edit in an image editor

Open the PGM in GIMP or similar:
- **Black pixels** (value 0) → occupied (100 in OccupancyGrid) → keepout zone
- **White pixels** (value 254) → free (0 in OccupancyGrid) → no restriction
- Paint keepout areas solid black
- Leave everything else white (not the original map data — pure white)

**Important**: The mask only defines keepout regions. It does NOT need to replicate walls or obstacles from the original map. Paint it as: white everywhere, black only where keepout is desired.

### Step 3: Save as PGM and create YAML

```yaml
# keepout_mask.yaml
image: keepout_mask.pgm
mode: trinary
resolution: 0.05          # MUST match your navigation map
origin: [-10.0, -10.0, 0.0]  # MUST match your navigation map
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

## Launch Setup

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Serve the keepout mask
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_mask_server',
            parameters=[{
                'yaml_filename': '/path/to/keepout_mask.yaml',
                'topic_name': '/keepout_mask',
                'frame_id': 'map',
                'use_sim_time': False
            }],
            output='screen'
        ),

        # Filter info server for keepout
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_filter_info_server',
            parameters=[{
                'type': 0,  # 0 = keepout filter
                'filter_info_topic': '/keepout_filter_info',
                'mask_topic': '/keepout_mask',
                'use_sim_time': False
            }],
            output='screen'
        ),
    ])
```

## Costmap YAML

Add `KeepoutFilter` to the **global costmap** plugins list. Place BEFORE inflation_layer:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "keepout_filter", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

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
          obstacle_range: 5.0
          raytrace_range: 6.5

      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: true
        filter_info_topic: "/keepout_filter_info"

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Keepout in Local Costmap Too?

Usually NOT needed. The global planner avoids the keepout zone, so the robot never approaches it. However, if you want an extra safety layer:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "keepout_filter", "inflation_layer"]

      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: true
        filter_info_topic: "/keepout_filter_info"
```

This prevents the controller from ever commanding motion into a keepout zone, even if pushed by external forces.

## Mask Values

| PGM pixel | OccupancyGrid value | Costmap effect |
|---|---|---|
| 0 (black) | 100 (occupied) | Lethal (254) — full keepout |
| 127 (gray) | ~50 | Still lethal — any occupied value above threshold |
| 254 (white) | 0 (free) | No restriction |

The KeepoutFilter treats any occupied cell in the mask as lethal. There is no graduated effect — it's binary (keepout or not).

## Lifecycle Management

Both `keepout_mask_server` and `keepout_filter_info_server` must be managed by the lifecycle manager:

```yaml
lifecycle_manager:
  ros__parameters:
    node_names: ['controller_server', 'planner_server', 'bt_navigator',
                 'map_server', 'keepout_mask_server', 'keepout_filter_info_server']
```

## Dynamic Keepout Updates

To change keepout zones at runtime:
1. Publish a new OccupancyGrid to the `/keepout_mask` topic
2. The KeepoutFilter picks it up automatically on the next costmap update

This allows adding temporary keepout zones programmatically (e.g., a detected spill area).

## Debugging

- **Planner routes through keepout**: Verify the mask server is active and publishing. Check `ros2 topic echo /keepout_mask --once`. Verify the filter plugin is in the plugins list.
- **Entire map becomes lethal**: The mask image may be inverted. Check `negate: 0` in the YAML and that white = no restriction.
- **Misaligned keepout**: Origin and resolution don't match the navigation map. Compare both YAML files.
- **Visualize**: Add a second Map display in RViz on `/keepout_mask` topic. Overlay it on the main map to verify alignment.
