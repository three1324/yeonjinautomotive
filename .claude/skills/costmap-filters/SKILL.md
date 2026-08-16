<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: costmap-filters
description: 'Configure Nav2 costmap filters. Use when applying keepout zones, speed limits, or preferred lanes via filter masks.'
---

# Costmap Filter Infrastructure

## Overview

Costmap filters are an advanced Nav2 feature for applying spatial masks to the costmap. They enable keepout zones, speed-restricted areas, and custom binary triggers based on spatial regions — without modifying the base map.

## Architecture: Two Components Required

### 1. CostmapFilterInfoServer (standalone node)

Publishes filter metadata on `/costmap_filter_info` telling filter plugins what type of filter to apply and where to find the mask.

### 2. Filter Layer Plugin (in costmap)

A costmap plugin (e.g., `KeepoutFilter`, `SpeedFilter`, `BinaryFilter`) that subscribes to the filter info and the mask OccupancyGrid, then modifies the costmap accordingly.

```
map_server (mask) → /filter_mask topic → Filter Plugin (in costmap)
                                              ↑
CostmapFilterInfoServer → /costmap_filter_info topic
```

## Filter Types

| Type ID | Filter | Purpose |
|---------|--------|---------|
| 0 | KeepoutFilter | Makes marked regions impassable (lethal) |
| 1 | SpeedFilter | Limits robot speed in marked regions |
| 2 | BinaryFilter | Publishes true/false on a topic when robot enters/leaves region |

## The Filter Mask

The mask is a standard `nav_msgs/OccupancyGrid` — same format as a map. It must:

- Cover the same area as the navigation map (same origin, resolution, dimensions)
- Use cell values 0-100 (interpretation depends on filter type)
- Be served by a `map_server` node (or any node publishing OccupancyGrid)

### Creating a Filter Mask

1. Start with the same PGM image used for your map
2. Open in image editor (GIMP, Photoshop)
3. Paint filter regions:
   - For keepout: paint areas black (occupied = 100 in OccupancyGrid)
   - For speed filter: paint grayscale values (darker = slower)
   - Leave non-filtered areas as the original map or white (free = 0)
4. Save as a new PGM file
5. Create a matching YAML file pointing to the PGM

```yaml
# filter_mask.yaml
image: filter_mask.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

**Critical**: The origin and resolution MUST match your navigation map exactly.

## Launch Configuration

```python
# In your launch file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Serve the filter mask on a separate topic
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            parameters=[{
                'yaml_filename': '/path/to/filter_mask.yaml',
                'topic_name': '/filter_mask',
                'frame_id': 'map',
                'use_sim_time': False
            }],
            output='screen'
        ),

        # Costmap filter info server
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            parameters=[{
                'type': 0,                          # 0=keepout, 1=speed, 2=binary
                'filter_info_topic': '/costmap_filter_info',
                'mask_topic': '/filter_mask',
                'use_sim_time': False
            }],
            output='screen'
        ),
    ])
```

Both nodes must be in the `active` lifecycle state. Include them in your lifecycle manager or manually activate them.

## YAML Configuration for the Costmap

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "keepout_filter", "inflation_layer"]

      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: true
        filter_info_topic: "/costmap_filter_info"
```

**Plugin order**: Place filters AFTER obstacle layers but BEFORE inflation. Keepout zones must be seen by the inflation layer to generate proper gradients around them.

## Combining Multiple Filters

You can run multiple filters simultaneously by using separate info servers and mask servers:

```yaml
plugins: ["static_layer", "obstacle_layer", "keepout_filter", "speed_filter", "inflation_layer"]

keepout_filter:
  plugin: "nav2_costmap_2d::KeepoutFilter"
  filter_info_topic: "/keepout_filter_info"

speed_filter:
  plugin: "nav2_costmap_2d::SpeedFilter"
  filter_info_topic: "/speed_filter_info"
```

Each filter needs its own CostmapFilterInfoServer and map_server pair, publishing on different topics.

## Lifecycle Management

Both `map_server` (for the mask) and `costmap_filter_info_server` are lifecycle nodes. They must be activated alongside the rest of the Nav2 stack. Add them to the `lifecycle_manager` node list:

```yaml
lifecycle_manager:
  ros__parameters:
    node_names: ['controller_server', 'planner_server', 'bt_navigator',
                 'filter_mask_server', 'costmap_filter_info_server']
```

## Debugging

- **Filter not working**: Verify both nodes are in `active` state: `ros2 lifecycle get /filter_mask_server`.
- **Mask not aligned**: Check origin and resolution match the base map exactly. Visualize the mask in RViz on its own Map display.
- **No filter_info published**: `ros2 topic echo /costmap_filter_info --once`. If empty, the info server isn't active.
