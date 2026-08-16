<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: speed-restricted-zones
description: 'Configure Nav2 speed restricted zones. Use when limiting robot velocity in specific map areas via costmap filter masks.'
---

# Speed-Restricted Zones

## Purpose

Speed-restricted zones limit the robot's maximum velocity in designated areas. Use cases: crowded hallways, ramp areas, areas near fragile equipment, zones with reduced sensor visibility.

## How It Works

1. A **mask image** with grayscale values represents speed limits
2. A **map_server** serves the mask as an OccupancyGrid
3. A **CostmapFilterInfoServer** (type=1) provides metadata
4. A **SpeedFilter** plugin reads the mask and publishes speed limits
5. The **controller** subscribes to speed limits and adjusts max velocity

## Mask Value Mapping

The mask uses OccupancyGrid values (0-100) to represent speed limits:

### Percentage mode (percentage: true)
| Mask value | Speed |
|---|---|
| 0 | 0% of max speed (stopped) |
| 50 | 50% of max speed |
| 100 | 100% of max speed (unrestricted) |

### Absolute mode (percentage: false)
| Mask value | Speed |
|---|---|
| 0 | `speed_limit` minimum (from parameter) |
| 100 | `speed_limit` maximum (from parameter) |

**Percentage mode is recommended** — it works regardless of the robot's configured max speed.

## Creating the Speed Mask

### Step 1: Start with a blank white image

Same dimensions, resolution, and origin as your navigation map. White = free = no speed restriction (100%).

### Step 2: Paint speed zones

Use grayscale values:
- **White (254)** → free (value 0 in OccupancyGrid) → unrestricted speed (100%)
- **Light gray (191)** → ~25 in OccupancyGrid → 25% speed limit
- **Medium gray (127)** → ~50 in OccupancyGrid → 50% speed limit
- **Dark gray (64)** → ~75 in OccupancyGrid → 75% speed limit
- **Black (0)** → 100 in OccupancyGrid → 0% speed (should not typically be used — use keepout instead)

**Note on inversion**: The PGM-to-OccupancyGrid mapping inverts values (white in PGM = free = 0 in OccupancyGrid). For percentage mode, OccupancyGrid 0 = 0% speed and 100 = 100% speed. Since white PGM → 0 in OccupancyGrid → 0% speed, paint unrestricted areas as FREE in the OccupancyGrid context. Check your `negate` and threshold settings carefully.

Simpler approach: create the OccupancyGrid directly from a script:

```python
import numpy as np
from nav_msgs.msg import OccupancyGrid

# Start with 100 (full speed) everywhere
mask = np.full((height, width), 100, dtype=np.int8)

# Set a region to 50% speed
mask[y1:y2, x1:x2] = 50

# Set a region to 25% speed  
mask[y3:y4, x3:x4] = 25
```

### Step 3: Create YAML

```yaml
# speed_mask.yaml
image: speed_mask.pgm
mode: raw             # Use 'raw' to preserve exact pixel values
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
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
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='speed_mask_server',
            parameters=[{
                'yaml_filename': '/path/to/speed_mask.yaml',
                'topic_name': '/speed_mask',
                'frame_id': 'map',
                'use_sim_time': False
            }],
            output='screen'
        ),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='speed_filter_info_server',
            parameters=[{
                'type': 1,                      # 1 = speed filter
                'filter_info_topic': '/speed_filter_info',
                'mask_topic': '/speed_mask',
                'use_sim_time': False
            }],
            output='screen'
        ),
    ])
```

## Costmap YAML

Add `SpeedFilter` to the global costmap plugins:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "speed_filter", "inflation_layer"]

      speed_filter:
        plugin: "nav2_costmap_2d::SpeedFilter"
        enabled: true
        filter_info_topic: "/speed_filter_info"
        speed_limit_topic: "/speed_limit"  # Controllers subscribe to this
        percentage: true
```

### speed_limit_topic

The SpeedFilter publishes `nav2_msgs/SpeedLimit` messages on this topic. The controller server subscribes and adjusts max velocity accordingly.

## Controller Support

### MPPI Controller
Natively supports speed limits. No extra configuration — it subscribes to `/speed_limit` automatically when available.

### Regulated Pure Pursuit (RPP)
Supports speed limits natively. The regulated speed is capped by the speed limit.

### DWB Controller
Does NOT subscribe to `/speed_limit` natively. To support speed-limited zones with DWB, you need a custom critic or an external node that modifies the `max_vel_x` parameter dynamically based on `/speed_limit` messages.

## Complete Configuration Example

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

      plugins: ["static_layer", "obstacle_layer", "speed_filter", "keepout_filter", "inflation_layer"]

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

      speed_filter:
        plugin: "nav2_costmap_2d::SpeedFilter"
        enabled: true
        filter_info_topic: "/speed_filter_info"
        speed_limit_topic: "/speed_limit"
        percentage: true

      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: true
        filter_info_topic: "/keepout_filter_info"

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Lifecycle Management

Add to lifecycle manager:

```yaml
lifecycle_manager:
  ros__parameters:
    node_names: ['controller_server', 'planner_server', 'bt_navigator',
                 'map_server', 'speed_mask_server', 'speed_filter_info_server']
```

## Debugging

- **Speed not limiting**: Verify controller supports it (MPPI/RPP). Check `ros2 topic echo /speed_limit` while the robot is in a restricted zone.
- **Wrong speed values**: Check mask alignment and value mapping. Visualize `/speed_mask` in RViz.
- **Speed changes are jerky**: The speed limit changes abruptly at zone boundaries. Consider using gradient edges in the mask for smoother transitions.
- **Robot ignores speed limit entirely**: The controller may not be subscribed. Check controller parameters and verify `/speed_limit` topic has subscribers.
