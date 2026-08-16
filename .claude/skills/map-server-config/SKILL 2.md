<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: map-server-config
description: 'Configure Nav2 map server. Use when loading OccupancyGrid maps, setting YAML parameters, or serving maps to the navigation stack.'
---

# Nav2 Map Server Configuration

## Overview

The map server (`nav2_map_server`) serves a static OccupancyGrid map to the navigation stack. It reads a YAML metadata file pointing to an image file (PGM/PNG), publishes the map on a latched topic, and supports runtime map switching. It runs as a **lifecycle node** managed by `nav2_lifecycle_manager`.

## Map Server Parameters

```yaml
map_server:
  ros__parameters:
    yaml_filename: /home/robot/maps/my_map.yaml   # Path to the map YAML file.
    topic_name: map                                 # Topic to publish OccupancyGrid on. Default: "map".
    frame_id: map                                   # Frame ID set in the published map header. Default: "map".
    use_sim_time: false
```

The `yaml_filename` is the only required parameter. The map server reads this file at activation and publishes the resulting OccupancyGrid with `TRANSIENT_LOCAL` durability QoS, meaning late-joining subscribers (like costmaps) receive the last published map immediately.

## The Map YAML File

```yaml
image: my_map.pgm                  # Path to image file (absolute or relative to this YAML).
resolution: 0.050000               # Meters per pixel. 0.05 = 5cm cells.
origin: [-12.200000, -10.700000, 0.000000]  # [x, y, yaw] of the bottom-left pixel in the map frame.
negate: 0                           # 0 = standard (dark=occupied). 1 = inverted.
occupied_thresh: 0.65               # Pixels with occupancy probability >= this are occupied.
free_thresh: 0.25                   # Pixels with occupancy probability <= this are free.
mode: trinary                       # trinary | scale | raw
```

### Field Details

- **image**: Relative paths are resolved from the YAML file's directory. PGM (uncompressed) or PNG (lossless compression) only.
- **resolution**: Meters per pixel. A 100x100 pixel image at 0.05 resolution covers a 5m × 5m area.
- **origin**: `[x, y, yaw]` — position and rotation of the image's bottom-left corner in the map coordinate frame. `yaw` is rotation in radians (usually 0).
- **negate**: If `1`, black pixels become free and white become occupied. Useful when the source image uses inverted conventions.
- **occupied_thresh / free_thresh**: Define the thresholds for the three-state classification.
- **mode**:
  - `trinary`: Pixels are classified as free (<=free_thresh), occupied (>=occupied_thresh), or unknown (between).
  - `scale`: Continuous probability values [0, 100] mapped linearly from pixel values.
  - `raw`: Pixel byte value is used directly as the occupancy value (0-255 mapped to -1 to 100).

## Lifecycle Management

The map server is managed by `nav2_lifecycle_manager`. It transitions through: `unconfigured → inactive → active`. The map is loaded during the `configure` transition and published when the node becomes `active`.

```yaml
lifecycle_manager:
  ros__parameters:
    autostart: true
    node_names: ['map_server', 'amcl', 'controller_server', 'planner_server']
```

## Loading Maps Dynamically

### Via the /load_map Service

```bash
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
  "{map_url: '/home/robot/maps/floor2.yaml'}"
```

The `LoadMap` service accepts a path to a new YAML file. The server replaces the current map and republishes. This is essential for multi-floor navigation.

### Service Definition (LoadMap.srv)

```
string map_url        # Path to the YAML file.
---
nav_msgs/OccupancyGrid map   # The loaded map.
uint8 result                  # 0=success, else error code.
```

## Interaction with AMCL

AMCL subscribes to `/map` to initialize its particle filter. When a new map is loaded:

1. AMCL receives the new OccupancyGrid.
2. AMCL automatically reinitializes its internal map representation.
3. You **must** publish a new initial pose via `/initialpose` so particles are seeded in the correct location on the new map.

```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}}" --once
```

## Interaction with Costmaps

Both the global and local costmaps in Nav2 subscribe to the `/map` topic via the `StaticLayer` plugin. When a new map is published:

- The **global costmap** replaces its static layer with the new map.
- The **local costmap** typically does not use the static layer (it relies on real-time sensor data), but if configured with one, it also updates.

Ensure the `StaticLayer` plugin has `subscribe_to_updates: true` if you plan to change maps at runtime.

## Launch Example

```python
from launch_ros.actions import Node

map_server = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[{
        'yaml_filename': '/home/robot/maps/my_map.yaml',
        'use_sim_time': False,
    }],
)
```

## Verifying the Map

```bash
# Check if the map is published:
ros2 topic echo /map --once | head -20

# Check map metadata:
ros2 topic echo /map_metadata --once

# Visualize in RViz2:
# Add a Map display, set topic to /map.
```

## Common Issues

| Issue | Cause | Fix |
|-------|-------|-----|
| Map not published | Node not active (lifecycle) | Check lifecycle_manager is managing map_server |
| Blank map | Wrong image path in YAML | Use absolute path or verify relative path base |
| Map rotated/offset | Wrong origin in YAML | Recalculate origin from SLAM output |
| AMCL diverges after map load | No initial_pose published | Publish /initialpose after every map change |
| Costmap doesn't update | StaticLayer missing or `subscribe_to_updates: false` | Add StaticLayer with `subscribe_to_updates: true` |
