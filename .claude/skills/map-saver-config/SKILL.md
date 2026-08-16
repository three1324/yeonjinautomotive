<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: map-saver-config
description: 'Configure Nav2 map saver. Use when saving maps from SLAM, setting thresholds, or automatic periodic map saving.'
---

# Nav2 Map Saver Configuration

## Overview

The map saver (`nav2_map_server` package, `map_saver_server` executable) saves the current OccupancyGrid map to disk as a YAML + image file pair. It can be used via its service interface or the convenience CLI tool. The saved map can then be loaded by the map server for navigation.

## Map Saver Server Parameters

```yaml
map_saver:
  ros__parameters:
    save_map_timeout: 2.0                # Timeout (seconds) waiting for map data when saving.
    free_thresh_default: 0.25            # Default free threshold written to the output YAML.
    occupied_thresh_default: 0.65        # Default occupied threshold written to the output YAML.
    map_subscribe_transient_local: true  # Subscribe with TRANSIENT_LOCAL QoS to get latched maps.
    use_sim_time: false
```

### QoS: TRANSIENT_LOCAL

When `map_subscribe_transient_local: true`, the saver subscribes with `TRANSIENT_LOCAL` durability. This means it receives the last published map even if it connects after the map server published. This is the correct setting for saving maps published by map_server or SLAM Toolbox. Set to `false` only if your map publisher uses `VOLATILE` QoS.

## CLI Tool: map_saver_cli

The simplest way to save a map:

```bash
# Save to my_map.pgm + my_map.yaml in the current directory:
ros2 run nav2_map_server map_saver_cli -f my_map

# Save to a specific directory:
ros2 run nav2_map_server map_saver_cli -f /home/robot/maps/floor1

# Save as PNG instead of PGM:
ros2 run nav2_map_server map_saver_cli -f my_map --image-format png

# Remap the map topic (if not default /map):
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -r map:=/slam_toolbox/map

# Set custom thresholds:
ros2 run nav2_map_server map_saver_cli -f my_map --free 0.15 --occ 0.65

# Use sim time:
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p use_sim_time:=true
```

### CLI Arguments

| Argument | Description |
|----------|-------------|
| `-f, --output` | Output file base name (without extension) |
| `--image-format` | `pgm` (default) or `png` |
| `--free` | Free threshold (default 0.25) |
| `--occ` | Occupied threshold (default 0.65) |
| `--mode` | `trinary` (default), `scale`, or `raw` |
| `-t, --map-topic` | Map topic to subscribe to (default `/map`) |

## Service Interface

The map saver server exposes a service for programmatic saving:

```bash
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: '/map', map_url: '/home/robot/maps/saved_map', \
    image_format: 'pgm', map_mode: 'trinary', \
    free_thresh: 0.25, occupied_thresh: 0.65}"
```

### SaveMap.srv Definition

```
string map_topic         # Topic to read the map from.
string map_url           # Output file path (base name, no extension).
string image_format      # "pgm" or "png".
string map_mode          # "trinary", "scale", or "raw".
float32 free_thresh      # Free threshold for the YAML.
float32 occupied_thresh  # Occupied threshold for the YAML.
---
bool result              # true = success.
```

## CLI vs Service: When to Use Each

| Use Case | Approach |
|----------|----------|
| Manual map save during development | CLI tool |
| Automated periodic saves from a node | Service call |
| Saving after mapping run completes | CLI tool |
| Multi-floor save automation | Service call from a manager node |

## Saving SLAM Toolbox Maps

For SLAM Toolbox, saving the OccupancyGrid (PGM) gives you a static map but **loses the pose graph**. To preserve the full graph state (for lifelong mode or localization mode):

```bash
# Serialize the full graph:
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/robot/maps/house_map'}"
```

This produces `.posegraph` + `.data` files. You should save **both**: the serialized graph (for continued SLAM) and the PGM/YAML (for static navigation with AMCL).

## Output Files

After running `map_saver_cli -f my_map`, you get:

**my_map.yaml:**
```yaml
image: my_map.pgm
mode: trinary
resolution: 0.05
origin: [-12.2, -10.7, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

**my_map.pgm:** A grayscale image where:
- Pixel value 0 → occupied (black)
- Pixel value 205 → free (light gray)  
- Pixel value 254 → unknown (near-white)

## Common Mistakes

### Saving Before the Map Is Stable

If you save while SLAM is still building a submap, edges of the map will have partial/uncertain data. Wait until:
- The robot has completed at least one loop closure.
- No active submap construction is in progress at the area of interest.
- The map has visually stabilized in RViz.

### Topic QoS Mismatch

If the CLI hangs and times out, the QoS settings may not match:

```bash
# Check the publisher's QoS:
ros2 topic info /map --verbose
```

If the publisher uses `RELIABLE` + `TRANSIENT_LOCAL`, the saver must also use `TRANSIENT_LOCAL` (the default). If the publisher uses `BEST_EFFORT`, you need `map_subscribe_transient_local: false`.

### Wrong Map Topic

SLAM Toolbox publishes on `/map` by default, but if you're running multiple SLAM instances or have remapped topics, specify the correct one:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map -t /slam_toolbox/map
```

## Automated Periodic Saving (Python)

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap

class PeriodicMapSaver(Node):
    def __init__(self):
        super().__init__('periodic_map_saver')
        self.client = self.create_client(SaveMap, '/map_saver/save_map')
        self.timer = self.create_timer(300.0, self.save_map)  # Every 5 minutes.

    def save_map(self):
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Map saver service not available')
            return
        req = SaveMap.Request()
        req.map_topic = '/map'
        req.map_url = '/home/robot/maps/auto_save'
        req.image_format = 'pgm'
        req.map_mode = 'trinary'
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
        future = self.client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'Map saved: {f.result().result}'))
```
