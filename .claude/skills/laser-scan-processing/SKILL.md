<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: laser-scan-processing
description: 'Process LaserScan data in ROS 2. Use when filtering, segmenting, or extracting features from 2D lidar scans.'
---

# LaserScan Processing

## Message Structure

`sensor_msgs/msg/LaserScan` fields:

| Field | Type | Description |
|---|---|---|
| `angle_min` | float32 | Start angle of the scan (rad) |
| `angle_max` | float32 | End angle of the scan (rad) |
| `angle_increment` | float32 | Angular distance between measurements (rad) |
| `time_increment` | float32 | Time between measurements (s); 0 if scan is instantaneous |
| `scan_time` | float32 | Time between full scans (s) |
| `range_min` | float32 | Minimum valid range value (m) |
| `range_max` | float32 | Maximum valid range value (m) |
| `ranges[]` | float32[] | Range data; `inf` = no return, `NaN` = measurement error |
| `intensities[]` | float32[] | Intensity data (optional, sensor-dependent) |

The angle for the i-th reading is `angle_min + i * angle_increment`. Valid readings satisfy `range_min <= ranges[i] <= range_max`. Values of `inf` mean the beam did not return (open space beyond max range). Values of `NaN` indicate a sensor error for that ray.

## Polar to Cartesian Conversion

Each valid range reading converts to a 2D point in the sensor frame:

```python
angle = msg.angle_min + i * msg.angle_increment
x = msg.ranges[i] * math.cos(angle)
y = msg.ranges[i] * math.sin(angle)
```

Always check `math.isfinite(msg.ranges[i])` before conversion. The resulting points are in the LIDAR's own frame (typically `laser_frame` or `base_scan`).

## Filtering Techniques

**Min/Max Range Filtering**: Discard readings outside a working range. Removes ground returns (very short range) and unreliable far readings:

```python
if msg.range_min <= r <= msg.range_max and math.isfinite(r):
    # valid
```

**Angular Sector Selection**: Extract only a forward-facing wedge or exclude the rear arc. Useful when rear sensors are occluded by the chassis.

**Median Filtering**: Replace each reading with the median of a sliding window (e.g., 5 readings). Suppresses impulse noise without blurring edges as much as mean filtering.

## laser_filters Package

The `laser_filters` package provides a composable filter chain applied to `LaserScan` messages. Each filter is a plugin loaded at runtime.

### Key Filter Plugins

| Filter | Purpose |
|---|---|
| `LaserScanAngularBoundsFilter` | Exclude readings in an angular range (e.g., chassis self-hits) |
| `LaserScanRangeFilter` | Clamp or discard readings outside min/max range |
| `LaserScanBoxFilter` | Exclude/include points within a 3D box in a target frame |
| `LaserScanShadowsFilter` | Remove shadow artifacts at object edges |
| `LaserScanSpeckleFilter` | Remove isolated speckle noise points |

### Filter Chain YAML

```yaml
scan_filter_chain:
- name: chassis_exclusion
  type: laser_filters/LaserScanAngularBoundsFilter
  params:
    lower_angle: -2.35619    # -135 degrees
    upper_angle: -1.91986    # -110 degrees — chassis visible here

- name: range_limit
  type: laser_filters/LaserScanRangeFilter
  params:
    use_message_range_limits: false
    lower_threshold: 0.15
    upper_threshold: 12.0
    lower_replacement_value: .nan
    upper_replacement_value: .inf

- name: speckle
  type: laser_filters/LaserScanSpeckleFilter
  params:
    filter_type: 0           # 0 = distance-based
    max_range: 2.0
    max_range_difference: 0.1
    filter_window: 2

- name: shadows
  type: laser_filters/LaserScanShadowsFilter
  params:
    min_angle: 10.0          # degrees
    max_angle: 170.0
    neighbors: 1
    window: 1
```

### Launch Integration

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter_chain',
            parameters=[{'use_sim_time': False}],
            remappings=[
                ('scan', '/scan_raw'),
                ('scan_filtered', '/scan'),
            ],
            # Load filter chain from YAML
            arguments=['--ros-args', '--params-file', 'laser_filters.yaml'],
        ),
    ])
```

## Costmap Integration

Nav2 costmaps consume `LaserScan` on the `observation_sources` topic. Publish the filtered scan to `/scan` and configure the costmap:

```yaml
observation_sources: scan
scan:
  topic: /scan
  data_type: LaserScan
  marking: true
  clearing: true
  max_obstacle_height: 2.0
  min_obstacle_height: 0.0
  obstacle_max_range: 10.0
  obstacle_min_range: 0.15
  raytrace_max_range: 12.0
  raytrace_min_range: 0.0
```

## Common Issue: Chassis Self-Hits

If the LIDAR is mounted where it can see the robot's own body, those ranges appear as permanent close obstacles. Diagnose by visualizing `/scan` in RViz and rotating the robot — fixed points in the `base_link` frame are chassis hits. Fix with `LaserScanAngularBoundsFilter` excluding the affected angular range, or `LaserScanBoxFilter` excluding a box around the robot footprint.
