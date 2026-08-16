<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: lidar-filtering
description: 'Filter lidar data with laser_filters. Use when removing angular ranges, applying median filters, or shadow/speckle filtering.'
---

# LIDAR Filtering with laser_filters

## Overview

The `laser_filters` package provides a chain of configurable filters applied to `sensor_msgs/msg/LaserScan` messages. Filters are loaded as plugins and executed sequentially. The filtered output is published on a separate topic for consumption by Nav2 costmaps and other nodes.

## Filter Types

### LaserScanAngularBoundsFilter

Excludes readings within an angular range. Primary use: remove readings where the LIDAR sees the robot's own chassis.

```yaml
- name: angular_bounds
  type: laser_filters/LaserScanAngularBoundsFilter
  params:
    lower_angle: -2.356       # -135° in radians
    upper_angle: -1.920       # -110° in radians
```

Readings between `lower_angle` and `upper_angle` are set to `NaN`. Multiple instances can exclude multiple ranges (e.g., two chassis legs at different angles).

### LaserScanAngularBoundsFilterInPlace

Inverse of the above — keeps only readings within the angular range, discards everything else. Useful for extracting a forward-facing sector:

```yaml
- name: forward_only
  type: laser_filters/LaserScanAngularBoundsFilterInPlace
  params:
    lower_angle: -1.5708      # -90°
    upper_angle: 1.5708       #  90°
```

### LaserScanRangeFilter

Excludes readings outside a min/max range. Handles both close-range noise (ground reflections, sensor minimum) and far-range unreliable readings.

```yaml
- name: range_filter
  type: laser_filters/LaserScanRangeFilter
  params:
    use_message_range_limits: false   # use our own limits
    lower_threshold: 0.15             # meters
    upper_threshold: 12.0             # meters
    lower_replacement_value: .nan     # set bad readings to NaN
    upper_replacement_value: .inf     # beyond max → infinity (no obstacle)
```

### LaserScanBoxFilter

Excludes or includes readings that fall within a 3D box defined in a target TF frame. Powerful for removing the robot footprint from the scan regardless of LIDAR mounting angle.

```yaml
- name: box_filter
  type: laser_filters/LaserScanBoxFilter
  params:
    box_frame: "base_link"
    min_x: -0.25
    max_x: 0.25
    min_y: -0.20
    max_y: 0.20
    min_z: -0.5
    max_z: 0.5
    invert: false             # false = remove points inside box
```

`invert: true` keeps only points inside the box (useful for cropping to a region of interest).

### LaserScanShadowsFilter

Removes shadow artifacts that occur at object edges. When the laser beam grazes an object's edge, the return from behind the object creates a "shadow" point at an incorrect intermediate range.

```yaml
- name: shadows
  type: laser_filters/LaserScanShadowsFilter
  params:
    min_angle: 10.0           # degrees — minimum angle between adjacent beams
    max_angle: 170.0          # degrees — maximum angle
    neighbors: 1              # number of adjacent readings to compare
    window: 1                 # filtering window size
    remove_shadow_start_point: true
```

Shadow points produce sharp angle changes between consecutive readings. The filter detects these by computing the angle formed by adjacent scan points and removing readings where the angle is outside `[min_angle, max_angle]`.

### LaserScanSpeckleFilter

Removes isolated noise points (speckle) that appear randomly due to sensor noise, dust, or multipath reflections.

```yaml
- name: speckle
  type: laser_filters/LaserScanSpeckleFilter
  params:
    filter_type: 0            # 0 = distance-based, 1 = euclidean-based
    max_range: 2.0            # only filter within this range
    max_range_difference: 0.1 # max allowed range difference between neighbors
    filter_window: 2          # number of neighboring readings to check
```

A point is classified as speckle if its range differs from all neighbors by more than `max_range_difference`.

### LaserScanIntensityFilter

Filters readings based on return signal intensity. Useful for removing weak returns from glass, black surfaces, or transparent obstacles.

```yaml
- name: intensity
  type: laser_filters/LaserScanIntensityFilter
  params:
    lower_threshold: 100      # minimum intensity to keep
    upper_threshold: 10000    # maximum intensity to keep
    filter_override_range: true
    filter_override_intensity: true
```

## Complete Filter Chain for Indoor Robot

```yaml
# laser_filter_chain.yaml
scan_filter_chain:
  # 1. Remove chassis self-hits (rear-left support bracket)
  - name: chassis_rear_left
    type: laser_filters/LaserScanAngularBoundsFilter
    params:
      lower_angle: -2.44      # -140°
      upper_angle: -2.09      # -120°

  # 2. Remove chassis self-hits (rear-right support bracket)
  - name: chassis_rear_right
    type: laser_filters/LaserScanAngularBoundsFilter
    params:
      lower_angle: 2.09       # 120°
      upper_angle: 2.44       # 140°

  # 3. Enforce valid range
  - name: range
    type: laser_filters/LaserScanRangeFilter
    params:
      use_message_range_limits: false
      lower_threshold: 0.12
      upper_threshold: 12.0
      lower_replacement_value: .nan
      upper_replacement_value: .inf

  # 4. Remove shadow artifacts at edges
  - name: shadows
    type: laser_filters/LaserScanShadowsFilter
    params:
      min_angle: 10.0
      max_angle: 170.0
      neighbors: 1
      window: 1
      remove_shadow_start_point: true

  # 5. Remove speckle noise
  - name: speckle
    type: laser_filters/LaserScanSpeckleFilter
    params:
      filter_type: 0
      max_range: 2.0
      max_range_difference: 0.1
      filter_window: 2
```

## Launch Integration

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    filter_config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config', 'laser_filter_chain.yaml'
    )

    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_chain',
            output='screen',
            parameters=[filter_config],
            remappings=[
                ('scan', '/scan_raw'),
                ('scan_filtered', '/scan'),
            ],
        ),
    ])
```

## Debugging

Visualize both `/scan_raw` and `/scan` (filtered) in RViz simultaneously. Use different colors. Rotate the robot slowly and verify:
- Chassis points are removed
- Walls remain clean
- No excessive filtering of valid obstacles

Use `ros2 topic hz /scan` to verify the filter chain does not reduce the scan rate (it should match the input rate).

## Common Pitfall

Filter order matters. Apply angular and range filters first (cheap, remove large chunks), then shadow and speckle filters (more expensive, operate on the remaining valid points). Placing the speckle filter before the range filter may waste computation on out-of-range readings.
