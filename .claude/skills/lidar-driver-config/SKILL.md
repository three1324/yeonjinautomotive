<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: lidar-driver-config
description: 'Configure lidar drivers for ROS 2. Use when setting up LD19, RPLidar, Hokuyo, or Velodyne driver parameters.'
---

# LIDAR Driver Configuration for ROS 2

## LIDAR Options for Mobile Robots

| LIDAR | Range | Package | Interface | Price Tier |
|-------|-------|---------|-----------|------------|
| LDROBOT LD19 | 0.02-12m, 360° | ldlidar_stl_ros2 | UART (CP2102) | Budget |
| RPLidar A1 | 0.15-12m, 360° | rplidar_ros / sllidar_ros2 | UART (CP2102) | Budget |
| RPLidar A2 M8 | 0.2-12m, 360° | rplidar_ros / sllidar_ros2 | UART (CP2102) | Mid |
| RPLidar S2 | 0.05-30m, 360° | sllidar_ros2 | UART | Mid |
| Hokuyo URG-04LX | 0.02-5.6m, 240° | urg_node2 | USB/Ethernet | High |
| Velodyne VLP-16 | 1-100m, 360°×30° | velodyne_driver | Ethernet | Premium |
| Livox Mid-360 | 0.1-40m, 360°×59° | livox_ros_driver2 | Ethernet | Mid-High |

## LDROBOT LD19 Configuration

The LD19 is a common budget 2D lidar. Uses a CP2102 USB-serial adapter at 230400 baud.

### Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ldlidar',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD19',
                'topic_name': '/scan',
                'frame_id': 'laser_frame',
                'port_name': '/dev/lidar',
                'port_baudrate': 230400,
                'laser_scan_dir': True,          # True = counterclockwise (ROS standard)
                'enable_angle_crop_func': False,
                'angle_crop_min': 0.0,
                'angle_crop_max': 360.0,
            }],
        ),
    ])
```

### Key Parameters

- `laser_scan_dir`: The LD19 scans clockwise physically. Set `True` to flip to counterclockwise (ROS convention). If your map builds mirrored, toggle this.
- `frame_id`: Must match the URDF link name where the lidar is mounted. This connects the scan data to the robot's TF tree.
- `enable_angle_crop_func`: Set `True` to exclude angular ranges where the robot's body is visible. Useful when the lidar is partially obstructed by the chassis.

## RPLidar / Slamtec Configuration

### rplidar_ros (Legacy Package)

```python
Node(
    package='rplidar_ros',
    executable='rplidar_node',
    name='rplidar',
    parameters=[{
        'serial_port': '/dev/rplidar',
        'serial_baudrate': 256000,         # A2: 256000, A1: 115200
        'frame_id': 'laser_frame',
        'inverted': False,
        'angle_compensate': True,
        'scan_mode': 'Standard',           # or 'Express', 'Boost', 'Sensitivity'
    }],
)
```

### sllidar_ros2 (Current Slamtec Package)

```python
Node(
    package='sllidar_ros2',
    executable='sllidar_node',
    name='sllidar',
    parameters=[{
        'channel_type': 'serial',
        'serial_port': '/dev/rplidar',
        'serial_baudrate': 256000,
        'frame_id': 'laser_frame',
        'inverted': False,
        'angle_compensate': True,
        'scan_mode': 'Standard',
        'scan_frequency': 10.0,            # Hz, adjustable 5-15 Hz
    }],
)
```

### Scan Modes

| Mode | Points/Scan | Quality | Use Case |
|------|-------------|---------|----------|
| Standard | ~400 | Good | Navigation, general SLAM |
| Express | ~2000 | Good | Dense mapping |
| Boost | ~4000 | Lower | High-res short range |
| Sensitivity | ~2000 | Best | Long range, reflective surfaces |

## Hokuyo URG Configuration

```python
Node(
    package='urg_node2',
    executable='urg_node2_node',
    name='urg_node',
    parameters=[{
        'ip_address': '',                  # empty for USB
        'serial_port': '/dev/hokuyo',
        'serial_baud': 115200,
        'frame_id': 'laser_frame',
        'angle_min': -2.356,               # -135° in radians
        'angle_max': 2.356,                # +135° in radians
        'range_min': 0.02,
        'range_max': 5.6,
        'cluster': 1,                      # group N adjacent points
        'skip': 0,                         # skip N scans between publishes
    }],
)
```

## Frame ID and URDF Integration

The `frame_id` parameter connects laser data to the robot model. In URDF:

```xml
<link name="laser_frame">
  <visual>
    <geometry><cylinder length="0.04" radius="0.035"/></geometry>
  </visual>
</link>

<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser_frame"/>
  <origin xyz="0.15 0.0 0.12" rpy="0 0 0"/>
</joint>
```

The `xyz` values define the lidar's position relative to `base_link`. **Measure carefully** — a 2cm error in the lidar mount position causes SLAM map distortion.

## Inverted Mounting

If the lidar is mounted upside-down (common for under-chassis mounting):

- Set `inverted: True` in the driver — this flips the scan vertically
- In URDF, add `rpy="3.14159 0 0"` (180° roll) to the joint origin
- Verify with RViz: scan points should appear at the correct height and orientation

## Orientation and Scan Direction

ROS `sensor_msgs/LaserScan` convention:
- `angle_min` to `angle_max` sweeps counterclockwise when viewed from above
- 0° points forward (along the robot's x-axis)

If your SLAM map appears mirrored, the scan direction is wrong. Solutions:
1. Driver parameter: `laser_scan_dir`, `inverted`, or `scan_dir_reverse`
2. URDF: Rotate the laser frame by 180° around z: `rpy="0 0 3.14159"`

## Angle Offset

If the lidar's zero-angle doesn't align with the robot's forward direction:

```yaml
# Rotate the scan by adding angle_offset in the URDF transform
# If lidar zero is 90° to the right of robot forward:
<origin xyz="0.15 0.0 0.12" rpy="0 0 1.5708"/>  # 90° yaw
```

Alternatively, some drivers have an `angle_offset` parameter. Either approach works; URDF is preferred because it's visible in RViz and doesn't depend on driver support.

## Multi-LIDAR Setup

For robots with multiple lidars (e.g., front + rear for full 360° coverage):

```python
Node(
    package='ldlidar_stl_ros2',
    executable='ldlidar_stl_ros2_node',
    name='lidar_front',
    parameters=[{
        'port_name': '/dev/lidar_front',
        'topic_name': '/scan_front',
        'frame_id': 'laser_front_frame',
    }],
),
Node(
    package='ldlidar_stl_ros2',
    executable='ldlidar_stl_ros2_node',
    name='lidar_rear',
    parameters=[{
        'port_name': '/dev/lidar_rear',
        'topic_name': '/scan_rear',
        'frame_id': 'laser_rear_frame',
    }],
),
```

### Merging Scans

Use `ira_laser_tools` to combine multiple LaserScan topics:

```python
Node(
    package='ira_laser_tools',
    executable='laserscan_multi_merger',
    name='scan_merger',
    parameters=[{
        'destination_frame': 'base_link',
        'cloud_destination_topic': '/merged_cloud',
        'scan_destination_topic': '/scan',
        'laserscan_topics': '/scan_front /scan_rear',
        'angle_min': -3.14159,
        'angle_max': 3.14159,
        'range_min': 0.05,
        'range_max': 12.0,
    }],
)
```

The merger transforms each scan into `base_link` frame (using TF), combines them, and publishes a unified `/scan` topic for SLAM and navigation.

## Laser Filtering

Use `laser_filters` to clean up scan data before feeding it to SLAM:

```yaml
scan_to_scan_filter_chain:
  ros__parameters:
    filter1:
      name: range
      type: laser_filters/LaserScanRangeFilter
      params:
        lower_threshold: 0.05
        upper_threshold: 10.0
    filter2:
      name: angular
      type: laser_filters/LaserScanAngularBoundsFilter
      params:
        lower_angle: -2.5
        upper_angle: 2.5
    filter3:
      name: median
      type: laser_filters/LaserScanMedianFilter
      params:
        window_size: 3
```

Filters remove out-of-range readings, crop angular sections (exclude robot body), and smooth noisy measurements.
