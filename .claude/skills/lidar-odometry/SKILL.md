<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: lidar-odometry
description: 'Set up lidar odometry. Use when using laser-based ego-motion with rf2o_laser_odometry or KISS-ICP.'
---

# Lidar Odometry — Scan Matching for Motion Estimation

## Overview

Lidar odometry estimates robot motion by matching consecutive laser scans (2D) or point clouds (3D). It provides a `nav_msgs/Odometry` output that can supplement or replace wheel odometry, especially on surfaces with poor traction.

---

## Available Packages

| Package | Input | Method | ROS 2 Status |
|---------|-------|--------|-------------|
| `rf2o_laser_odometry` | 2D LaserScan | Range-flow (closed-form scan matching) | Available, lightweight |
| `kiss-icp` | 3D PointCloud2 | ICP with adaptive thresholding | Active development, excellent |
| `scan_tools` (laser_scan_matcher) | 2D LaserScan | Correlative Scan Matching (CSM) | ROS 2 ports exist |

---

## rf2o_laser_odometry

### How It Works

rf2o computes scan-to-scan motion by formulating the problem as a range flow constraint. Instead of feature extraction and matching, it directly estimates the rigid-body transform that minimizes the difference between consecutive scan ranges. This makes it fast and robust in geometrically rich environments.

### Configuration

```yaml
rf2o_laser_odometry_node:
  ros__parameters:
    laser_scan_topic: "/scan"
    odom_topic: "/odom_rf2o"
    base_frame_id: "base_link"        # Robot body frame
    odom_frame_id: "odom_rf2o"        # Output frame (use distinct name if not replacing wheel odom)
    publish_tf: false                  # Let EKF handle TF
    freq: 20.0                         # Processing rate (Hz)
    init_pose_from_topic: ""           # Leave empty for default initialization
    verbose: false
```

### Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,
                'freq': 20.0,
            }],
            output='screen',
        ),
    ])
```

---

## kiss-icp (3D Lidar)

For robots with 3D lidar (e.g., Ouster, Velodyne, Livox):

```yaml
kiss_icp_node:
  ros__parameters:
    topic: "/points"
    base_frame: "base_link"
    odom_frame: "odom"
    publish_odom_tf: false
    max_range: 100.0
    min_range: 0.5
    voxel_size: 0.5           # Downsampling resolution (meters)
    max_points_per_voxel: 20
    initial_threshold: 2.0     # Adaptive ICP threshold start
    min_motion_th: 0.1         # Minimum motion to trigger registration
```

---

## Strengths and Weaknesses

### When Lidar Odometry Excels
- **Feature-rich environments**: offices with walls, furniture, doors — many geometric features to match against
- **Wheel slip scenarios**: smooth floors, ramps, thresholds where encoders are unreliable
- **Constant lighting**: unlike visual odometry, lidar is unaffected by illumination

### When Lidar Odometry Struggles
- **Long featureless corridors**: parallel walls provide only one constraint axis, causing drift along the corridor
- **Symmetric environments**: identical rooms or hallways cause ambiguity
- **Open spaces**: sparse features lead to poor scan matching
- **Dynamic environments**: many moving obstacles corrupt scan matching (people, pets)

---

## EKF Integration

### As Supplement to Wheel Odometry

Use differential mode for lidar odom, similar to visual odometry:

```yaml
ekf_filter_node:
  ros__parameters:
    # Primary: wheel odometry
    odom0: "odom/unfiltered"
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]

    # Secondary: lidar odometry (differential)
    odom1: "odom_rf2o"
    odom1_config: [true,  true,  false,
                   false, false, true,
                   false, false, false,
                   false, false, false,
                   false, false, false]
    odom1_differential: true
    odom1_queue_size: 5

    imu0: "imu/data"
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  true,  false, false]
    imu0_remove_gravitational_acceleration: true
```

### As Replacement for Wheel Odometry

If wheels are highly unreliable, lidar odom can be the primary source. Fuse velocities directly (non-differential):

```yaml
odom0: "odom_rf2o"
odom0_config: [false, false, false,
               false, false, false,
               true,  false, false,
               false, false, true,
               false, false, false]
odom0_differential: false
```

**Warning**: If both lidar odom and wheel odom provide vyaw, the EKF weights them by covariance. Ensure covariances are realistic — don't set both to near-zero.

---

## scan_matcher_karto (Alternative)

Part of the SLAM Toolbox ecosystem, `scan_matcher_karto` uses correlation-based scan matching:

```yaml
scan_matcher_karto_node:
  ros__parameters:
    use_scan_matching: true
    use_scan_barycenter: true
    resolution: 0.05
    range_threshold: 12.0
    minimum_travel_distance: 0.1
    minimum_travel_heading: 0.1
```

This is heavier than rf2o but can be more robust in environments with sparse features.

---

## Performance Comparison

| Metric | rf2o | kiss-icp | scan_matcher_karto |
|--------|------|----------|--------------------|
| CPU load | Low (~5% single core) | Moderate (10-20%) | Moderate (10-15%) |
| Accuracy (feature-rich) | Good | Excellent | Good |
| Accuracy (feature-poor) | Poor | Fair | Fair |
| Latency | <5ms per scan | ~20ms per cloud | ~10ms per scan |
| Input | 2D LaserScan | 3D PointCloud2 | 2D LaserScan |

---

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| Lidar odom drifts in corridors | Insufficient geometric constraints perpendicular to corridor | Supplement with wheel odom; corridor drift is fundamental |
| Odom jumps when door opens/closes | Large scan change between consecutive frames | Increase covariance, use differential mode in EKF |
| High CPU under rf2o | Processing rate too high for scan density | Reduce `freq` or downsample the scan |
| Zero output from rf2o | TF from laser frame to base_link missing | Ensure robot_state_publisher provides the laser→base_link transform |
