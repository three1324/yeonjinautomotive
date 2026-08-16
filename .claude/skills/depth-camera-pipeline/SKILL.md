<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: depth-camera-pipeline
description: 'Set up depth camera processing pipeline. Use when converting depth images to point clouds, setting up registered depth, or depth filtering.'
---

# Depth Camera Pipeline

## Pipeline Overview

```
raw depth image ──→ depth_image_proc ──→ point cloud ──→ filters ──→ costmap integration
       │
       └──→ depthimage_to_laserscan ──→ virtual LaserScan ──→ costmap integration
```

Depth cameras (Intel RealSense, OAK-D, Azure Kinect) produce registered depth images (`sensor_msgs/Image`, encoding `16UC1` or `32FC1`) alongside color images. Two main paths exist for Nav2 integration: through point clouds or through virtual laser scans.

## depth_image_proc Package

Part of `image_pipeline`. Converts depth images and camera info into point clouds.

### point_cloud_xyz

Generates an uncolored `PointCloud2` from a depth image and `CameraInfo`:

```python
ComposableNode(
    package='depth_image_proc',
    plugin='depth_image_proc::PointCloudXyzNode',
    name='point_cloud_xyz',
    remappings=[
        ('image_rect', '/camera/depth/image_rect_raw'),
        ('camera_info', '/camera/depth/camera_info'),
        ('points', '/camera/points'),
    ],
),
```

### point_cloud_xyzrgb

Generates a colored point cloud by fusing depth and registered color image:

```python
ComposableNode(
    package='depth_image_proc',
    plugin='depth_image_proc::PointCloudXyzrgbNode',
    name='point_cloud_xyzrgb',
    remappings=[
        ('depth_registered/image_rect', '/camera/aligned_depth_to_color/image_raw'),
        ('rgb/image_rect_color', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('points', '/camera/points_rgb'),
    ],
),
```

Both nodes require the depth image and `CameraInfo` to share the same frame. For stereo cameras, the factory-calibrated `CameraInfo` provides the intrinsics (fx, fy, cx, cy) needed for back-projection.

## depthimage_to_laserscan

Converts a depth image directly into a `sensor_msgs/LaserScan` without creating an intermediate point cloud. Lighter on CPU than the point-cloud path.

### How It Works

Selects a horizontal band of `scan_height` pixel rows centered on the image, takes the minimum depth in each column, and produces a 2D scan.

### Configuration

```yaml
depthimage_to_laserscan_node:
  ros__parameters:
    scan_height: 10           # number of pixel rows to aggregate
    scan_time: 0.033          # seconds between scans
    range_min: 0.15           # minimum valid range (m)
    range_max: 10.0           # maximum valid range (m)
    output_frame_id: "base_scan"  # frame for the output LaserScan
```

### Launch

```python
Node(
    package='depthimage_to_laserscan',
    executable='depthimage_to_laserscan_node',
    name='depth_to_scan',
    remappings=[
        ('depth', '/camera/depth/image_rect_raw'),
        ('depth_camera_info', '/camera/depth/camera_info'),
        ('scan', '/camera/virtual_scan'),
    ],
    parameters=[depth_to_scan_params],
),
```

## depth_to_laserscan vs pointcloud_to_laserscan

| Aspect | depthimage_to_laserscan | pointcloud_to_laserscan |
|---|---|---|
| Input | Depth image | PointCloud2 |
| CPU cost | Low (operates on 2D image) | Higher (iterates 3D points) |
| Flexibility | Limited to camera FOV, single height band | Arbitrary height range, any PC source |
| Multi-sensor | Each camera needs its own node | Can merge point clouds first |
| Resolution | Tied to image width | Configurable angular increment |

Use `depthimage_to_laserscan` when you have a single depth camera and want minimal CPU overhead. Use `pointcloud_to_laserscan` when combining multiple depth sources or when you need precise height-based filtering.

## Frame Conventions

Depth cameras use the **optical frame** convention:
- Z axis: forward (into the scene)
- X axis: right
- Y axis: down

ROS robot convention (`base_link`):
- X axis: forward
- Y axis: left
- Z axis: up

The URDF must include a static transform from the camera's physical mounting frame to the optical frame. `depth_image_proc` nodes output points in the optical frame; downstream nodes (costmaps, `pointcloud_to_laserscan`) use TF to transform to `base_link`.

Typical URDF link chain:
```
base_link → camera_link → camera_depth_frame → camera_depth_optical_frame
```

The `camera_link` → `camera_depth_optical_frame` transform is a 90° rotation published by the camera driver or your URDF.

## Full Launch Pipeline

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='depth_pipeline',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='xyz_node',
                remappings=[
                    ('image_rect', '/camera/depth/image_rect_raw'),
                    ('camera_info', '/camera/depth/camera_info'),
                    ('points', '/camera/points'),
                ],
            ),
        ],
    )

    depth_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_scan',
        remappings=[
            ('depth', '/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/camera/scan'),
        ],
        parameters=[{
            'scan_height': 10,
            'range_min': 0.15,
            'range_max': 8.0,
            'output_frame_id': 'base_scan',
        }],
    )

    return LaunchDescription([container, depth_scan])
```

## Costmap Integration

Add the virtual scan as an observation source alongside the primary LIDAR:

```yaml
observation_sources: lidar depth_cam
depth_cam:
  topic: /camera/scan
  data_type: LaserScan
  marking: true
  clearing: true
  obstacle_max_range: 5.0
  obstacle_min_range: 0.15
  raytrace_max_range: 6.0
```

This gives the costmap obstacle data below the LIDAR plane (table legs, low furniture) that a 2D LIDAR alone would miss.
