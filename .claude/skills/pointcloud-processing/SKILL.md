<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: pointcloud-processing
description: 'Process PointCloud2 data in ROS 2. Use when filtering, downsampling, or segmenting 3D point clouds with PCL.'
---

# PointCloud2 Processing

## Binary Data Format

`sensor_msgs/msg/PointCloud2` stores 3D point data in a tightly packed binary buffer. Key fields:

| Field | Type | Description |
|---|---|---|
| `header` | Header | Frame and timestamp |
| `height` | uint32 | 1 for unorganized clouds; rows for organized (e.g., depth camera) |
| `width` | uint32 | Number of points per row |
| `fields[]` | PointField[] | Description of each channel (name, offset, datatype, count) |
| `is_bigendian` | bool | Byte order of the data |
| `point_step` | uint32 | Bytes per point |
| `row_step` | uint32 | Bytes per row (`point_step * width`) |
| `data` | uint8[] | The raw binary point data |
| `is_dense` | bool | `true` if no invalid points (no NaN/inf) |

Common field layouts:

- **XYZ**: fields `x`, `y`, `z` — each `FLOAT32`, `point_step` = 12 or 16 (with padding)
- **XYZRGB**: adds `rgb` field packed as a `FLOAT32` (actually 3 bytes: R, G, B)
- **XYZI**: adds `intensity` as `FLOAT32`

## Reading PointCloud2 in Python

```python
import sensor_msgs_py.point_cloud2 as pc2

def cloud_callback(msg):
    for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
        x, y, z = point
```

In C++, use `sensor_msgs::PointCloud2Iterator` or the `pcl_conversions` bridge to convert to `pcl::PointCloud<pcl::PointXYZ>`.

## PCL-ROS Filtering Pipeline

The `pcl_ros` package wraps PCL filters as ROS 2 nodelets/components. Key filters:

### VoxelGrid Downsampling

Divides space into voxels of size `leaf_size` and replaces all points in each voxel with their centroid. Reduces point count dramatically while preserving structure.

```yaml
voxel_grid:
  ros__parameters:
    leaf_size_x: 0.05
    leaf_size_y: 0.05
    leaf_size_z: 0.05
    input_topic: /camera/points
    output_topic: /camera/points_downsampled
```

### PassThrough Filter

Crops points to an axis-aligned bounding box. Remove floor, ceiling, or limit range:

```yaml
passthrough_z:
  ros__parameters:
    filter_field_name: "z"
    filter_limit_min: 0.1     # above ground plane
    filter_limit_max: 1.5     # below ceiling
    filter_limit_negative: false
```

### StatisticalOutlierRemoval

For each point, computes the mean distance to its K nearest neighbors. Points with mean distance beyond `stddev_mul * global_stddev` are removed. Effective against sparse noise.

```yaml
statistical_outlier:
  ros__parameters:
    mean_k: 50
    stddev_mul_thresh: 1.0
```

### CropBox

Similar to PassThrough but operates on a full 3D box with optional transform frame. Useful for removing the robot body from the point cloud:

```yaml
crop_box:
  ros__parameters:
    min_x: -0.3
    min_y: -0.3
    min_z: -0.1
    max_x: 0.3
    max_y: 0.3
    max_z: 0.5
    negative: true            # remove points INSIDE the box
    input_frame: base_link
```

## pointcloud_to_laserscan

Projects a 3D point cloud into a virtual 2D `LaserScan` for consumption by Nav2 costmaps. Each angular bin takes the nearest (or smallest Z within height range) point.

### Configuration

```yaml
pointcloud_to_laserscan_node:
  ros__parameters:
    target_frame: base_link       # transform cloud to this frame first
    transform_tolerance: 0.01
    min_height: 0.05              # ignore points below (ground)
    max_height: 1.0               # ignore points above (ceiling/irrelevant)
    angle_min: -3.14159           # full 360° or restrict to FOV
    angle_max: 3.14159
    angle_increment: 0.00436      # ~0.25° resolution
    scan_time: 0.1
    range_min: 0.15
    range_max: 10.0
    use_inf: true                 # emit inf for empty bins
    inf_epsilon: 1.0
```

### Launch Example

```python
Node(
    package='pointcloud_to_laserscan',
    executable='pointcloud_to_laserscan_node',
    name='pc_to_scan',
    remappings=[
        ('cloud_in', '/camera/points_filtered'),
        ('scan', '/camera/scan'),
    ],
    parameters=[pointcloud_to_laserscan_params],
),
```

## Frame Considerations

Point clouds from depth cameras arrive in the camera's optical frame (Z-forward, X-right, Y-down). The `pointcloud_to_laserscan` node uses TF to transform to `target_frame` (usually `base_link`) before projection. Ensure TF is published between `camera_depth_optical_frame` → `base_link` via your URDF static transforms.

## Integration with Nav2

After projection to a virtual scan, add it as an observation source in the costmap:

```yaml
observation_sources: lidar_scan depth_scan
depth_scan:
  topic: /camera/scan
  data_type: LaserScan
  marking: true
  clearing: true
  obstacle_max_range: 5.0
  obstacle_min_range: 0.15
```

For direct point cloud sources (without `pointcloud_to_laserscan`), use `data_type: PointCloud2` in the costmap observation source and configure `min_obstacle_height`/`max_obstacle_height`.
