<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: map-merging
description: 'Merge multiple maps. Use when combining maps from different sessions, robots, or partial mapping runs.'
---

# Map Merging Techniques

## Overview

Map merging combines maps from multiple sessions, robots, or partial scans into a single coherent map. This is necessary when a single mapping run cannot cover the entire environment, or when multiple robots map simultaneously. There is no one-size-fits-all solution—the approach depends on whether you have pose graph data, overlapping regions, or only raw images.

## Approach 1: SLAM Toolbox Deserialization + Continue Mapping

The most robust approach when using SLAM Toolbox. You serialize a partial map, then start a new session that loads the serialized graph and continues mapping.

### Workflow

```bash
# Session 1: Map area A
ros2 launch slam_toolbox online_sync_launch.py params_file:=params.yaml
# Drive through area A, then serialize:
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/robot/maps/area_a'}"

# Session 2: Continue into area B
# Configure SLAM Toolbox to load the serialized graph:
```

```yaml
slam_toolbox:
  ros__parameters:
    map_file_name: /home/robot/maps/area_a    # Base name, no extension.
    map_start_pose: [0.0, 0.0, 0.0]
    map_start_at_dock: true
    mode: mapping
```

```bash
# Start SLAM Toolbox with the above config.
# Drive through area B. The graph now contains A + B.
# Serialize the combined graph:
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/robot/maps/area_ab'}"
```

### Requirements
- The robot must start session 2 at a location within the area covered by session 1's map.
- SLAM Toolbox must successfully match the initial scans against the loaded graph.
- If the robot starts outside the mapped area, merging fails silently (no constraints connect old and new nodes).

### Verifying the Merge
After deserialization + continued mapping, check the constraint visualization in RViz. There should be constraints connecting old (area A) nodes to new (area B) nodes at the overlap region.

## Approach 2: multirobot_map_merge Package

The `multirobot_map_merge` package merges OccupancyGrid maps from multiple sources using feature matching on the map images.

```bash
sudo apt install ros-jazzy-multirobot-map-merge   # If available for Jazzy.
```

### Configuration

```yaml
map_merge:
  ros__parameters:
    robot_map_topic: map
    robot_namespace: ''
    merged_map_topic: /merged_map
    world_frame: map
    known_init_poses: true        # true if you know the relative poses of the robots/sessions.
    init_pose_x_0: 0.0
    init_pose_y_0: 0.0
    init_pose_yaw_0: 0.0
    init_pose_x_1: 10.0           # Robot 2 starts 10m east of robot 1.
    init_pose_y_1: 0.0
    init_pose_yaw_1: 0.0
    merging_rate: 0.5             # Hz. How often to attempt merging.
    estimation_rate: 0.5          # Hz for transform estimation (if known_init_poses=false).
    estimation_confidence: 1.0    # Minimum confidence for automatic alignment.
```

### Known vs Unknown Initial Poses

- **known_init_poses: true**: You provide the relative starting positions of each robot/session. The package directly overlays the maps using these transforms. More reliable.
- **known_init_poses: false**: The package attempts to find the alignment automatically using feature matching on the occupancy grids. Works if there is sufficient overlap (~30%) and distinctive features. Often fails in symmetric or featureless environments.

### Limitations
- Only merges 2D OccupancyGrids—no pose graph, so no correction of internal drift.
- Requires significant overlap between maps for automatic alignment.
- Quality depends on individual map quality. Distorted maps produce distorted merges.

## Approach 3: Manual Image Merging

For simple cases, manually aligning map images in an editor can be effective.

### Steps

1. Open both PGM/PNG maps in GIMP or similar.
2. Identify overlapping features (walls, doorways).
3. Align the second map over the first using translate/rotate.
4. Merge layers: for each pixel, take the most informative value (occupied > free > unknown).
5. Export as PGM.
6. Manually create the YAML metadata file with the correct origin and resolution.

### Resolution Matching

**Both maps must have the same resolution.** If they differ, rescale one:

```bash
# In GIMP: Image → Scale Image → set width/height to match the resolution ratio.
# Or use ImageMagick:
convert map_b.pgm -resize 200% map_b_scaled.pgm   # If map_b is 0.1m and map_a is 0.05m.
```

### Origin Alignment

After merging, the origin must be recalculated. The new origin is the bottom-left corner of the merged image in map coordinates:

```python
# If map_a origin is (-12.2, -10.7) and map_b was shifted by (15.0, 0.0):
# New origin depends on which map's bottom-left is further negative.
new_origin_x = min(origin_a_x, origin_b_x + shift_x)
new_origin_y = min(origin_a_y, origin_b_y + shift_y)
```

## Approach 4: Cartographer Map Merging

Cartographer supports multi-trajectory mapping natively. You can add a second trajectory that shares the same pose graph:

```bash
# Start trajectory 1 for robot 1:
ros2 service call /start_trajectory cartographer_ros_msgs/srv/StartTrajectory ...

# Start trajectory 2 for robot 2 (or second session):
ros2 service call /start_trajectory cartographer_ros_msgs/srv/StartTrajectory ...
```

Both trajectories contribute to the same global optimization. The constraint builder finds inter-trajectory loop closures where the trajectories overlap.

## Overlapping Region Requirements

For any automated merging to work:

| Method | Minimum Overlap |
|--------|----------------|
| SLAM Toolbox deserialization | Robot must start within mapped area |
| multirobot_map_merge (auto) | ~30% shared area with distinctive features |
| multirobot_map_merge (known poses) | Any overlap |
| Cartographer multi-trajectory | Physical overlap with loop closure opportunity |
| Manual (GIMP) | Visual landmarks to align |

## Map Georeferencing

For outdoor or multi-building scenarios, maps can be aligned to a global coordinate system (GPS/UTM):

1. Record GPS coordinates at known points during mapping.
2. Compute the affine transform from map coordinates to UTM.
3. Store the transform in the map metadata or a separate config.

This enables merging maps from different buildings or areas by aligning them in the shared global frame.

## Best Practices

- **Always serialize the full SLAM graph**, not just the image. Images lose internal pose graph structure.
- **Test merges incrementally**: merge two maps, verify, then add a third.
- **Prefer graph-based merging** (SLAM Toolbox deserialization, Cartographer multi-trajectory) over image-based when possible.
- **Validate with loop closures**: after merging, drive the robot through the overlap region to confirm the merge is geometrically correct.
- **Version your maps**: keep dated copies of serialized graphs so you can revert a bad merge.
