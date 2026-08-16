<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: slam-toolbox-lifelong
description: 'Configure SLAM Toolbox lifelong mode. Use when updating maps over time, setting map update/trim parameters, or persistent mapping.'
---

# SLAM Toolbox Lifelong Mode

## Overview

Lifelong mode (`slam_toolbox::LifelongSlamToolbox`) is designed for persistent robots that run for days or weeks. It starts from a serialized map, continues mapping, and **trims old nodes** to prevent unbounded graph growth. The map evolves over time—new obstacles appear, old ones are removed.

## When to Use Lifelong vs Online

| Scenario | Mode |
|----------|------|
| First-time mapping of a new environment | Online sync/async |
| Robot runs 8+ hours continuously | Lifelong |
| Environment changes frequently (furniture moves) | Lifelong |
| Need to update a saved map periodically | Lifelong |
| One-shot mapping, then switch to Nav2 AMCL | Online → serialize → stop |

## Plugin Declaration

```yaml
slam_toolbox:
  ros__parameters:
    plugin: slam_toolbox::LifelongSlamToolbox
```

## Lifelong-Specific Behavior

### Graph Trimming

The key difference from online mode: lifelong mode monitors graph size and removes old, low-information nodes when the graph exceeds internal thresholds. This prevents:
- Memory growth proportional to runtime
- Solver slowdown from an ever-growing pose graph
- Stale map data from persisting indefinitely

Trimming is automatic. Nodes are prioritized for removal based on how much information they contribute (scan match quality, constraint count, recency).

### Map Evolution

When the robot revisits an area, lifelong mode:
1. Matches the current scan against existing nodes in the area.
2. If the environment has changed, the old node's scan data is replaced.
3. The graph is re-optimized with the updated data.
4. The published map reflects the new reality.

This means walls that were removed, furniture that moved, or doors that were opened will eventually be reflected in the map.

## Serialized Map Loading (Deserialization)

Lifelong mode typically starts from a previously saved map.

### File Pair

SLAM Toolbox serializes to two files:
- `my_map.posegraph` — The pose graph structure (nodes, edges, constraints).
- `my_map.data` — Scan data associated with each node.

Both must be present in the same directory with the same base name.

### Configuration for Deserialization

```yaml
map_file_name: /home/robot/maps/house_map    # Base name, no extension.
map_start_pose: [0.0, 0.0, 0.0]              # Initial pose in the loaded map.
map_start_at_dock: true                       # true = use map_start_pose as initial pose.
```

On startup, the node loads the graph, publishes the map, and begins matching incoming scans against it.

## Complete Lifelong YAML

```yaml
slam_toolbox:
  ros__parameters:
    # Plugin
    plugin: slam_toolbox::LifelongSlamToolbox

    # Frames & topics
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    use_sim_time: false

    # Map
    resolution: 0.05
    max_laser_range: 8.0
    minimum_time_interval: 0.5
    map_update_interval: 5.0
    transform_publish_period: 0.02

    # Motion thresholds
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5

    # Scan matching
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Loop closure
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03
    loop_search_maximum_distance: 3.0
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Solver
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # Deserialization
    map_file_name: /home/robot/maps/house_map
    map_start_pose: [0.0, 0.0, 0.0]
    map_start_at_dock: true

    # Misc
    stack_size_to_use: 40000000
    max_queue_size: 10             # Max queued scans before dropping. Prevents buildup during heavy optimization.
    enable_interactive_mode: true
    debug_logging: false
    throttle_scans: 1
    tf_buffer_duration: 30.0
    mode: mapping
```

## Workflow: Initial Mapping → Serialize → Lifelong

### Step 1: Create Initial Map (Online Mode)

```bash
ros2 launch slam_toolbox online_sync_launch.py params_file:=online_params.yaml
# Drive the robot through the entire environment.
```

### Step 2: Serialize via RViz2 Plugin or Service

```bash
# Using the service:
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/robot/maps/house_map'}"
```

This writes `house_map.posegraph` and `house_map.data`.

### Step 3: Switch to Lifelong Mode

Update your launch to use `LifelongSlamToolbox` and point `map_file_name` to the serialized map. Restart the node.

### Step 4: Ongoing Operation

The robot patrols, and the map updates as the environment changes. Periodically re-serialize to save the latest state:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/robot/maps/house_map_$(date +%Y%m%d)'}"
```

## Interactive RViz2 Plugin

The SLAM Toolbox RViz2 panel (add via Panels → Add New Panel → SlamToolboxPlugin) provides:

| Button | Action |
|--------|--------|
| Serialize Map | Save current graph to disk |
| Deserialize Map | Load a graph from disk |
| Clear Changes | Revert to the last deserialized state |
| Save Map | Export the current map as an OccupancyGrid image (PGM) |
| Continue Mapping | Resume adding scans to the loaded graph |
| Clear | Clear the entire graph (destructive) |

## Common Issues

- **Long startup time**: Deserializing a large graph (>5000 nodes) can take 10-30 seconds. The node won't publish TF until complete.
- **Drift after long runtime**: Even with loop closure, persistent drift can accumulate. Periodic serialization and clean restart helps.
- **stack_size_to_use**: If the node crashes during optimization on large graphs, increase to 80000000 (80MB).
- **Stale map regions**: If the robot never revisits an area, lifelong mode may trim those nodes. Ensure patrol routes cover all critical areas.
