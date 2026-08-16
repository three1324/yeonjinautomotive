<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: slam-toolbox-online
description: 'Configure SLAM Toolbox online mode. Use when running real-time SLAM, setting scan matching, loop closure, or solver parameters.'
---

# SLAM Toolbox Online Synchronous Mode

## Overview

SLAM Toolbox's online synchronous mode performs real-time SLAM by processing each laser scan as it arrives, building a pose graph, and publishing the map. The plugin `slam_toolbox::OnlineSyncSlamToolbox` runs as a lifecycle-managed node compatible with Nav2.

## Plugin Declaration

```yaml
slam_toolbox:
  ros__parameters:
    plugin: slam_toolbox::OnlineSyncSlamToolbox
```

There is also `OnlineAsyncSlamToolbox` which queues scans and processes them in a background thread—useful when CPU cannot keep up, but the map may lag behind reality.

## Frame and Topic Configuration

```yaml
odom_frame: odom              # Odometry frame published by your robot.
map_frame: map                # Frame the map is published in.
base_frame: base_link         # Robot base frame for TF lookups.
scan_topic: /scan             # LaserScan topic name.
use_sim_time: false
```

The node publishes the `map → odom` transform. Your robot must already publish `odom → base_link`.

## Core Mapping Parameters

```yaml
resolution: 0.05              # Map cell size in meters. 0.05 = 5cm per pixel.
max_laser_range: 8.0          # Max usable range (m). Clip beyond this.
minimum_time_interval: 0.5    # Minimum seconds between processed scans. Reduces CPU load.
map_update_interval: 5.0      # Seconds between /map topic publications.
transform_publish_period: 0.02  # Seconds between TF broadcasts of map→odom.
```

### Scan Processing Control

```yaml
minimum_travel_distance: 0.5  # Robot must move this far (m) before adding a new scan to the graph.
minimum_travel_heading: 0.5   # Robot must rotate this much (rad) before adding a new scan.
```

These prevent graph bloat when the robot is stationary or barely moving.

## Solver Configuration

SLAM Toolbox uses Google Ceres Solver for graph optimization.

```yaml
solver_plugin: solver_plugins::CeresSolver
ceres_linear_solver: SPARSE_NORMAL_CHOLESKY   # SPARSE_NORMAL_CHOLESKY or DENSE_QR.
ceres_preconditioner: SCHUR_JACOBI             # Preconditioner for iterative solvers.
ceres_trust_strategy: LEVENBERG_MARQUARDT      # Or DOGLEG.
ceres_dogleg_type: TRADITIONAL_DOGLEG          # Only if trust_strategy is DOGLEG.
ceres_loss_function: None                      # None, HuberLoss, CauchyLoss.
```

For large maps (>1000 nodes), `SPARSE_NORMAL_CHOLESKY` is essential for performance.

## Scan Matching Parameters

### Correlation Scan Matcher (local matching)

The correlation scan matcher finds the best alignment between the current scan and the local map using a brute-force grid search.

```yaml
correlation_search_space_dimension: 0.5        # Search window size (m). Larger = more robust but slower.
correlation_search_space_resolution: 0.01      # Resolution of the search grid (m).
correlation_search_space_smear_deviation: 0.1  # Gaussian smear applied to the search space.
```

### Loop Closure Scan Matcher (global matching)

```yaml
loop_search_space_dimension: 8.0               # Search window for loop closure candidates (m).
loop_search_space_resolution: 0.05             # Resolution of loop closure search grid.
loop_search_space_smear_deviation: 0.03        # Smear for loop closure search.
loop_search_maximum_distance: 3.0              # Max distance between candidate nodes for loop closure.
```

## Loop Closure Control

```yaml
do_loop_closing: true                          # Enable or disable loop closure detection.
loop_match_minimum_chain_size: 10              # Minimum chain of connected nodes before considering loop closure.
loop_match_maximum_variance_coarse: 3.0        # Variance threshold for coarse loop matching.
loop_match_minimum_response_coarse: 0.35       # Minimum response from coarse matcher to proceed.
loop_match_minimum_response_fine: 0.45         # Minimum response from fine matcher to accept closure.
```

**Local vs Global Matching**: The correlation scan matcher aligns each new scan against the nearest submap for local pose refinement. Loop closure operates globally—when the robot revisits an area, the loop matcher compares the current scan against distant nodes in the graph. A successful loop closure triggers a full graph optimization redistributing accumulated drift.

## Stack Size

```yaml
stack_size_to_use: 40000000    # 40MB. Ceres solver can use deep recursion on large graphs.
```

If the node segfaults on large maps, increase this value.

## Interactive Mode

```yaml
enable_interactive_mode: true
```

Enables the SLAM Toolbox RViz2 plugin. With it you can:
- Serialize the current graph to disk (`.posegraph` + `.data` files).
- Deserialize a saved graph to continue mapping.
- Manually manipulate the graph (add/remove nodes) via the RViz panel.

## Complete Online Sync YAML

```yaml
slam_toolbox:
  ros__parameters:
    # Plugin
    plugin: slam_toolbox::OnlineSyncSlamToolbox

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

    # Misc
    stack_size_to_use: 40000000
    enable_interactive_mode: true
    debug_logging: false
    throttle_scans: 1
    tf_buffer_duration: 30.0
    map_file_name: ""           # Empty = don't load a prior map.
    map_start_pose: [0.0, 0.0, 0.0]
    map_start_at_dock: true
    mode: mapping               # "mapping" or "localization".
```

## Switching to Localization Mode

After mapping, serialize the graph, then restart with:

```yaml
mode: localization
map_file_name: /path/to/my_map   # Without extension. Loads .posegraph + .data.
map_start_pose: [0.0, 0.0, 0.0]
```

In localization mode, SLAM Toolbox performs scan matching against the loaded graph without adding new nodes.
