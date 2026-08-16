<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: wall-line-extraction
description: 'Extract wall lines from lidar data. Use when detecting walls, fitting lines to scan segments, or building geometric maps.'
---

# Wall Line Extraction

## Purpose

Extracting wall lines from LaserScan data enables higher-level reasoning: room boundary detection, door identification, map feature extraction, and navigation reference alignment. Raw scan points are noisy and unstructured; line extraction produces clean geometric primitives.

## Algorithms

### Split-and-Merge

Recursive line fitting. Fast and simple, well-suited for structured indoor environments.

**Algorithm**:

```
function SplitAndMerge(points, threshold):
    if len(points) < 2:
        return []

    # Fit a line from first to last point
    line = Line(points[0], points[-1])

    # Find the point farthest from the line
    max_dist = 0
    split_idx = -1
    for i in range(1, len(points) - 1):
        d = perpendicular_distance(points[i], line)
        if d > max_dist:
            max_dist = d
            split_idx = i

    if max_dist > threshold:
        # Recursively split
        left = SplitAndMerge(points[0:split_idx+1], threshold)
        right = SplitAndMerge(points[split_idx:], threshold)
        return left + right
    else:
        # All points fit the line within threshold
        return [line]
```

After splitting, a **merge** pass combines collinear adjacent segments whose gap is below a threshold.

**Tuning**: `threshold` (perpendicular distance) controls sensitivity. Indoor walls: 0.02-0.05m. Larger values produce fewer, longer segments.

### Hough Transform

Voting-based method. Each scan point votes for all lines that could pass through it, parameterized in (r, θ) space where `r = x·cos(θ) + y·sin(θ)`.

**Steps**:
1. Discretize (r, θ) parameter space into an accumulator array
2. For each point (x, y), increment all cells (r, θ) satisfying the line equation
3. Find peaks in the accumulator — each peak is a line
4. Group points belonging to each line and extract endpoints

**Pros**: Robust to outliers and gaps. Finds lines even with missing segments.
**Cons**: Discrete resolution limits accuracy. Computationally heavier than Split-and-Merge. Difficult to extract exact endpoints.

### RANSAC (Random Sample Consensus)

**Steps**:
1. Randomly select 2 points, fit a line
2. Count inliers (points within distance threshold of the line)
3. Repeat K times, keep the line with the most inliers
4. Refine the best line using all its inliers (least squares)
5. Remove inliers from the point set, repeat for the next line

**Pros**: Very robust to outliers. Produces accurate line parameters.
**Cons**: Non-deterministic. May miss short segments. Requires tuning: number of iterations K, distance threshold, minimum inliers.

## Comparison

| Aspect | Split-and-Merge | Hough Transform | RANSAC |
|---|---|---|---|
| Speed | Fast (O(n log n)) | Moderate (O(n·θ_bins)) | Variable |
| Outlier robustness | Low | High | Very high |
| Endpoint extraction | Natural | Requires post-processing | Requires post-processing |
| Deterministic | Yes | Yes | No |
| Best for | Clean indoor scans | Noisy/cluttered scenes | Moderate noise |

## laser_line_extraction ROS Package

The `laser_line_extraction` package implements line extraction from `LaserScan` messages and publishes `visualization_msgs/MarkerArray` for RViz and a custom `LineSegmentList` message.

### Parameters

```yaml
line_extraction_node:
  ros__parameters:
    frequency: 25.0
    frame_id: "laser_frame"
    scan_topic: "/scan"
    min_line_length: 0.5      # meters — ignore very short segments
    min_line_points: 9         # minimum points to form a line
    max_line_gap: 0.4          # meters — merge lines with gaps smaller than this
    min_range: 0.15            # ignore readings closer than this
    max_range: 10.0            # ignore readings farther than this
    min_split_dist: 0.05       # split threshold for Split-and-Merge (meters)
    outlier_dist: 0.05         # distance to exclude outlier points from line fitting
    bearing_var: 1.0e-5        # measurement bearing variance
    range_var: 0.012           # measurement range variance
```

### Output Topics

| Topic | Type | Description |
|---|---|---|
| `/line_segments` | `LineSegmentList` | Extracted line segments with start/end points |
| `/line_markers` | `MarkerArray` | Visualization markers for RViz |

### Launch

```python
Node(
    package='laser_line_extraction',
    executable='line_extraction_node',
    name='line_extraction',
    parameters=[line_extraction_params],
    remappings=[('scan', '/scan')],
),
```

## Applications

### Door Detection

Doors appear as gaps between collinear wall segments. Detect doors by finding pairs of line segment endpoints that are:
- Close together (gap < door width, typically 0.7-1.0m)
- From segments that are approximately collinear (angle difference < 10°)

### Room Segmentation

Connected wall lines form closed or semi-closed polygons. Room detection groups lines into rooms based on enclosure analysis.

### Navigation Reference

Align the robot to detected wall lines for precise corridor-following. Compute the robot's lateral offset and yaw relative to the nearest wall segment, and feed this to a controller.

## Implementation Notes

- Filter the scan before line extraction (remove chassis hits, speckle noise)
- Line extraction works best with dense, low-noise scans (e.g., Hokuyo, SICK)
- For sparse LIDAR (e.g., LD19 with 4000 points/360°), increase `min_line_points` to reduce false segments
- Run line extraction at scan frequency or lower; it is computationally cheap
- Lines are in the LIDAR frame; transform endpoints to `map` frame via TF for map-level reasoning
