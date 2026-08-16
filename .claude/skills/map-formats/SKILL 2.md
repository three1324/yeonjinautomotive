<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: map-formats
description: 'Understand ROS 2 map formats. Use when working with PGM/PNG occupancy grids, YAML metadata, or map resolution/origin.'
---

# Map File Formats in ROS 2

## Overview

ROS 2 maps are stored as a **YAML metadata file** paired with a **grayscale image file**. The image encodes occupancy information, and the YAML describes how to interpret it. SLAM Toolbox also uses a proprietary serialized format for storing the full pose graph.

## The YAML + Image Pair

### YAML Metadata File

```yaml
image: my_map.pgm
resolution: 0.050000
origin: [-12.200000, -10.700000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
mode: trinary
```

### Image File Encoding (Standard Convention)

In the **standard** (non-negated) convention, pixel grayscale values map to occupancy:

| Pixel Value | Meaning | OccupancyGrid Value |
|------------|---------|-------------------|
| 0 (black) | Occupied | 100 |
| 205 (light gray) | Free | 0 |
| 254 (near white) | Unknown | -1 |

The mapping from pixel value `p` to occupancy probability `occ`:
```
occ = (255 - p) / 255.0    (when negate=0)
occ = p / 255.0              (when negate=1)
```

## PGM Format

**Portable GrayMap (PGM)** is the default format. It's an uncompressed grayscale image format.

### Structure

```
P5                    # Magic number: P5 = binary PGM, P2 = ASCII PGM.
# Comment line
800 600               # Width Height (pixels).
255                   # Max grayscale value.
<binary pixel data>   # Width × Height bytes, row-major, top to bottom.
```

### Advantages
- No compression artifacts—pixel values are exact.
- Simple format, universally supported.
- Fast to read (no decompression).

### Disadvantages
- Large file size: an 800×600 map = 480KB uncompressed.
- No transparency channel.

## PNG Format

**Portable Network Graphics (PNG)** uses lossless compression.

```bash
# Save as PNG:
ros2 run nav2_map_server map_saver_cli -f my_map --image-format png
```

### Advantages
- Lossless compression: typically 60-80% smaller than PGM.
- Widely supported by editors.

### Disadvantages
- Slightly slower to load (decompression required).
- Some tools may add metadata that confuses simple parsers (rare).

## NEVER Use JPEG

**JPEG uses lossy compression.** It introduces compression artifacts that change pixel values, destroying the occupancy encoding. A pixel that should be `0` (occupied) might become `3` or `5`, and a pixel at `205` (free) might become `202` or `208`. This corrupts thresholds and creates phantom obstacles or missing walls.

**Rule: Only PGM or PNG for robot maps. Never JPEG, WebP, or any lossy format.**

## The `negate` Field

Controls how pixel brightness maps to occupancy:

| negate | Black pixels | White pixels |
|--------|-------------|-------------|
| 0 (default) | Occupied (high probability) | Free (low probability) |
| 1 | Free (low probability) | Occupied (high probability) |

Use `negate: 1` when your source image uses the inverted convention (e.g., white = walls). Most SLAM outputs use the standard convention, so `negate: 0` is typical.

## Resolution

Meters per pixel. Common values:

| Resolution | Meaning | Typical Use |
|-----------|---------|-------------|
| 0.01 | 1cm/pixel | High-detail local maps |
| 0.025 | 2.5cm/pixel | High-quality indoor maps |
| 0.05 | 5cm/pixel | Standard indoor (most common) |
| 0.1 | 10cm/pixel | Large buildings, outdoor |

A 20m × 20m room at 0.05 resolution = 400 × 400 pixels = 160KB (PGM).

## Origin

`[x, y, yaw]` — the pose of the **bottom-left pixel** of the image in the map coordinate frame.

- **x, y**: Position in meters.
- **yaw**: Rotation in radians (almost always 0.0).

The origin is typically a negative value because SLAM places the robot's starting position near the map center, and the bottom-left corner of the image is offset in the negative x/y direction.

```
Map frame:
  origin at robot start → (0, 0)
  Bottom-left of image → (origin_x, origin_y)
```

## Three Interpretation Modes

### Trinary (default)

Pixels are classified into exactly three states:

```python
if occ >= occupied_thresh:
    cell = 100  # occupied
elif occ <= free_thresh:
    cell = 0    # free
else:
    cell = -1   # unknown
```

Most navigation stacks expect trinary maps.

### Scale

Continuous probability values. The pixel value maps linearly to occupancy [0, 100]:

```python
cell = int(occ * 100)  # 0-100, no unknown
```

Unknown is only assigned to pixels with exact value 205 (if negate=0) — or the magic "unknown" value.

### Raw

The byte value is used directly. Values 0-100 map to occupancy probability 0-100. Value 101-254 maps to "unknown" or special values per application.

## Editing Maps in GIMP

1. Open the PGM/PNG file in GIMP (File → Open).
2. Ensure Image → Mode → Grayscale.
3. Use the **pencil tool** (not brush—pencil has hard edges) with:
   - Black (0) to add walls.
   - Light gray (~205) to mark free space.
   - Near-white (~254) to mark unknown.
4. Remove artifacts outside the building boundary by flood-filling with unknown (254).
5. Fix small gaps in walls by drawing with black.
6. Export as PGM: File → Export As → select PGM → Raw encoding.
7. Or export as PNG: File → Export As → select PNG → no interlacing.

**Warning**: GIMP's "Save" creates `.xcf` files. Always use "Export As" for PGM/PNG.

## SLAM Toolbox Serialized Format

SLAM Toolbox stores the full pose graph in two binary files:

| File | Contents |
|------|----------|
| `map_name.posegraph` | Graph structure: nodes (poses), edges (constraints), covariances |
| `map_name.data` | Laser scan data associated with each node |

These files are **not human-readable or editable**. They preserve the full SLAM state, enabling:
- Continued mapping (lifelong mode).
- Localization-only mode (scan match against the graph).
- Graph manipulation via the RViz2 plugin.

To convert a serialized graph to a standard PGM map, use the map saver while the SLAM node is running with the deserialized graph.

## File Size Reference

For a typical 30m × 30m house at 0.05 resolution (600 × 600 pixels):

| Format | Approximate Size |
|--------|-----------------|
| PGM | 360 KB |
| PNG | 60-120 KB |
| SLAM Toolbox .posegraph + .data | 2-20 MB (depends on node count) |
