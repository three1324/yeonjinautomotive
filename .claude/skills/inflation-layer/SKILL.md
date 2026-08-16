<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: inflation-layer
description: 'Tune Nav2 inflation layer. Use when adjusting cost_scaling_factor, inflation_radius, or understanding the exponential decay cost formula.'
---

# InflationLayer — Deep Tuning Knowledge

## Purpose

InflationLayer creates a cost gradient radiating outward from each lethal (254) obstacle cell. This gradient biases planners and controllers to keep distance from obstacles without making nearby cells impassable.

## MUST Be the Last Plugin

InflationLayer reads all lethal and inscribed cells currently in the costmap, then generates the gradient. Any layer added AFTER inflation overwrites the gradient. This is the most common costmap misconfiguration.

## The Two Key Parameters

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  inflation_radius: 0.55      # meters — max distance from obstacle where cost > 0
  cost_scaling_factor: 3.0     # exponential decay rate
```

## The Cost Formula

```
cost(d) = 253 * exp(-cost_scaling_factor * (d - inscribed_radius))
```

Where:
- `d` = distance from the nearest lethal cell (meters)
- `inscribed_radius` = radius of the largest circle fitting inside the robot footprint
- At `d = inscribed_radius`: cost = 253 (inscribed cost — robot center here means guaranteed collision)
- At `d > inscribed_radius`: cost decays exponentially toward 0
- At `d >= inflation_radius`: cost = 0

## The Inverse Relationship

This is the most counterintuitive aspect:

**Higher cost_scaling_factor → STEEPER decay → robot navigates CLOSER to walls**
**Lower cost_scaling_factor → GENTLER decay → robot stays FURTHER from walls**

Why? With a steep decay, the cost drops to near-zero quickly. The planner sees low cost close to walls and routes there. With a gentle decay, significant cost extends further from walls, pushing paths toward the center of corridors.

## Cost Curves at Different Scaling Factors

Assuming `inscribed_radius = 0.18m`, `inflation_radius = 0.55m`:

| Distance from obstacle | factor=2.0 | factor=3.0 | factor=5.0 | factor=10.0 |
|---|---|---|---|---|
| 0.18m (inscribed) | 253 | 253 | 253 | 253 |
| 0.20m | 243 | 238 | 228 | 206 |
| 0.25m | 220 | 204 | 176 | 119 |
| 0.30m | 199 | 174 | 134 | 69 |
| 0.35m | 180 | 149 | 102 | 40 |
| 0.40m | 163 | 128 | 78 | 23 |
| 0.45m | 148 | 109 | 59 | 14 |
| 0.50m | 134 | 93 | 45 | 8 |
| 0.55m | 0 | 0 | 0 | 0 |

At `factor=2.0`, cost at 0.35m is still 180 — the planner strongly avoids this zone.
At `factor=10.0`, cost at 0.35m is only 40 — the planner barely notices it.

## Inscribed and Circumscribed Radii

These are computed from `robot_radius` (circle) or `footprint` (polygon):

- **Inscribed radius**: Largest circle fitting entirely inside the footprint. If the robot center is within this distance of a lethal cell, collision is guaranteed. Cost = 253 (inscribed cost).
- **Circumscribed radius**: Smallest circle enclosing the entire footprint. If the robot center is within this distance, collision is possible (depends on orientation). Planners use this for initial feasibility checks.

```
For robot_radius: 0.18m
  inscribed_radius = 0.18m
  circumscribed_radius = 0.18m

For footprint: [[0.20, 0.15], [-0.20, 0.15], [-0.20, -0.15], [0.20, -0.15]]
  inscribed_radius = 0.15m  (min dimension)
  circumscribed_radius = 0.25m  (diagonal)
```

## Tuning Guidelines

### Indoor differential drive (slow, narrow corridors)
```yaml
inflation_radius: 0.40
cost_scaling_factor: 2.5
```
Moderate buffer. Gentle gradient keeps robot centered in corridors. Won't squeeze through narrow gaps unless necessary.

### Indoor differential drive (needs to navigate tight spaces)
```yaml
inflation_radius: 0.30
cost_scaling_factor: 5.0
```
Small buffer with steep decay. Robot can navigate close to walls and through doorways. Less margin for error.

### General-purpose indoor (recommended starting point)
```yaml
inflation_radius: 0.55
cost_scaling_factor: 3.0
```
Good balance. Enough inflation for corridor centering, fast enough decay to not block narrow passages.

## Common Mistakes

1. **inflation_radius too small (< inscribed_radius + 0.1m)**: The planner sees almost no soft cost gradient. It plans paths that technically avoid collision but leave zero safety margin. The controller may oscillate trying to follow these paths.

2. **inflation_radius too large (> 1.5m indoor)**: Large inflation zones overlap in corridors, creating high costs everywhere. The planner struggles to find any "good" path and may fail or produce suboptimal routes.

3. **cost_scaling_factor too high (> 15)**: The gradient is essentially a step function. Without a smooth gradient, the planner cannot distinguish between "barely safe" and "comfortably safe" paths.

4. **Not last in plugins list**: If any layer writes after inflation, it overwrites the gradient in those cells. The inflation effect is lost for those regions.

## Multiple Inflation Layers

It is possible (rarely needed) to have different inflation parameters for different costmaps:

```yaml
# Global: gentle inflation for smooth global paths
global_costmap:
  ...
  inflation_layer:
    inflation_radius: 0.55
    cost_scaling_factor: 2.5

# Local: tighter inflation for reactive maneuvering
local_costmap:
  ...
  inflation_layer:
    inflation_radius: 0.40
    cost_scaling_factor: 5.0
```

This gives the global planner wider berth from walls while letting the local controller navigate closer when needed.
