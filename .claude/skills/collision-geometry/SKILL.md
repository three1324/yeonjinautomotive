<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: collision-geometry
description: 'Design URDF collision geometry. Use when simplifying collision shapes, combining primitives, or matching Nav2 footprint to URDF.'
---

# Collision Geometry Best Practices

## Collision ≠ Visual

The `<collision>` element defines what the physics engine and collision checker use. The `<visual>` element is only for rendering. They should often be **different**:

```xml
<link name="sensor_housing">
  <!-- Detailed mesh for rendering -->
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/sensor_housing.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
  <!-- Simple box for collision -->
  <collision>
    <geometry>
      <box size="0.08 0.06 0.04"/>
    </geometry>
    <origin xyz="0 0 0.02"/>
  </collision>
</link>
```

## Why Simplify Collision Geometry

- **Performance**: collision detection with triangle meshes is O(n²) at worst; primitives are O(1)
- **Stability**: complex concave meshes cause physics solver failures and tunneling
- **Predictability**: simple shapes produce consistent contact forces

## Primitive Shapes

Available primitives in URDF:

```xml
<geometry><box size="0.3 0.2 0.1"/></geometry>           <!-- x y z dimensions -->
<geometry><cylinder radius="0.05" length="0.03"/></geometry>  <!-- along z-axis -->
<geometry><sphere radius="0.025"/></geometry>
<geometry><mesh filename="package://pkg/meshes/part.stl"/></geometry>
```

**Prefer primitives over meshes for collision.** If a mesh is needed, use a **convex hull** (not the original concave mesh):

```bash
# Generate convex hull from mesh (using trimesh or similar)
python3 -c "
import trimesh
mesh = trimesh.load('part.stl')
hull = mesh.convex_hull
hull.export('part_convex.stl')
"
```

## Multiple Collision Elements Per Link

Approximate complex shapes by combining primitives in a single link:

```xml
<link name="l_shaped_bracket">
  <collision>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry><box size="0.02 0.1 0.1"/></geometry>
  </collision>
  <collision>
    <origin xyz="0.04 0 0" rpy="0 0 0"/>
    <geometry><box size="0.1 0.1 0.02"/></geometry>
  </collision>
</link>
```

Each `<collision>` element acts independently in the physics engine. Combined, they approximate the L-shape.

## Self-Collision Avoidance

Ensure collision shapes of connected links don't overlap in the robot's default (zero) configuration:

- Leave small gaps between adjacent collision geometries
- Wheels should not intersect the chassis collision box at any joint position
- For articulated arms, check collision at extreme joint limits

In Gazebo, adjacent links connected by a joint are excluded from mutual collision by default. Non-adjacent links are NOT excluded.

## Gazebo-Specific Configuration

### Collision Filtering

In Gazebo (gz-sim), use the `<collision>` SDF extensions for fine-grained control:

```xml
<!-- In a gazebo tag block for the link -->
<gazebo reference="left_wheel_link">
  <collision>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
      <contact>
        <ode>
          <max_vel>0.1</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
    </surface>
  </collision>
</gazebo>
```

### Material Properties

Friction coefficients affect how the robot interacts with the ground:

| Surface | mu (typical) |
|---|---|
| Rubber on carpet | 0.8–1.2 |
| Rubber on tile | 0.5–0.8 |
| Plastic on wood | 0.2–0.4 |
| Metal on metal | 0.3–0.6 |

### Max Contacts

Limit the number of contact points per collision pair to improve performance:

```xml
<gazebo reference="base_link">
  <max_contacts>5</max_contacts>
</gazebo>
```

## Nav2 Footprint Consistency

The Nav2 costmap `robot_radius` or `footprint` polygon is **separate** from the URDF collision but they must be consistent:

```yaml
# navigation.yaml
local_costmap:
  robot_radius: 0.22  # Should match or slightly exceed URDF collision envelope

# OR for non-circular robots:
local_costmap:
  footprint: "[[0.2, 0.15], [0.2, -0.15], [-0.2, -0.15], [-0.2, 0.15]]"
```

Key relationships:
- **inscribed_radius**: largest circle fitting inside the footprint → robot can rotate freely in this space
- **circumscribed_radius**: smallest circle enclosing the footprint → robot occupies at most this much space
- Both must be >= the URDF collision envelope projected onto the ground plane

## Measurement Strategy

1. Measure the actual robot's physical extent from above (footprint)
2. Set the URDF collision shapes to match or slightly exceed the physical envelope
3. Set the Nav2 footprint to match or slightly exceed the URDF collision projection
4. Add 1–2 cm margin for safety in the Nav2 footprint

## Common Collision Mistakes

| Mistake | Consequence |
|---|---|
| Using detailed mesh as collision | Slow simulation, potential solver failure |
| Collision extends beyond visual | Robot collides with things it appears not to touch |
| Missing collision on wheels | Wheels fall through the ground |
| No collision on sensor mounts | Objects pass through sensor housings |
| Nav2 footprint smaller than collision | Planner plans paths the robot can't fit through |
| Overlapping collision on connected links | Constant self-collision forces, unstable behavior |
