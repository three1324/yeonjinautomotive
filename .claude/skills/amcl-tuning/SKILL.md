<!--
SPDX-FileCopyrightText: 2026 Michael Wimble
SPDX-License-Identifier: Apache-2.0
-->

---
name: amcl-tuning
description: 'Tune AMCL particle filter localization. Use when adjusting alpha1-5 motion model, min/max_particles, laser_model_type, or recovery parameters.'
---

# AMCL Tuning Guide — Adaptive Monte Carlo Localization

## Algorithm Overview

AMCL implements a **particle filter** (Monte Carlo Localization) with KLD-adaptive resampling. Each particle represents a hypothesis of the robot's pose (x, y, θ) in the map frame. On each update cycle:

1. **Prediction** — particles are propagated forward using the odometry motion model, with noise injected via the alpha parameters.
2. **Measurement update** — each particle is weighted by how well the expected laser scan at that pose matches the actual scan.
3. **Resampling** — low-weight particles are culled, high-weight particles are duplicated. KLD-sampling adjusts the particle count dynamically.

---

## Motion Model Parameters (alpha1–alpha5)

The motion model injects noise into the particle propagation. Higher alpha = **more noise** = particles spread more = slower convergence but more robust to odometry model errors. Lower alpha = less noise = faster convergence but the filter may lose track if the model is inaccurate.

For differential drive, use `robot_model_type: "nav2_amcl::DifferentialMotionModel"`.

| Parameter | Controls | Physical Meaning |
|-----------|----------|------------------|
| `alpha1` | Rotation noise FROM rotation | Robot turns → uncertainty in how much it actually turned |
| `alpha2` | Rotation noise FROM translation | Robot drives straight → small heading drift accumulates |
| `alpha3` | Translation noise FROM translation | Robot drives → uncertainty in distance actually traveled |
| `alpha4` | Translation noise FROM rotation | Robot turns → small positional drift (center of rotation error) |
| `alpha5` | Translation noise FROM translation (omni only) | Lateral slip in omnidirectional drives — **ignored for diff-drive** |

### Tuning Strategy

Start with these defaults and adjust:

```yaml
alpha1: 0.2   # Rotation from rotation — increase if robot heading drifts during turns
alpha2: 0.2   # Rotation from translation — increase if heading drifts during straight-line driving
alpha3: 0.2   # Translation from translation — increase if distance traveled is uncertain (wheel slip)
alpha4: 0.2   # Translation from rotation — increase if position drifts during turns
alpha5: 0.2   # Omni only — leave at 0.2 for diff-drive
```

**If particles diverge after turns**: increase `alpha1` and `alpha4`.  
**If particles diverge during straight driving**: increase `alpha2` and `alpha3`.  
**If localization is sluggish**: decrease alpha values cautiously—too low and the filter won't cover the true pose.

---

## Laser Model Parameters

### Model Selection

| Model | Pros | Cons |
|-------|------|------|
| `likelihood_field` | Fast, recommended for most cases | Doesn't model max-range or short readings |
| `beam` | Physically accurate, models all phenomena | Slow (ray casting at every update), rarely needed |

### Mixture Weights (must sum to 1.0)

The laser model is a weighted mixture of four components:

```yaml
z_hit: 0.5     # Probability that a reading is a "good" hit on a known obstacle
z_rand: 0.5    # Probability that a reading is random noise (uniform distribution)
z_max: 0.0     # Probability that the sensor returns max range (saturated)
z_short: 0.0   # Probability of an unexpected short reading (dynamic obstacle)
```

For `likelihood_field`, only `z_hit` and `z_rand` matter (z_max and z_short are beam-model concepts). Keep `z_hit + z_rand = 1.0`.

### Sensor Parameters

```yaml
sigma_hit: 0.2                # Gaussian std dev for the hit model — smaller = stricter matching
laser_likelihood_max_dist: 2.0 # Max distance to inflate obstacles in the likelihood field (meters)
max_beams: 60                  # Number of laser rays used per update (from the full scan)
```

`max_beams`: More rays = better accuracy but higher CPU cost. 60 is a solid default. For dense environments, 120–300 may help. For CPU-constrained systems, 30 can work.

---

## Particle Filter Parameters

```yaml
min_particles: 500     # Minimum particle count (floor for KLD-sampling)
max_particles: 2000    # Maximum particle count (ceiling for KLD-sampling)
pf_err: 0.05           # Maximum error between true and estimated distribution (KLD)
pf_z: 0.99             # Upper quantile for KLD (lower pf_err OR higher pf_z = more particles)
resample_interval: 1   # Resample every N filter updates (1 = every update)
```

**KLD-adaptive resampling**: The filter dynamically adjusts particle count between min and max. Lower `pf_err` → more particles allocated → better approximation but higher CPU. In practice, `min_particles: 500` and `max_particles: 2000` work for most indoor environments.

---

## Recovery Parameters

When the robot is "kidnapped" or particles converge to the wrong spot, random particles must be injected to allow re-localization.

```yaml
recovery_alpha_fast: 0.1   # Exponential decay rate for short-term likelihood average
recovery_alpha_slow: 0.001 # Exponential decay rate for long-term likelihood average
```

When the short-term average drops significantly below the long-term average, random particles are injected. Setting both to `0.0` **disables** recovery (not recommended). Typical values: `fast: 0.1`, `slow: 0.001`.

---

## Update Triggers

```yaml
update_min_d: 0.25   # Minimum translation (meters) before a filter update
update_min_a: 0.2    # Minimum rotation (radians) before a filter update
```

**Critical for CPU**: If both are too small, AMCL updates constantly even for minor jitter. If too large, the robot moves far between updates and particles may lag behind. For typical indoor use, 0.25 m and 0.2 rad (~11°) are good defaults.

---

## Transform and Initialization

```yaml
transform_tolerance: 1.0    # TF publication future-dating tolerance (seconds)
set_initial_pose: true       # Automatically set initial pose on startup
initial_pose:
  x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0
```

If `set_initial_pose: true`, AMCL will initialize the particle cloud at the given pose on startup. Otherwise, you must publish to `/initialpose` (e.g., from RViz).

`transform_tolerance`: How far into the future the map→odom transform is published. Higher values smooth out TF jitter but add latency. If the robot "jumps" erratically, try increasing this to 0.5–1.0.

---

## Complete AMCL Configuration

```yaml
amcl:
  ros__parameters:
    use_sim_time: false
    # Motion model
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2

    # Laser model
    laser_model_type: "likelihood_field"
    z_hit: 0.5
    z_rand: 0.5
    z_max: 0.0
    z_short: 0.0
    sigma_hit: 0.2
    laser_likelihood_max_dist: 2.0
    max_beams: 60
    laser_max_range: 12.0
    laser_min_range: 0.1

    # Particle filter
    min_particles: 500
    max_particles: 2000
    pf_err: 0.05
    pf_z: 0.99
    resample_interval: 1

    # Recovery
    recovery_alpha_fast: 0.1
    recovery_alpha_slow: 0.001

    # Update triggers
    update_min_d: 0.25
    update_min_a: 0.2

    # TF and frames
    global_frame_id: "map"
    odom_frame_id: "odom"
    base_frame_id: "base_footprint"
    scan_topic: "/scan"
    transform_tolerance: 1.0

    # Beam skip (for dense environments)
    do_beamskip: false
    beam_skip_distance: 0.5
    beam_skip_threshold: 0.3
    beam_skip_error_threshold: 0.9

    # Initialization
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
    always_reset_initial_pose: false

    # First map only
    first_map_only: false
```

---

## Common Issues and Fixes

| Symptom | Cause | Fix |
|---------|-------|-----|
| Particles diverge, robot position wanders | Alpha values too low for actual odometry noise | Increase alpha1–alpha4 by 50% |
| Robot jumps to a different position | Particle cloud split between two hypotheses | Increase `min_particles`, decrease `transform_tolerance` |
| Localization lost in featureless corridors | Not enough unique scan features to disambiguate | Add distinctive features (posters, furniture), increase `max_particles` |
| High CPU usage | Too many particles or max_beams | Reduce `max_beams` to 30–60, lower `max_particles` |
| Slow to localize on startup | Initial pose far from truth with few particles | Set `set_initial_pose: true` with approximate pose, increase `min_particles` to 1000 |
| Oscillation between two poses | Symmetric environment (identical hallways) | Use global localization or manually set initial pose |

---

## Beam Skip Optimization

In dense environments (many obstacles close together), beam skip prevents a single bad scan ray from torpedoing the entire measurement update:

```yaml
do_beamskip: true
beam_skip_distance: 0.5       # Skip beams that deviate more than this from expected
beam_skip_threshold: 0.3      # Fraction of beams that can be skipped before discarding the entire scan
beam_skip_error_threshold: 0.9 # Error threshold for individual beam rejection
```

Enable this if AMCL struggles in cluttered environments with many dynamic obstacles.
