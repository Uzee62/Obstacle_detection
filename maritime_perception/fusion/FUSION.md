# Fusion Module

The `fusion/` package is the multi-object tracker at the heart of the perception system. It receives `DetectionObservation` objects from any number of sensors, maintains Kalman-filtered obstacle tracks across time, and produces a stable `WorldModel` that safety and navigation layers consume.

The fusion layer predicts where known objects should be, matches new detections to them, updates their motion estimates, manages their lifecycle, and publishes a stable world model for navigation and safety.

---

## Table of Contents

- [Overview](#overview)
- [The Tracking Problem](#the-tracking-problem)
- [tracker.py — FusionTracker](#trackerpy--fusiontracker)
- [motion_model.py — KalmanFilter](#motion_modelpy--kalmanfilter)
- [association.py — Data Association](#associationpy--data-association)
- [builder.py — WorldModelBuilder](#builderpy--worldmodelbuilder)
- [Algorithm Summary](#algorithm-summary)
- [Configuration Reference](#configuration-reference)

---

## Overview

Each 10 Hz cycle, the main loop calls the tracker with a batch of fresh observations:

```
FusionTracker.update(observations)
    │
    ├─ 1. Predict: advance every existing track forward in time (Kalman predict)
    ├─ 2. Associate: match observations to existing tracks (Hungarian + Mahalanobis)
    ├─ 3. Update: incorporate matched observations (Kalman update)
    ├─ 4. Lifecycle: promote, coast, or prune tracks
    └─ Returns: WorldModel (via WorldModelBuilder)
```

**Why Kalman + Hungarian?**

These are the industry-standard algorithms for multi-object tracking in real-time systems. They are well-understood, mathematically principled, and require no training data. Their behavior is fully predictable and auditable — critical for a safety system.

---

## The Tracking Problem

At each scan, the system receives a list of obstacle detections. Without tracking, these would be noisy, discontinuous blobs — appearing and disappearing with every wave. Tracking solves three sub-problems:

1. **Association** — which detection matches which existing obstacle?
2. **State estimation** — given noisy position measurements, what is the best estimate of position and velocity?
3. **Lifecycle management** — when is a new detection a real obstacle (vs. noise)? When has an obstacle truly disappeared (vs. a missed detection)?

The fusion layer addresses each in sequence.

---

## tracker.py — FusionTracker

**Class:** `FusionTracker`

The top-level coordinator. Holds the list of active `ObjectTrack` objects and drives the predict → associate → update → lifecycle cycle.

### Track State Machine

Every track passes through well-defined states:

```
                    ┌──────────────┐
   new detection ──►│  TENTATIVE   │── miss > miss_to_delete ──► DEAD
                    └──────┬───────┘
                           │ hit_count ≥ hits_to_confirm
                           ▼
                    ┌──────────────┐
          recovered │  CONFIRMED   │── miss > miss_to_coast ──►┐
          ◄─────────┤              │                            │
                    └──────────────┘                            ▼
                                                        ┌──────────────┐
                                                        │   COASTING   │── miss > miss_to_dead ──► DEAD
                                                        └──────────────┘
```

| State | Meaning | Published? |
|---|---|---|
| `TENTATIVE` | New track; not yet confirmed | No |
| `CONFIRMED` | Seen consistently; trusted | Yes |
| `COASTING` | No recent observation; predicted forward | Yes (flagged) |
| `DEAD` | Pruned from the tracker | No |

**Default thresholds (configurable):**

| Threshold | Default | Description |
|---|---|---|
| `hits_to_confirm` | 3 | Consecutive hits needed to confirm a new track |
| `miss_to_coast` | 10 | Missed scans before CONFIRMED → COASTING |
| `miss_to_dead_tentative` | 3 | Missed scans to prune a TENTATIVE track |
| `miss_to_dead_coasting` | 15 | Missed scans to prune a COASTING track |

### Track Score (Quality Metric)

Every track carries a `score` in [0, 1] that evolves asymptotically:

```python
# When a detection is associated (hit):
score = score + 0.15 * (1.0 - score)   # exponential rise toward 1.0

# When no detection is associated (miss):
score = score * 0.80                    # exponential decay toward 0.0
```

After 20 consecutive hits, a track scores ~0.96. After 5 consecutive misses from a perfect score, it falls to ~0.33. This score becomes the `confidence` field in `WorldObject`.

### Cycle Details

**Predict phase:**

Every active (non-DEAD) track is advanced one time step using `KalmanFilter.predict()`. This gives a predicted position that is used for Mahalanobis gating in the association step.

**Association phase:**

`DataAssociator.associate()` returns:
- `matched_pairs` — `(track_id, observation)` pairs
- `unmatched_observations` — detections with no matching track
- `unmatched_tracks` — tracks with no matching detection

**Update phase:**

- **Matched tracks:** `KalmanFilter.update(observation)` incorporates the measurement.
- **Unmatched observations:** Spawn new TENTATIVE tracks.
- **Unmatched tracks:** Increment `miss_count`; apply score decay.

**Lifecycle phase:**

Walk every track and apply state transitions based on current counters.

**Dead tracks:** Removed from the active list. Track IDs are never reused within a session.

---

## motion_model.py — KalmanFilter

**Class:** `KalmanFilter`

A standard four-state constant-velocity Kalman filter. It is the mathematical core of each track's state estimate.

### State Vector

```
state_vec = [x, y, vx, vy]
             ↑  ↑   ↑   ↑
             position   velocity
             (metres)   (m/s)
```

### Matrices

All matrices use `dt = 0.1 s` (the 10 Hz fusion rate).

**F — State transition matrix** (predicts next state from current state):

```
F = [[1, 0, dt,  0],
     [0, 1,  0, dt],
     [0, 0,  1,  0],
     [0, 0,  0,  1]]
```

This encodes the constant-velocity assumption: `x_new = x + vx × dt`.

**H — Observation matrix** (maps state to measurement space):

```
H = [[1, 0, 0, 0],
     [0, 1, 0, 0]]
```

Sensors measure position only — not velocity. Velocity is inferred from position changes over time.

**Q — Process noise covariance** (how much the model trusts the constant-velocity assumption):

```
Q = diag([0.1², 0.1², 0.5², 0.5²])
    (pos noise 0.01 m²,  vel noise 0.25 (m/s)²)
```

Larger values tell the filter "the model is imperfect; allow more freedom." The position values are tight (0.1 m); the velocity values are looser (0.5 m/s) to handle manoeuvring vessels.

**R — Measurement noise covariance** (how much to trust the sensor):

```
R = diag([0.3², 0.3²])
    (0.09 m²  per axis — LiDAR centroid accuracy)
```

### Methods

**`predict()`:**

```
x_pred = F × x
P_pred = F × P × Fᵀ + Q
```

Advances the state estimate one time step. The covariance `P` grows because we are less certain about the state after prediction.

**`update(measurement)`:**

```
y = z - H × x_pred          (innovation — difference from prediction)
S = H × P × Hᵀ + R          (innovation covariance)
K = P × Hᵀ × S⁻¹            (Kalman gain)
x = x_pred + K × y           (updated state)
P = (I - K × H) × P × (I - K × H)ᵀ + K × R × Kᵀ    (Joseph form)
```

The **Joseph form** for the covariance update is numerically more stable than the simpler `P = (I - KH)P` form, especially when the gain is close to 1. It guarantees a symmetric positive-definite covariance matrix even under floating-point rounding.

**`mahalanobis(measurement)`:**

```
y = z - H × x_pred
S = H × P × Hᵀ + R
d² = yᵀ × S⁻¹ × y
```

Returns the squared Mahalanobis distance between the measurement and the track's predicted position, normalized by the combined uncertainty. This is used for gating in the association step.

**Why Mahalanobis distance instead of Euclidean?**

A Euclidean distance does not account for uncertainty. A track that has been coasting for 5 scans has a large position uncertainty — it should accept associations from a wider area. The Mahalanobis distance automatically widens the gate as `P` grows during prediction-only steps.

---

## association.py — Data Association

**Class:** `DataAssociator`

Solves the assignment problem: which detection belongs to which track?

### Mahalanobis Gating

Before running the assignment algorithm, observations that are clearly implausible for a given track are rejected. The gate threshold is `9.21` (the 99.5th percentile of a χ² distribution with 2 degrees of freedom). Any observation with a squared Mahalanobis distance > 9.21 from a track is forbidden from being assigned to it.

This prevents:
- A detection from one part of the scene being assigned to a track on the other side.
- Corrupting well-established tracks with spurious noise detections.

### Hungarian Algorithm

After gating, the remaining (track, observation) pairs are passed to a **Hungarian algorithm** (also called the Munkres algorithm) implemented in `scipy.optimize.linear_sum_assignment`.

The cost matrix is the Mahalanobis distance for each valid (track, observation) pair, and infinity for gated-out pairs. The algorithm finds the globally optimal assignment that minimises total cost in O(n³) time.

**Why global optimum instead of greedy nearest-neighbour?**

Greedy matching can make a locally good assignment that prevents a better global assignment. Example: if track A is 0.5 m from detection 1 and 2.0 m from detection 2, and track B is 0.6 m from detection 1 and 0.3 m from detection 2, greedy would assign A→1, B→nothing. Hungarian correctly assigns A→1, B→2 (total cost 0.8 vs. 0.5).

With typical scene sizes of n < 20, the O(n³) complexity is entirely negligible.

### Return Value

```python
matched_pairs:            list[tuple[int, DetectionObservation]]
unmatched_observations:   list[DetectionObservation]
unmatched_tracks:         list[int]   # track IDs
```

---

## builder.py — WorldModelBuilder

**Class:** `WorldModelBuilder`

Converts the tracker's internal `ObjectTrack` list into the public `WorldModel` snapshot.

### Filtering

Only tracks in `CONFIRMED` or `COASTING` state pass through. `TENTATIVE` tracks are suppressed — they have not yet been validated as real obstacles. `DEAD` tracks have already been removed from the active list.

An additional confidence threshold (default 0.25) drops tracks that technically qualify by state but have very low scores.

### Projection

For each qualifying track, the builder reads:

```python
x, y, vx, vy = track.state_vec

heading_deg = degrees(atan2(vy, vx))
speed_ms    = sqrt(vx² + vy²)
range_m     = sqrt(x² + y²)
bearing_deg = degrees(atan2(y, x))

# Position uncertainty from the top-left 2×2 of the covariance matrix
pos_var     = P[0,0] + P[1,1]
pos_std_m   = sqrt(max(0.0, pos_var))

safety_radius_m = size_m + pos_std_m
```

The `safety_radius_m` is the key output for collision avoidance — it grows whenever the Kalman filter is uncertain about where the obstacle is.

### Output

A frozen `WorldModel` dataclass containing a frozen list of `WorldObject` instances. The `frozen=True` flag on both types ensures that downstream consumers (safety layer, navigation, publisher) cannot accidentally mutate the perception system's state.

---

## Algorithm Summary

| Problem | Algorithm | Complexity | Library |
|---|---|---|---|
| State prediction | Kalman predict (linear) | O(1) per track | numpy |
| State update | Kalman update (Joseph form) | O(1) per track | numpy |
| Association gating | Mahalanobis distance | O(t × d) | numpy |
| Optimal assignment | Hungarian algorithm | O(n³) | scipy |
| Uncertainty propagation | Covariance matrix | O(1) per track | numpy |

Where `t` = active tracks, `d` = detections per scan, `n` = min(t, d).

---

## Configuration Reference

All parameters live in `configs/default.yaml` under the `tracking` key.

```yaml
tracking:
  # Kalman filter noise
  process_noise_position: 0.1    # metres — how much position can deviate from model
  process_noise_velocity: 0.5    # m/s   — how much velocity can deviate from model
  measurement_noise: 0.3         # metres — LiDAR centroid accuracy

  # Association
  mahalanobis_gate: 9.21         # 99.5% CI for 2-DOF (chi-squared)

  # Lifecycle thresholds
  hits_to_confirm: 3             # consecutive hits → TENTATIVE to CONFIRMED
  miss_to_coast: 10              # consecutive misses → CONFIRMED to COASTING
  miss_to_dead_tentative: 3      # consecutive misses → TENTATIVE to DEAD
  miss_to_dead_coasting: 15      # consecutive misses → COASTING to DEAD

  # Dynamic classification
  dynamic_speed_threshold_ms: 0.15   # m/s — above this = dynamic obstacle

fusion:
  loop_rate_hz: 10.0             # main fusion loop rate
```

**Tuning guidance:**

- **High false-positive rate** (non-existent tracks appearing): Increase `hits_to_confirm`, tighten `mahalanobis_gate`.
- **Tracks disappearing too quickly**: Increase `miss_to_coast`, increase `miss_to_dead_coasting`.
- **Velocity estimates noisy**: Decrease `process_noise_velocity` (trust the constant-velocity model more).
- **Position estimates lagging real movement**: Increase `process_noise_position` (allow more dynamic response).
