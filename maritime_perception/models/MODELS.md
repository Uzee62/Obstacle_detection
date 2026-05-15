# Models Module

The `models/` package defines every shared data structure used across the system. Nothing in this package imports from any other package — it is the foundation that all other layers build upon. If you want to understand what data looks like at any stage of the pipeline, start here.

---

## Table of Contents

- Design Philosophy
- Module Overview
- common.py — Timing and Source Primitives
- observation.py — Universal Sensor Output
- pose.py — Vessel State
- track.py — Tracker Internal State
- world_model.py — Public Perception Output
- Data Flow Through Models

---

## Design Philosophy

**No cross-package imports.** Every file in `models/` imports only from the Python standard library or from other files within `models/`. This means:

- Any module in the system can import from `models/` without creating a circular dependency.
- The data layer is fully testable in isolation — no hardware, no config files, no sensor drivers required.
- Refactoring a higher-level module (e.g., the tracker) cannot break the data definitions.

**Immutability at the boundary.** `WorldObject` — the type that safety and navigation layers consume — uses `@dataclass(frozen=True)`. Downstream consumers cannot accidentally mutate the perception system's state.

**Explicitness over convenience.** Every field has an explicit type annotation and a clear unit in its name or docstring (`position_x` in metres, `timestamp_ns` in nanoseconds). There are no catch-all dict fields.

---

## Module Overview

| File             |        Primary type        |  Used by |
|---               |---                         |--       -|
| `common.py`      |  `Header`, `SensorSource`  |Everything — attached to every message |
| `observation.py` | `DetectionObservation`     |Sensor pipelines (produce),fusion tracker          (consumes)|
| `pose.py`        |   `VesselPose`              | Future ego-motion compensation in tracker |
| `track.py`       | `ObjectTrack`, `TrackState` | Fusion tracker (internal) |
| `world_model.py` | `WorldModel`, `WorldObject` | Builder (produces), interfaces + health (consume) |

---

## common.py — Timing and Source Primitives

This file contains the lowest-level types that travel with every piece of data in the system.

### `SensorSource` (Flag Enum)

A bitmask enum that identifies which physical sensors contributed to a piece of data.

```
LIDAR  = 0b00001   (1)
AIS    = 0b00010   (2)
FLS    = 0b00100   (4)
MBES   = 0b01000   (8)
RADAR  = 0b10000   (16)
```

Being a `Flag` enum means you can combine sources: `SensorSource.LIDAR | SensorSource.AIS` represents a fused observation from both sensors, stored as the integer `3`.

### `Header`

Metadata carried by every message in the system. Attach one to any data structure that travels between modules.

| Field | Type | Description |
|---|---|---|
| `timestamp_ns` | `int` | Monotonic nanosecond timestamp (from `time.monotonic_ns()`) |
| `sensor_id` | `str` | Human-readable sensor identifier, e.g. `"lidar_0"` |
| `frame_id` | `str` | Coordinate frame, typically `"vessel"` (bow=+x, port=+y) |
| `source` | `SensorSource` | Bitmask of contributing sensors |

**Why monotonic?** `time.monotonic_ns()` never jumps backward and is unaffected by NTP corrections. This is critical for Kalman filter `dt` calculations — a backward-jumping wall clock would produce a negative `dt` and corrupt the state covariance.

### Helper Functions

| Function | Returns | Description |
|---|---|---|
| `now_ns()` | `int` | Current monotonic timestamp in nanoseconds |
| `ns_to_ms(ns)` | `float` | Convert nanoseconds → milliseconds |
| `elapsed_ms(start_ns)` | `float` | Milliseconds elapsed since `start_ns` |

---

## observation.py — Universal Sensor Output

### `DetectionObservation`

The single data type produced by **every sensor pipeline** in the system. This common interface is what makes sensor-agnostic fusion possible — the tracker never needs to know whether it is looking at a LiDAR detection or an AIS contact.

| Field | Type | Unit | Description |
|---|---|---|---|
| `header` | `Header` | — | Timestamp, sensor ID, frame, source bitmask |
| `position_x` | `float` | metres | Forward offset from vessel centre of rotation (bow = positive) |
| `position_y` | `float` | metres | Lateral offset (port = positive) |
| `size_m` | `float` | metres | Estimated obstacle radius |
| `range_m` | `float` | metres | Distance from vessel origin |
| `bearing_deg` | `float` | degrees | Angle from bow, clockwise positive |
| `point_count` | `int` | — | Number of sensor returns that formed this detection |
| `confidence` | `float` | 0.0–1.0 | Detection quality estimate |

**Confidence model (LiDAR):**

```
point_confidence  = min(1.0, point_count / 30)      # saturates at 30 returns
range_penalty     = 1.0 - 0.5 × (range_m / 30.0)   # decays linearly with range
confidence        = point_confidence × range_penalty
```

This model is intentionally simple and auditable. A detection with 30+ points at close range scores near 1.0; a detection with 5 points at 28 m scores around 0.08.

---

## pose.py — Vessel State

### `VesselPose`

Represents the estimated state of the vessel itself — its position, velocity, and orientation — typically from an RTK-GPS / IMU fusion. This type is defined now for use in future ego-motion compensation (adjusting obstacle velocities for own-ship motion).

**Position (NED frame, metres from fixed origin):**

| Field | Description |
|---|---|
| `pos_north` | Northing |
| `pos_east` | Easting |
| `pos_down` | Depth (positive downward) |

**Velocity (NED frame, m/s):**

| Field | Description |
|---|---|
| `vel_north` | Northward speed |
| `vel_east` | Eastward speed |
| `vel_down` | Vertical speed |

**Orientation:**

| Field | Unit | Description |
|---|---|---|
| `heading_deg` | degrees | True heading (0 = North, clockwise) |
| `roll_deg` | degrees | Roll angle |
| `pitch_deg` | degrees | Pitch angle |
| `yaw_rate_dps` | deg/s | Turn rate |

**RTK metadata:**

| Field | Description |
|---|---|
| `fix_quality` | Integer quality code (0=none, 4=RTK fixed) |
| `hdop` | Horizontal dilution of precision |
| `num_satellites` | Tracked satellites |

---

## track.py — Tracker Internal State

These types are **internal to the fusion layer**. Nothing outside `fusion/` should hold a reference to an `ObjectTrack`.

### `TrackState` (Enum)

The four lifecycle states of a tracked obstacle:

```
TENTATIVE  — New track; not yet confirmed; not visible to consumers
CONFIRMED  — Seen consistently; published in the WorldModel
COASTING   — Recently lost; still predicted forward; still published
DEAD       — Pruned; removed from the tracker's active list
```

### `ObjectTrack`

Holds the complete state of one tracked obstacle.

| Field | Type | Description |
|---|---|---|
| `track_id` | `int` | Globally unique, never reused within a session |
| `state` | `TrackState` | Current lifecycle state |
| `state_vec` | `np.ndarray` shape (4,) | `[x, y, vx, vy]` — position and velocity in metres / m/s |
| `covariance` | `np.ndarray` shape (4,4) | Kalman state covariance (position uncertainty lives in the top-left 2×2) |
| `score` | `float` | Quality metric in [0, 1]; evolves asymptotically toward 1 on hits, decays on misses |
| `hit_count` | `int` | Cumulative observations associated to this track |
| `miss_count` | `int` | Consecutive scans without an associated observation |
| `age_scans` | `int` | Total scans since track creation |
| `last_observation` | `DetectionObservation \| None` | Most recent associated observation |
| `size_m` | `float` | Running estimate of obstacle physical size |
| `kalman` | `KalmanFilter \| None` | Attached motion model instance |

**Score evolution:**
```
On hit:  score ← score + 0.15 × (1 - score)    # asymptotic rise toward 1.0
On miss: score ← score × 0.80                    # exponential decay
```

**Dynamic / static classification:**

A track is classified as dynamic if its estimated speed exceeds 0.15 m/s:
```python
speed = sqrt(vx² + vy²)
dynamic = speed > 0.15  # m/s
```

---

## world_model.py — Public Perception Output

These are the types that safety and navigation layers consume. They represent the final, curated output of the entire perception stack.

### `WorldObject`

An **immutable snapshot** of one confirmed obstacle. Created by `WorldModelBuilder` and never modified after creation (`frozen=True` dataclass).

| Field | Type | Description |
|---|---|---|
| `object_id` | `int` | Same ID as the internal track; stable across scans |
| `position_x` | `float` | Estimated x position, metres (vessel frame) |
| `position_y` | `float` | Estimated y position, metres |
| `velocity_x` | `float` | Estimated x velocity, m/s |
| `velocity_y` | `float` | Estimated y velocity, m/s |
| `heading_deg` | `float` | Estimated movement direction, degrees |
| `range_m` | `float` | Distance from vessel origin |
| `bearing_deg` | `float` | Bearing from bow, degrees |
| `speed_ms` | `float` | Estimated speed magnitude, m/s |
| `size_m` | `float` | Estimated obstacle physical radius, metres |
| `safety_radius_m` | `float` | `size_m + position_std_m` — the collision avoidance boundary |
| `confidence` | `float` | Track score [0, 1] |
| `position_std_m` | `float` | Position uncertainty (sqrt of trace of position covariance) |
| `dynamic` | `bool` | True if speed > 0.15 m/s |
| `coasting` | `bool` | True if track is COASTING (no recent observation) |
| `sensor_sources` | `int` | Bitmask of `SensorSource` values that contributed |

**`safety_radius_m`** is the field that collision-avoidance algorithms should use. It inflates the physical size by the current position uncertainty — when the Kalman filter is unsure exactly where the obstacle is, the exclusion zone grows accordingly.

### `WorldModel`

The top-level perception output published each cycle.

| Field | Type | Description |
|---|---|---|
| `header` | `Header` | Cycle timestamp and metadata |
| `scan_id` | `int` | Monotonically increasing cycle counter |
| `latency_ms` | `float` | End-to-end time from scan acquisition to world model creation |
| `objects` | `list[WorldObject]` | All confirmed and coasting obstacles this cycle |

---

## Data Flow Through Models

```
Sensor hardware
    │
    ▼ produces
LidarScan  (internal to lidar/driver.py)
    │
    ▼ preprocessed into
CartesianPoint[]  (internal to lidar/ package)
    │
    ▼ grouped into
Segment[]  (internal to lidar/ package)
    │
    ▼ converted into
DetectionObservation[]   ←── universal sensor interface
    │
    ▼ consumed by
FusionTracker → ObjectTrack[]  (internal to fusion/ package)
    │
    ▼ projected into
WorldModel / WorldObject[]   ←── public perception output
    │
    ▼ consumed by
JsonPublisher, HealthMonitor, Safety layer
```
