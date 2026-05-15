# LiDAR Perception Pipeline

The `sensors/lidar/` package implements the complete processing chain from raw RPLidar S2 serial data to structured `DetectionObservation` objects. It is the primary sensor in the system and contains the most algorithmic complexity.

---

## Table of Contents

- [Overview](#overview)
- [Pipeline Stages](#pipeline-stages)
- [driver.py — Hardware I/O](#driverpy--hardware-io)
- [preprocessor.py — Point Validation and Coordinate Transform](#preprocessorpy--point-validation-and-coordinate-transform)
- [noise_filter.py — Adaptive Sea-Clutter Rejection](#noise_filterpy--adaptive-sea-clutter-rejection)
- [segmentation.py — Obstacle Grouping](#segmentationpy--obstacle-grouping)
- [extractor.py — Observation Generation](#extractorpy--observation-generation)
- [pipeline.py — Orchestration and Diagnostics](#pipelinepy--orchestration-and-diagnostics)
- [sensor_thread.py — Background Thread and Health](#sensor_threadpy--background-thread-and-health)
- [Configuration Reference](#configuration-reference)

---

## Overview

```
Serial port (RPLidar S2)
        │
        ▼
    RPLidarDriver           ← hardware I/O, quality filter, mm→m
        │ LidarScan
        ▼
    LidarPreprocessor       ← range gate, FOV mask, polar→Cartesian
        │ CartesianPoint[]
        ▼
    AdaptiveNoiseFilter     ← temporal persistence, adaptive threshold
        │ CartesianPoint[]
        ▼
    JumpDistanceSegmenter   ← jump-distance split + KD-tree merge
        │ Segment[]
        ▼
    ObstacleExtractor       ← centroid, size, confidence model
        │ DetectionObservation[]
        ▼
    LidarSensorThread       ← wraps pipeline in background thread
        │ latest() → DetectionObservation[]
        ▼
    FusionTracker (main.py)
```

Each stage is a separate class with a single responsibility. The `LidarPerceptionPipeline` wires them together and the `LidarSensorThread` moves the whole chain into a background thread.

---

## Pipeline Stages

### Stage 1 — driver.py — Hardware I/O

**Class:** `RPLidarDriver`

Handles all communication with the physical RPLidar S2 sensor. The rest of the system never touches the serial port directly.

**Data types produced:**

- `RawScanPoint(angle_deg, distance_m, quality)` — a single laser return
- `LidarScan(points, header)` — a complete 360° rotation

**What the driver does:**

1. Opens the serial port at 1,000,000 baud (RPLidar S2 requirement).
2. Calls the SDK's iterator to yield successive full rotations.
3. For each point, drops returns with `quality == 0` or `distance == 0` (the sensor uses zero to signal invalid returns).
4. Converts millimetres to metres.
5. Timestamps the scan at the moment of acquisition using `time.monotonic_ns()`.

**Auto-reconnect:**

If the serial connection is lost (USB disconnect, port error), the driver retries up to 10 times with a 2-second backoff between attempts before giving up and allowing the health system to escalate the fault.

**Key configuration:**

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Serial port path |
| `baudrate` | `1000000` | Must match sensor firmware |
| `timeout_s` | `3.0` | Read timeout before retry |
| `max_reconnect_attempts` | `10` | Retries on connection loss |

---

### Stage 2 — preprocessor.py — Point Validation and Coordinate Transform

**Class:** `LidarPreprocessor`

Turns a raw `LidarScan` into validated `CartesianPoint` objects in the vessel coordinate frame. This is a five-gate sequential filter — a point that fails any gate is dropped.

**`CartesianPoint` fields:** `angle_deg`, `distance_m`, `x`, `y`, `quality`

**The five gates (in order):**

| Gate | Rejects |
|---|---|
| 1. **Validity** | NaN, ±∞, zero distances |
| 2. **Range** | Points closer than `min_range_m` (0.3 m) — own-vessel returns; farther than `max_range_m` (30 m) — outside sensor accuracy |
| 3. **Intensity** | (Optional) Points below `min_quality_threshold` — weak returns, often noise |
| 4. **FOV mask** | Points inside defined "blind sectors" — e.g., the aft mast at 178°–182° |
| 5. **Polar → Cartesian** | Passes all surviving points; converts to x/y |

**FOV masking detail:**

The vessel profile defines sectors to blank out that correspond to physical structures (mast, superstructure) that would otherwise appear as permanent false obstacles. Masks can wrap around 0°:

```yaml
# 350° to 10° wraps through north — handled correctly
fov_masks:
  - {start: 350.0, end: 10.0}
  - {start: 178.0, end: 182.0}
```

**Coordinate convention:**

```
+x → bow (forward)
+y → port (left)
```

Conversion from polar `(angle_deg, distance_m)`:
```python
x = distance_m * cos(radians(angle_deg))
y = distance_m * sin(radians(angle_deg))
```

---

### Stage 3 — noise_filter.py — Adaptive Sea-Clutter Rejection

**Class:** `AdaptiveNoiseFilter`

Marine environments produce strong, time-varying clutter: wave crests, spray, floating debris. A simple intensity threshold cannot distinguish a wave from a buoy. This filter uses **temporal persistence** — a genuine obstacle appears in the same location scan after scan; clutter does not.

**How it works:**

The filter maintains a 2D grid of `0.25 m × 0.25 m` cells covering the sensor range. Each cell accumulates a hit count across recent scans.

A point survives the filter only if its cell's hit count reaches `current_threshold`.

**Adaptive threshold:**

The threshold is not fixed. Each scan, the filter measures what fraction of cells are "busy" (clutter density):

```
If clutter_density > high_water_mark:
    threshold ← threshold + 1    (more strict)
Else if clutter_density < low_water_mark:
    threshold ← threshold - 1    (more lenient)
```

The change is smoothed with an exponential moving average (α = 0.1) to prevent oscillation. The threshold is clamped to `[min_threshold, max_threshold]` (default 5–8 hits).

**In rough seas:** Many cells are active → density is high → threshold rises → only persistent, consistent returns survive → wave clutter is rejected.

**In calm water:** Few cells active → density is low → threshold falls → even brief contacts are retained → distant, low-return obstacles are not missed.

**Cell eviction:**

Each cell has a TTL of 15 scans. A cell that has not been hit in 15 consecutive scans is removed from the grid. This prevents the grid from filling with stale returns from obstacles that have moved away.

**Diagnostics output:**

The filter returns a `rejection_pct` — the fraction of input points that were discarded. This feeds into `LidarPipelineStats` and the rolling health monitor, where a sudden spike in rejection can indicate changed sea state or hardware issues.

**Key configuration:**

| Parameter | Default | Description |
|---|---|---|
| `cell_size_m` | `0.25` | Grid resolution |
| `min_threshold` | `5` | Minimum hit count to survive |
| `max_threshold` | `8` | Maximum hit count ceiling |
| `cell_ttl_scans` | `15` | Scans before cell expiry |
| `ema_alpha` | `0.1` | Smoothing factor for threshold adaptation |

---

### Stage 4 — segmentation.py — Obstacle Grouping

**Class:** `JumpDistanceSegmenter`

Groups the filtered `CartesianPoint` stream into discrete obstacle `Segment` objects. Uses two phases:

**Phase 1 — Jump-distance segmentation:**

Points are sorted by angle. The algorithm walks them in angular order. When the distance gap between two adjacent points exceeds a threshold, a new segment begins:

```
threshold = jump_distance_m + range_m × adaptive_k
```

The `adaptive_k` term accounts for the natural spreading of adjacent points at range — at 20 m, two adjacent points are further apart in space than at 2 m even for the same angular step, so the threshold grows with range to avoid over-splitting.

After splitting, each segment is checked against quality filters:
- Minimum and maximum point count (default 3–2000 points)
- Minimum angular arc span (default 0.5°) — eliminates single-point noise

**Phase 2 — KD-tree centroid merge:**

Segmentation can fragment a single physical hull into multiple segments (gaps from rigging, shadows between hull sections). The merge phase reunifies these fragments:

1. Compute the centroid of each segment.
2. Build a `scipy.spatial.cKDTree` on the centroids.
3. Query for pairs within `merge_distance_m` (default 1.0 m).
4. Use a union-find structure to merge connected groups.

The KD-tree approach is O(n log n) — far faster than the naïve O(n²) pairwise comparison. With n typically under 50 segments per scan, this is not a bottleneck, but the design scales cleanly.

**`Segment` fields:**

| Field | Description |
|---|---|
| `points` | All `CartesianPoint` objects in this segment |
| `centroid_x`, `centroid_y` | Mean position |
| `arc_span_deg` | Angular extent — used as a crude size estimate |
| `mean_range_m` | Average range of the segment |

---

### Stage 5 — extractor.py — Observation Generation

**Class:** `ObstacleExtractor`

Converts each `Segment` into a `DetectionObservation` — the universal type that the fusion layer understands.

**Per-segment extraction:**

```python
centroid_x = mean(point.x for point in segment.points)
centroid_y = mean(point.y for point in segment.points)

# Bounding radius = max distance from centroid to any point
radius_m = max(sqrt((p.x - cx)² + (p.y - cy)²) for p in segment.points)
```

**Plausibility check:** If `radius_m > max_obstacle_radius_m` (default 15 m), the segment is discarded. This eliminates harbour walls, jetties, and other fixed infrastructure that cannot be tracked as discrete obstacles.

**Confidence model:**

```
point_confidence = min(1.0, point_count / reference_points)
                 # reference_points default 30; saturates at 30 points

range_penalty = 1.0 - 0.5 × (range_m / max_range_m)
              # decays linearly: 1.0 at 0 m, 0.5 at 30 m

confidence = point_confidence × range_penalty
```

This model is intentionally simple: auditable, monotonic, and bounded to [0, 1]. It captures two physical intuitions:
1. More sensor returns → higher-quality detection.
2. Far detections → lower positional accuracy → lower confidence.

---

## pipeline.py — Orchestration and Diagnostics

**Class:** `LidarPerceptionPipeline`

Wires stages 2–5 together and emits per-scan diagnostic statistics. The driver runs separately (in the sensor thread); the pipeline processes whatever `LidarScan` the driver delivers.

**Processing order:**

```python
cartesian_points = preprocessor.process(scan)
filtered_points  = noise_filter.filter(cartesian_points)
segments         = segmenter.segment(filtered_points)
observations     = extractor.extract(segments)
```

**`LidarPipelineStats` (per-scan metrics):**

| Field | Description |
|---|---|
| `raw_points` | Points delivered by the driver |
| `after_preprocess` | Surviving the five-gate preprocessor |
| `after_noise_filter` | Surviving the temporal persistence filter |
| `noise_rejection_pct` | `(after_preprocess - after_noise_filter) / after_preprocess × 100` |
| `num_segments` | Segments after KD-tree merge |
| `num_observations` | Final `DetectionObservation` count |
| `process_time_ms` | Wall-clock time for stages 2–5 |

These stats are consumed by `HealthMonitor` for trend analysis and by the watchdog for latency alerting.

---

## sensor_thread.py — Background Thread and Health

**Class:** `LidarSensorThread`

Implements `AbstractSensorPipeline`. Runs the driver + pipeline in a `threading.Thread`, isolating blocking hardware I/O from the main fusion loop.

**Threading design:**

- The background thread loops: `acquire scan → run pipeline → store result`
- Results are stored in a `queue.Queue(maxsize=1)` — a single-slot buffer.
- If the main loop is slow, the old result is replaced by the new one; no backlog accumulates.
- `latest()` does a non-blocking `queue.get_nowait()` — returns `[]` if nothing is ready.

**Health checks (`is_healthy()`):**

The thread is considered healthy if all of the following are true:

1. The background thread is alive (`thread.is_alive()`).
2. A scan has arrived within `max_gap_s` (default 1.0 s).
3. The error count in the recent window has not spiked above the threshold.

If any condition fails, `is_healthy()` returns `False`, the watchdog escalates, and the system health state degrades. Existing tracks coast forward until the sensor recovers or the system operator intervenes.

**Error tracking:**

The thread maintains a rolling count of exceptions. A sudden spike — e.g., 5 errors in 10 scans — indicates an unstable connection and is reported before a complete disconnect occurs.

---

## Configuration Reference

All parameters live in `configs/default.yaml` and `configs/vessel_profile.yaml`.

**LiDAR hardware (`default.yaml → lidar`):**

```yaml
lidar:
  port: /dev/ttyUSB0
  baudrate: 1000000
  timeout_s: 3.0
  max_reconnect_attempts: 10
  reconnect_backoff_s: 2.0
```

**Preprocessing (`default.yaml → preprocessing`):**

```yaml
preprocessing:
  min_range_m: 0.3
  max_range_m: 30.0
  min_quality_threshold: 10     # set to 0 to disable intensity gate
```

**FOV masking (`vessel_profile.yaml`):**

```yaml
fov_masks:
  - {start_deg: 178.0, end_deg: 182.0}   # aft mast
```

**Noise filter (`default.yaml → noise_filter`):**

```yaml
noise_filter:
  cell_size_m: 0.25
  min_threshold: 5
  max_threshold: 8
  cell_ttl_scans: 15
```

**Segmentation (`default.yaml → segmentation`):**

```yaml
segmentation:
  jump_distance_m: 0.5
  adaptive_k: 0.02
  min_points: 3
  max_points: 2000
  min_arc_span_deg: 0.5
  merge_distance_m: 1.0
```

**Extraction (`default.yaml → extraction`):**

```yaml
extraction:
  max_obstacle_radius_m: 15.0
  reference_points: 30
```
