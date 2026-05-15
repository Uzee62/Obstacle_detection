# Configuration Reference

The system uses two YAML configuration files with clearly separated responsibilities. Understanding which file to edit — and why — prevents misconfiguration at sea.

---

## Table of Contents

- [Two-File Strategy](#two-file-strategy)
- [default.yaml — Runtime Parameters](#defaultyaml--runtime-parameters)
- [vessel_profile.yaml — Hardware Calibration](#vessel_profileyaml--hardware-calibration)
- [How Config Is Loaded](#how-config-is-loaded)
- [Tuning Guide](#tuning-guide)

---

## Two-File Strategy

| File | Changes when? | Who edits it? |
|---|---|---|
| `default.yaml` | Algorithm performance needs tuning; sea conditions change; new thresholds required | System operator or engineer |
| `vessel_profile.yaml` | The LiDAR is remounted; IMU orientation changes; a new vessel is commissioned | Commissioning engineer; never underway |

**Why separate?**

Physical hardware calibration (`vessel_profile.yaml`) describes facts about this specific vessel — the sensor mounting angle, the blind sectors caused by the mast. These values should be set once during commissioning and treated as read-only during normal operation. Accidentally editing them has safety implications.

Algorithm parameters (`default.yaml`) are tunable tradeoffs — more aggressive noise rejection vs. less clutter, wider association gates vs. fewer false merges. These can be adjusted safely without understanding the vessel hardware.

---

## default.yaml — Runtime Parameters

### `lidar` — Hardware Connection

```yaml
lidar:
  port: /dev/ttyUSB0
  baudrate: 1000000
  timeout_s: 3.0
  max_reconnect_attempts: 10
  reconnect_backoff_s: 2.0
```

| Parameter | Description |
|---|---|
| `port` | Serial device path. Change if the OS assigns a different path (e.g., `/dev/ttyUSB1`). |
| `baudrate` | Fixed at 1,000,000 baud by the RPLidar S2 firmware. Do not change. |
| `timeout_s` | Seconds to wait for a response before declaring the port unresponsive. |
| `max_reconnect_attempts` | How many times the driver retries after a connection loss before giving up. |
| `reconnect_backoff_s` | Pause between reconnect attempts. Prevents hammering the serial port. |

---

### `preprocessing` — Point Validity Gates

```yaml
preprocessing:
  min_range_m: 0.3
  max_range_m: 30.0
  min_quality_threshold: 10
```

| Parameter | Description |
|---|---|
| `min_range_m` | Points closer than this are rejected — they are own-vessel reflections. Increase if the vessel's hull is wider. |
| `max_range_m` | Points beyond this are rejected — the sensor's accuracy degrades at long range. The RPLidar S2 is rated to 30 m. |
| `min_quality_threshold` | Points with quality below this are rejected. Set to `0` to disable the intensity gate entirely. Raise this in heavy rain or fog to reduce spurious returns. |

---

### `noise_filter` — Adaptive Sea-Clutter Rejection

```yaml
noise_filter:
  cell_size_m: 0.25
  min_threshold: 5
  max_threshold: 8
  cell_ttl_scans: 15
  ema_alpha: 0.1
```

| Parameter | Description |
|---|---|
| `cell_size_m` | Grid resolution. Smaller cells → finer spatial discrimination but more memory usage. 0.25 m is suitable for vessel-scale obstacles. |
| `min_threshold` | A point must appear in its cell at least this many times to survive. The adaptive logic will not go below this floor. |
| `max_threshold` | Ceiling on the adaptive threshold. In very rough seas, the system will not raise the bar beyond this. |
| `cell_ttl_scans` | A cell that receives no hits for this many scans is evicted. Prevents the grid filling with stale returns from obstacles that have moved. |
| `ema_alpha` | Smoothing factor for threshold adaptation (0 = no adaptation, 1 = instant). 0.1 gives gradual, stable adaptation. |

**Tuning for rough seas:** If wave clutter is passing through the filter, increase `min_threshold` and `max_threshold`.

**Tuning for calm water:** If genuine distant obstacles are being rejected, decrease `min_threshold`.

---

### `segmentation` — Obstacle Grouping

```yaml
segmentation:
  jump_distance_m: 0.5
  adaptive_k: 0.02
  min_points: 3
  max_points: 2000
  min_arc_span_deg: 0.5
  merge_distance_m: 1.0
```

| Parameter | Description |
|---|---|
| `jump_distance_m` | Base distance gap between adjacent points that triggers a new segment. Increase in open water where the natural point spacing is larger. |
| `adaptive_k` | Per-metre increase to the jump distance threshold. Accounts for natural spreading of adjacent points at range. |
| `min_points` | Segments with fewer points than this are discarded. Raise to suppress single-point noise; lower to detect very small objects. |
| `max_points` | Segments with more points than this are discarded (harbour walls, jetties). |
| `min_arc_span_deg` | Segments spanning less than this arc are discarded. Eliminates single-return noise even if `min_points` passes. |
| `merge_distance_m` | Segments whose centroids are within this distance are merged. Handles hull fragmentation (rigging gaps, shadows). Increase for large vessels; decrease to avoid merging two close-proximity obstacles. |

---

### `extraction` — Observation Generation

```yaml
extraction:
  max_obstacle_radius_m: 15.0
  reference_points: 30
```

| Parameter | Description |
|---|---|
| `max_obstacle_radius_m` | Segments whose bounding radius exceeds this are discarded. Filters out fixed infrastructure (walls, piers) that cannot be tracked as discrete obstacles. |
| `reference_points` | The point count at which confidence saturates to its maximum contribution. A detection with ≥ this many points gets full point-confidence credit. |

---

### `tracking` — Kalman Filter and Lifecycle

```yaml
tracking:
  process_noise_position: 0.1
  process_noise_velocity: 0.5
  measurement_noise: 0.3
  mahalanobis_gate: 9.21
  hits_to_confirm: 3
  miss_to_coast: 10
  miss_to_dead_tentative: 3
  miss_to_dead_coasting: 15
  dynamic_speed_threshold_ms: 0.15
```

| Parameter | Description |
|---|---|
| `process_noise_position` | How much the Kalman filter trusts the constant-velocity model for position (metres). Increase for highly manoeuvring targets. |
| `process_noise_velocity` | How much velocity can deviate from the model (m/s). Increase for aggressively manoeuvring targets. |
| `measurement_noise` | LiDAR centroid accuracy assumption (metres). Increase if centroids are observed to be noisy. |
| `mahalanobis_gate` | Chi-squared threshold for association gating (9.21 = 99.5% CI, 2-DOF). Raise to associate across larger distances; lower to prevent cross-track association. |
| `hits_to_confirm` | Consecutive hits required to promote TENTATIVE → CONFIRMED. Raise to suppress spurious tracks. |
| `miss_to_coast` | Consecutive misses before CONFIRMED → COASTING. Raise to keep tracks alive longer in poor visibility. |
| `miss_to_dead_tentative` | Consecutive misses to prune a TENTATIVE track. |
| `miss_to_dead_coasting` | Consecutive misses to prune a COASTING track. |
| `dynamic_speed_threshold_ms` | Speed above which a track is classified as dynamic (moving). 0.15 m/s ignores GPS and tide drift. |

---

### `fusion` — Main Loop Rate

```yaml
fusion:
  loop_rate_hz: 10.0
```

The main fusion loop pace. The Kalman filter `dt` is derived from this value. Changing it requires retuning `process_noise_position` and `process_noise_velocity` accordingly.

---

### `interfaces` — Output

```yaml
interfaces:
  output_path: ~/Obstacle_Detection/world_model.json
```

Path where the JSON publisher writes each cycle. The directory is created if it does not exist. The `~` is expanded to the current user's home directory.

---

### `health` — Monitoring Thresholds

```yaml
health:
  max_scan_gap_s: 1.0
  latency_warn_ms: 80.0
  stats_window_size: 100
  error_spike_threshold: 3
```

| Parameter | Description |
|---|---|
| `max_scan_gap_s` | Seconds without a new scan before the watchdog declares a fault. |
| `latency_warn_ms` | Mean end-to-end latency that triggers a warning log. At 10 Hz there is 100 ms per cycle; 80 ms is an 80% utilisation warning. |
| `stats_window_size` | Rolling window depth for the health monitor. |
| `error_spike_threshold` | Error count in the recent window that triggers a watchdog fault. |

---

## vessel_profile.yaml — Hardware Calibration

### `vessel` — Identification

```yaml
vessel:
  name: "RV_EXAMPLE"
  lidar_height_m: 3.5
```

| Parameter | Description |
|---|---|
| `name` | Vessel name, used in log files and `sensor_id` fields. |
| `lidar_height_m` | Height of the LiDAR above the waterline. Used for future wave-height compensation. |

---

### `lidar_mounting` — Sensor Geometry

```yaml
lidar_mounting:
  angle_offset_deg: 0.0
  forward_offset_m: 1.2
  starboard_offset_m: 0.0
```

| Parameter | Description |
|---|---|
| `angle_offset_deg` | Rotational offset of the LiDAR from the vessel bow. If the sensor is mounted facing 5° to port, enter `-5.0`. Applied during coordinate transform. |
| `forward_offset_m` | Distance of the LiDAR forward from the vessel's centre of rotation (CoR). Used to correct obstacle positions to the CoR frame. |
| `starboard_offset_m` | Lateral offset (positive = starboard / right). |

**Important:** These offsets are applied during the polar → Cartesian conversion. Incorrect values shift all obstacle positions systematically — the symptom is stationary obstacles appearing to drift when the vessel turns.

---

### `fov_masks` — Blind Sectors

```yaml
fov_masks:
  - {start_deg: 178.0, end_deg: 182.0}
```

Each entry defines an angular sector where the LiDAR's own vessel returns are blanked out in the preprocessor.

| Parameter | Description |
|---|---|
| `start_deg` | Start of the blind sector in degrees (0 = bow, clockwise). |
| `end_deg` | End of the blind sector. |

**Wrap-around is supported:** A mask from `350.0` to `10.0` correctly blanks the aft sector that spans through 0°.

**To identify blind sectors during commissioning:** Run the system at the dock and plot the raw point cloud. Any bright arc that appears fixed relative to the vessel (not the world) is a self-return that should be masked.

---

### `imu_mounting` — Orientation Correction (Future)

```yaml
imu_mounting:
  roll_offset_deg: 0.0
  pitch_offset_deg: 0.0
  yaw_offset_deg: 0.0
```

Mounting angle offsets for the IMU. These will be used by the future ego-motion compensation module to correct LiDAR point clouds for vessel roll, pitch, and yaw during scanning.

---

### `rtk_antenna` — GPS Lever Arm (Future)

```yaml
rtk_antenna:
  forward_m: 2.5
  starboard_m: 0.0
  up_m: 4.2
```

Position of the RTK GPS antenna relative to the vessel's CoR. Used by the future geo-referenced world model to convert vessel-frame obstacle positions to absolute NED coordinates.

---

## How Config Is Loaded

`config.py` implements a simple loader:

1. Parse `configs/default.yaml`.
2. Parse `configs/vessel_profile.yaml`.
3. Deep-merge the vessel profile into the defaults (vessel-specific values win on conflict).
4. Return the merged dictionary.

The config object is passed to every component at startup. **There is no runtime config reload** — a configuration change requires restarting the system. This is intentional: at-sea configuration changes are a significant source of incidents.

---

## Tuning Guide

### The system is producing too many ghost tracks (false obstacles)

1. Increase `noise_filter.min_threshold` — reject points that do not persist.
2. Increase `tracking.hits_to_confirm` — require more evidence before publishing a track.
3. Decrease `tracking.mahalanobis_gate` — prevent spurious detections from merging with real tracks.

### The system is losing tracks on real obstacles too quickly

1. Increase `tracking.miss_to_coast` — allow more coasting before the track degrades.
2. Increase `tracking.miss_to_dead_coasting` — give coasting tracks more time to recover.
3. Decrease `noise_filter.min_threshold` — avoid rejecting genuine low-reflectivity returns.

### Obstacle positions are inaccurate or drifting

1. Verify `vessel_profile.yaml` mounting offsets against physical measurements.
2. Decrease `tracking.measurement_noise` if the LiDAR is more accurate than assumed.
3. Decrease `tracking.process_noise_position` to weight measurements more than prediction.

### System latency is consistently high (> 80 ms)

1. Increase `noise_filter.cell_size_m` to reduce grid operations.
2. Decrease `preprocessing.max_range_m` to reduce point count.
3. Profile the pipeline with `LidarPipelineStats.process_time_ms` to identify the slow stage.
