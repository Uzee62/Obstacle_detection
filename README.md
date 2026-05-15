
## Architecture

┌─────────────────────────────────────────────────────────────────┐
│  HARDWARE LAYER                                                 │
│  RPLidar S2  (/dev/ttyUSB0, 1 Mbaud, 360°)                     │
└───────────────────────────┬─────────────────────────────────────┘
                            │ raw scan points (angle, distance, quality)
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  SENSOR LAYER  (background thread)                              │
│  ┌──────────┐  ┌─────────────┐  ┌────────────┐  ┌──────────┐  │
│  │  Driver  │→ │Preprocessor │→ │NoiseFilter │→ │Segmenter │  │
│  └──────────┘  └─────────────┘  └────────────┘  └────┬─────┘  │
│                                                       │         │
│                                               ┌───────▼──────┐  │
│                                               │  Extractor   │  │
│                                               └───────┬──────┘  │
└───────────────────────────────────────────────────────┼─────────┘
                                                        │ DetectionObservation[]
                                                        ▼
┌─────────────────────────────────────────────────────────────────┐
│  FUSION LAYER  (main thread @ 10 Hz)                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐ │
│  │  Associate  │→ │   Kalman    │→ │   Lifecycle Manager     │ │
│  │ (Hungarian) │  │  Predict /  │  │ TENTATIVE→CONFIRMED→    │ │
│  │ (Mahalano.) │  │   Update    │  │ COASTING→DEAD           │ │
│  └─────────────┘  └─────────────┘  └────────────┬────────────┘ │
└────────────────────────────────────────────────────┼────────────┘
                                                     │ WorldModel
                                                     ▼
┌─────────────────────────────────────────────────────────────────┐
│  OUTPUT & MONITORING                                            │
│  ┌──────────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │  JsonPublisher   │  │HealthMonitor │  │    Watchdog      │  │
│  │ (atomic file IO) │  │ (rolling 100 │  │ NOMINAL/DEGRADED │  │
│  │                  │  │   scans)     │  │ /CRITICAL        │  │
│  └──────────────────┘  └──────────────┘  └──────────────────┘  │
└─────────────────────────────────────────────────────────────────┘






# Maritime Perception System

A production-grade maritime obstacle detection pipeline designed for autonomous vessel navigation. The system ingests 360° LiDAR point clouds, filters sea-state clutter, segments and tracks obstacles, and publishes a stable world model that safety and navigation layers can consume.


  
## Overview

The system is built around a single design principle: **every sensor speaks the same language**. Regardless of whether the source is LiDAR, AIS, sonar, or radar, it produces a `DetectionObservation`. That common currency flows into a shared Kalman-filter tracker that maintains persistent obstacle identities across time, ultimately producing a `WorldModel` — an immutable snapshot of confirmed obstacles that safety and navigation layers consume.

---

## Objective

Build a modular, sensor-agnostic obstacle detection system for maritime
autonomous vessels. The system must:

- Detect and track obstacles in real time from onboard sensors
- Produce a clean, unified **World Model** that safety and navigation
  layers can consume without knowing anything about underlying sensors
- Be extensible — adding or removing a sensor should require no changes
  to the fusion, safety, or navigation code
- Run reliably in marine environments across varying sea states

The guiding principle throughout:

```
Raw sensor data must never directly drive navigation or safety decisions.
Every sensor produces observations. Only the fusion layer produces world objects.
```

---

## Current State 

### What is built and working

The full LiDAR-first perception pipeline is implemented and tested.

**Sensor layer — Slamtec RPLidar S2**

- Hardware driver with auto-reconnect on USB disconnect
- Background sensor thread — non-blocking, single-slot buffer, always
  returns the latest scan
- Health monitoring — scan gap detection, error count tracking

**Preprocessing**

- Range gate — rejects readings outside `[0.3m, 30.0m]`
- Explicit validity gate — rejects zero, NaN, and inf distances
- FOV mask — blanks configurable angular sectors (hull, mast, radar dome)
- Mounting angle offset — corrects for sensor rotation from vessel bow
- Polar → Cartesian conversion in vessel frame (bow = +x, port = +y)

**Marine noise filtering**

- Adaptive temporal persistence grid
- Grid cells accumulate hits across scans; only persistent cells are trusted
- Threshold adapts automatically to sea state via clutter density estimation
- Wave crests and spray appear for 1–2 scans and are suppressed
- Real obstacles appear consistently and pass through

**Segmentation**

- Jump-distance segmentation on angularly-ordered scan points
- Adaptive threshold scales with range (handles angular spreading at distance)
- Minimum point count and minimum arc span filters
- KD-tree single-linkage merge for fragmented segments (O(n log n))

**Obstacle extraction**

- Centroid and bounding circle radius per cluster
- Range and bearing computed and stored
- Confidence model: point count contribution × range penalty
- Oversized clusters discarded (walls, quay faces)
- Output: `DetectionObservation` — the universal sensor interface

**Fusion and tracking**

- Hungarian algorithm association (globally optimal, no ID swaps)
- Mahalanobis distance gating (chi-squared 99.5%, accounts for track uncertainty)
- Kalman filter — constant velocity model, state `[x, y, vx, vy]`
- Full track lifecycle: `TENTATIVE → CONFIRMED → COASTING → DEAD`
- Track score — continuous quality metric, decays on miss, grows on hit
- Coast mode — confirmed tracks continue to be predicted and published
  during brief occlusions with decaying confidence

**World model**

- `WorldObject` — position, velocity, heading, size, confidence,
  position uncertainty, dynamic flag, coasting flag, sensor sources
- `safety_radius_m` property = physical size + position uncertainty
- Published as atomic JSON write every cycle

**Health and monitoring**

- Rolling statistics: scan rate, latency, noise rejection %, object count
- Watchdog: NOMINAL / DEGRADED / CRITICAL states
- Rotating log file at `/tmp/maritime_perception.log`

**Configuration**

- `configs/default.yaml` — all runtime tunable parameters
- `configs/vessel_profile.yaml` — vessel geometry, FOV mask, mounting calibration
- Validated on startup — fails fast with clear error messages

### Verified performance

Tested on synthetic scans simulating three obstacles (two vessels, one buoy):

| Metric | Result |
|---|---|
| Mean cycle time | ~8–10ms |
| Max safe budget | 100ms (10Hz) |
| Track ID stability | Stable across 40 scans |
| Velocity estimation | ~2.0 m/s measured correctly |
| Confirmed tracks | Correct obstacles only by scan 10 |

---

## Architecture

```
RPLidar Hardware
      │
      ▼
sensors/lidar/driver.py          LidarScan
      │
      ▼
sensors/lidar/preprocessor.py   list[CartesianPoint]
      │
      ▼
sensors/lidar/noise_filter.py   list[CartesianPoint]  (filtered)
      │
      ▼
sensors/lidar/segmentation.py   list[Segment]
      │
      ▼
sensors/lidar/extractor.py      list[DetectionObservation]
      │                                  ↑
      │                    universal sensor interface
      │                    all future sensors produce this
      ▼
fusion/tracker.py               Kalman + Hungarian + lifecycle
      │
      ▼
fusion/builder.py               WorldModel
      │
      ▼
interfaces/json_publisher.py    /tmp/world_model.json
```

### Key design rule — the sensor contract

Every sensor module implements `AbstractSensorPipeline`:

```python
class AbstractSensorPipeline:
    def latest(self) -> list[DetectionObservation]: ...
    def is_healthy(self) -> bool: ...
    def start(self) -> None: ...
    def stop(self) -> None: ...
```

The fusion engine calls `latest()` on every sensor every cycle and knows
nothing else about them. Adding AIS means writing one new module. Zero
changes to fusion, world model, safety, or navigation.

### Project structure

```
maritime_perception/
├── pyproject.toml
├── README.md
├── test_pipeline.py               smoke test, no hardware needed
│
├── configs/
│   ├── default.yaml               runtime parameters (tune per voyage)
│   └── vessel_profile.yaml        vessel geometry and calibration
│
└── maritime_perception/
    ├── main.py                    entry point, fusion loop, shutdown
    ├── config.py                  YAML loader with validation
    ├── logging_config.py          rotating file + console logging
    │
    ├── models/                    type contracts — no logic
    │   ├── common.py              Header, now_ns(), SensorSource
    │   ├── observation.py         DetectionObservation
    │   ├── pose.py                VesselPose (RTK + IMU)
    │   ├── track.py               ObjectTrack, TrackState
    │   └── world_model.py         WorldObject, WorldModel
    │
    ├── sensors/
    │   ├── base.py                AbstractSensorPipeline interface
    │   └── lidar/
    │       ├── driver.py          RPLidar S2, auto-reconnect
    │       ├── preprocessor.py    range gate, FOV mask, polar→cart
    │       ├── noise_filter.py    adaptive temporal persistence grid
    │       ├── segmentation.py    jump-distance + KD-tree merge
    │       ├── extractor.py       cluster → DetectionObservation
    │       ├── pipeline.py        stage orchestrator
    │       └── sensor_thread.py   background thread wrapper
    │
    ├── fusion/
    │   ├── motion_model.py        Kalman filter (F, H, Q, R)
    │   ├── association.py         Hungarian + Mahalanobis gate
    │   ├── tracker.py             lifecycle management
    │   └── builder.py             confirmed tracks → WorldModel
    │
    ├── interfaces/
    │   └── json_publisher.py      atomic JSON file output
    │
    └── health/
        ├── monitor.py             rolling stats
        └── watchdog.py            fault detection, DEGRADED flag
```

---

## Getting Started

### Requirements

- Python 3.11+
- Slamtec RPLidar S2 connected via USB
- Linux recommended (tested on Ubuntu / Raspberry Pi OS)

### Install

```bash
git clone <repo>
cd maritime_perception

python3.11 -m venv .venv
source .venv/bin/activate

pip install -e .
```

### Run the smoke test (no hardware needed)

```bash
python test_pipeline.py
```

Expected output: 40 scans processed, confirmed tracks, all assertions pass.

### Calibrate your FOV mask

Before first hardware run, find which angles return your own hull:

```bash
python -c "
from rplidar import RPLidar
lidar = RPLidar('/dev/ttyUSB0')
for scan in lidar.iter_scans():
    for quality, angle, distance in scan:
        if quality > 0 and 0 < distance < 500:
            print(f'angle={angle:.1f} dist={distance}mm')
    break
lidar.stop(); lidar.disconnect()
"
```

Add the resulting angles to `configs/vessel_profile.yaml`:

```yaml
fov_mask:
  - [178.0, 182.0]   # aft mast — replace with your readings
```

### Run on hardware

```bash
# check your port
ls /dev/ttyUSB*

# set it in configs/default.yaml
lidar:
  port: "/dev/ttyUSB0"

# run
maritime-perception
```

World model is written to `/tmp/world_model.json` every cycle.  
Watch it live:

```bash
watch -n 0.5 cat /tmp/world_model.json
```

### Example world model output

```json
{
  "header": {
    "timestamp_ns": 123456789000,
    "sensor_id": "fusion",
    "frame_id": "vessel"
  },
  "scan_id": 42,
  "latency_ms": 8.3,
  "object_count": 2,
  "objects": [
    {
      "id": 1,
      "position": {"x": 11.6, "y": 3.1},
      "velocity": {"x": -1.9, "y": -0.1},
      "heading_deg": 183.0,
      "range_m": 12.0,
      "bearing_deg": 15.0,
      "speed_ms": 1.9,
      "size_m": 0.8,
      "safety_radius_m": 1.1,
      "confidence": 0.94,
      "position_std_m": 0.3,
      "dynamic": true,
      "coasting": false,
      "sources": ["LIDAR"]
    }
  ]
}
```

---

## Timestamps

All internal timestamps use `time.monotonic_ns()` — a monotonically
increasing nanosecond counter immune to system clock corrections (NTP,
daylight saving, leap seconds). This is critical for future sensor fusion
where observations from multiple sensors must be correctly ordered and
interpolated.

Timestamps are set **at hardware acquisition** in the driver, not during
processing. Every message carries its original acquisition timestamp
unchanged through the entire pipeline.

---

## Configuration Reference

### `configs/default.yaml` — tune per voyage

| Section | Key | Default | Description |
|---|---|---|---|
| lidar | port | `/dev/ttyUSB0` | Serial port |
| lidar | timeout_s | 5.0 | Connection timeout |
| preprocessing | range_min_m | 0.3 | Minimum valid range |
| preprocessing | range_max_m | 30.0 | Maximum valid range |
| noise_filter | min_hits_base | 3 | Base persistence threshold |
| noise_filter | min_hits_max | 8 | Max threshold (rough sea) |
| segmentation | jump_distance_m | 0.5 | Gap threshold for new segment |
| tracking | gate_mahalanobis | 9.21 | Association gate (99.5%) |
| tracking | min_hits_confirm | 3 | Scans to confirm a track |
| tracking | max_misses_confirmed | 10 | Misses before coast mode |

### `configs/vessel_profile.yaml` — set once per vessel

| Key | Description |
|---|---|
| lidar_mounting.angle_offset_deg | Sensor rotation from bow |
| lidar_mounting.forward_m | Sensor offset ahead of CoR |
| fov_mask | Angular sectors to blank (hull returns) |
| imu_mounting | IMU rotation offsets |
| rtk_mounting | RTK antenna position |

---

## Known Limitations

**Ego-motion compensation not yet implemented.**  
On a stationary or very slow vessel this is fine. On a moving vessel,
stationary obstacles (buoys, markers) will appear to move in the tracker
because vessel motion is not subtracted. This is Phase 2 work.

**Static obstacle map not yet implemented.**  
Harbour walls, quay faces, and shoreline will generate persistent tracks
that pollute the world model with static structure. A static object filter
(anything with near-zero velocity for >30 seconds) will address this.

**Association breaks in dense crossing scenarios.**  
The Hungarian algorithm handles well-separated targets correctly. In
very dense environments with many simultaneous crossings, track score
degradation may still occur. The Mahalanobis gate reduces this
significantly but does not eliminate it entirely.

**Single sensor.**  
The world model currently reflects LiDAR detections only. AIS would add
cooperative vessel identity. FLS would add underwater hazards. Neither
is implemented yet.

---

## Roadmap

### Phase 2 — Ego-motion and stability

- [ ] IMU driver (`sensors/imu_driver.py`)
- [ ] RTK driver (`sensors/rtk_driver.py`)
- [ ] Pose estimator — fuses RTK + IMU → `VesselPose`
- [ ] Ego-motion compensation in preprocessor
- [ ] Variable dt Kalman filter (handles timing jitter)
- [ ] Static obstacle filter — move zero-velocity confirmed
      tracks to a static map layer
- [ ] Deep config merge (currently shallow)

### Phase 3 — Reliability and observability

- [ ] Replay logger — record raw scans for post-voyage debugging
- [ ] Polar visualiser — matplotlib real-time obstacle display
- [ ] Unit test suite (per module)
- [ ] ZeroMQ publisher for multi-consumer real-time distribution
- [ ] Prometheus metrics endpoint
- [ ] systemd service file for vessel computer deployment
- [ ] L-shape fitting for close-range vessel hull geometry

### Phase 4 — AIS integration

- [ ] AIS driver — NMEA sentence parser
- [ ] AIS decoder — MMSI, COG, SOG, vessel class
- [ ] AIS sensor pipeline — produces `DetectionObservation`
- [ ] Multi-sensor fusion — earliest-timestamp cycle header
- [ ] Sensor-specific confidence weighting in tracker
- [ ] AIS-LiDAR track correlation

### Phase 5 — Forward Looking Sonar

- [ ] FLS driver
- [ ] CFAR acoustic noise filter
- [ ] FLS obstacle extractor
- [ ] Underwater and semi-submerged hazard layer

### Phase 6 — MBES and terrain

- [ ] MBES driver
- [ ] Bathymetry processor — seabed mapping
- [ ] Static terrain layer — grounding risk
- [ ] Terrain-constrained path planning interface

### Phase 7 — Safety and navigation interface

- [ ] TCPA / DCPA computation from world model
- [ ] COLREGS-aware collision risk scoring
- [ ] Safety layer interface for autonomous manoeuvre decisions

---

## Dependencies

| Package | Version | Purpose |
|---|---|---|
| numpy | ≥ 2.0 | Kalman filter matrix math |
| scipy | ≥ 1.13 | Hungarian algorithm, KD-tree |
| pyyaml | ≥ 6.0 | Configuration loading |
| rplidar-roboticia | ≥ 0.9.5 | RPLidar hardware driver |

All other functionality uses Python standard library only.

---

## Adding a New Sensor

1. Create `sensors/<name>/` package
2. Implement `driver.py` — hardware IO, outputs typed scan model
3. Implement `pipeline.py` — processing stages, outputs `list[DetectionObservation]`
4. Implement `sensor_thread.py` — extends `AbstractSensorPipeline`
5. In `main.py`, add one line:

```python
sensors = [
    lidar_thread,
    YourNewSensorThread(config),   # ← this line only
]
```

Zero changes to fusion, world model, safety, or navigation.

---

