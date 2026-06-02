# Maritime Perception — Developer Design Document (DDD)

Audience: software engineers maintaining and extending the obstacle-detection system. Target read time: 15–30 minutes.

---

## 1. System Purpose

**Objective.** Detect, track, and continuously publish a deduplicated list of obstacles around a small vessel ("vatoz") at 10 Hz, from a single 2D LiDAR. The output is a `WorldModel` consumed by an external safety/navigation layer (not part of this repo).

**Operational environment.** Maritime, near-shore. The LiDAR is mounted ~0.9 m above the waterline. Sea spray, wave crests, sunlight glint, and multipath off the hull produce hundreds of false returns per scan; the architecture is built around rejecting those without losing real obstacles (vessels, buoys, pilings).

**Inputs.**
- Raw 360° polar scans from a Slamtec RPLidar S2 over serial, surfaced by a C++ subprocess (tools/lidar_publisher/main.cpp).
- Two YAML config files: configs/default.yaml (tunable runtime params) and configs/vessel_profile.yaml (vessel-specific calibration: mounting, FOV mask).

**Outputs.**
- A `WorldModel` JSON snapshot written atomically to `/tmp/world_model.json` each cycle by interfaces/json_publisher.py.
- Rolling health metrics and a system-health flag (`NOMINAL` / `DEGRADED` / `CRITICAL`).

**Key capabilities.**
- Adaptive marine noise rejection (persistence grid that self-tunes to sea state).
- Multi-object tracking with Kalman filter + Hungarian association.
- Track lifecycle with coast mode so vessels do not disappear during brief occlusions.
- Crash isolation between the SDK driver and Python via subprocess boundary.
- Sensor-agnostic fusion interface (`AbstractSensorPipeline`) ready for AIS / FLS / MBES.

---

## 2. System Architecture

```
                        +-----------------------------------------+
                        |       configs/default.yaml              |
                        |       configs/vessel_profile.yaml       |
                        +----------------+------------------------+
                                         | load_config()
                                         v
   +-------------------------------------------------------------------+
   |                           main.py                                 |
   |  (orchestrator; runs the 10 Hz fusion loop on the main thread)    |
   +-------------------------------------------------------------------+
           |                                          |
           | start()                                  | update()
           v                                          v
   +--------------------+               +-----------------------------+
   | LidarSensorThread  |               |      FusionTracker          |
   | (background)       |               | +-------------------------+ |
   | +----------------+ |  latest()     | |  associate (Hungarian)  | |
   | | RPLidarDriver  | |  (non-block)  | |  motion_model (Kalman)  | |
   | | ^ subprocess   | | ----------->  | |  lifecycle              | |
   | | lidar_publisher| |  list[DetObs] | +------------+------------+ |
   | +----------------+ |               |              v              |
   | +----------------+ |               |      WorldModelBuilder      |
   | | LidarPercep-   | |               +------------+----------------+
   | | tionPipeline   | |                            | WorldModel
   | |  preprocessor  | |                            v
   | |  noise_filter  | |               +-----------------------------+
   | |  segmenter     | |               |  JsonPublisher (atomic)     |
   | |  extractor     | |               |  /tmp/world_model.json      |
   | +----------------+ |               +-----------------------------+
   +---------+----------+
             | is_healthy(), scan_gap_ms
             v
   +-----------------------+         +-----------------------------+
   |   SensorWatchdog      |         |      HealthMonitor          |
   | (NOMINAL/DEGRADED/    |         | (rolling latency, rates,    |
   |   CRITICAL)           |         |  noise rejection %)         |
   +-----------------------+         +-----------------------------+
```

**Modules.**

| Module | Purpose | Inputs | Outputs | Depends on |
|---|---|---|---|---|
| tools/lidar_publisher/ (C++) | Owns Slamtec SDK / serial protocol. Crash isolation. | Serial port | Line-based SCAN packets on stdout | Slamtec RPLidar SDK |
| sensors/lidar/driver.py | Spawns publisher subprocess, parses SCAN frames, auto-reconnects. | publisher stdout | LidarScan (list of RawScanPoint) | subprocess |
| sensors/lidar/preprocessor.py | Validity/range/intensity gate, FOV mask, mounting offset, polar->Cartesian. | LidarScan | list[CartesianPoint] | — |
| sensors/lidar/noise_filter.py | Adaptive temporal-persistence grid for marine clutter. | list[CartesianPoint] | filtered list[CartesianPoint] | — |
| sensors/lidar/segmentation.py | Jump-distance split + KD-tree single-linkage merge. | filtered points | list[Segment] | numpy, scipy.spatial |
| sensors/lidar/extractor.py | Centroid, bounding circle, range/bearing, confidence. | list[Segment] | list[DetectionObservation] | — |
| sensors/lidar/pipeline.py | Stateful orchestration of the four LiDAR stages; per-scan diagnostics. | LidarScan | list[DetectionObservation] + LidarPipelineStats | the four stages above |
| sensors/lidar/sensor_thread.py | Background thread; single-slot buffer (latest-only). Implements AbstractSensorPipeline. | driver + pipeline | latest() -> observations, is_healthy() | threading, queue |
| fusion/association.py | Hungarian assignment with Mahalanobis gating. | observations + tracks | matched / unmatched lists | scipy.optimize |
| fusion/motion_model.py | 2D constant-velocity Kalman filter and matrix factories. | state, observation | predicted/updated state + covariance | numpy |
| fusion/tracker.py | Multi-object tracker, lifecycle, score model. | per-cycle observations | active ObjectTracks | association, motion_model |
| fusion/builder.py | Boundary between internal tracks and public output. | tracks | WorldModel | models |
| interfaces/json_publisher.py | Atomic JSON snapshot file. | WorldModel | JSON file | — |
| health/monitor.py | Rolling-window observability (rates, latency, noise %). | per-cycle CycleStats | log summaries | — |
| health/watchdog.py | Fault detection -> SystemHealth enum. | LidarSensorThread.is_healthy() | WatchdogStatus | — |
| config.py | Loads + validates merged YAML with fail-fast errors. | YAML files | dict | pyyaml |

Empty placeholder directories `sensors/AIS/`, `sensors/FLES/`, `sensors/MBES/` exist but contain no code — see §9.

---

## 3. End-to-End Data Flow

A single 360° scan moves through the system as follows.

| Stage | Owner | Input | Processing | Output |
|---|---|---|---|---|
| 1. Acquire | lidar_publisher (C++) | serial bytes | Slamtec SDK parses scan, emits SCAN <ns> + points + END on stdout | text frames |
| 2. Parse | RPLidarDriver._read_one_scan | publisher stdout | line-by-line parse; auto-reconnect on transient error | LidarScan(header, points, scan_id) |
| 3. Preprocess | LidarPreprocessor.process | LidarScan | finite/positive check -> range gate -> intensity gate -> mounting offset -> FOV mask -> polar->Cartesian | list[CartesianPoint] (vessel frame) |
| 4. Noise filter | AdaptiveNoiseFilter.update | Cartesian points + scan_id | vote into grid cells, compute clutter density, adapt min_hits threshold (EMA-smoothed), evict TTL-expired cells, keep only points whose cell >= threshold | filtered points |
| 5. Segment | JumpDistanceSegmenter.segment | filtered points (angular order preserved) | split on adaptive Euclidean gap, filter by min_points / min_arc_deg / max_points, KD-tree single-linkage merge by centroid | list[Segment] |
| 6. Extract | ObstacleExtractor.extract | segments + header | centroid, bounding-circle radius, range, bearing, confidence (point-count x range-penalty); discard oversized clusters | list[DetectionObservation] |
| 7. Buffer | LidarSensorThread._drain_and_put | observations | overwrite single-slot queue (drop older if not yet consumed) | queue holds latest only |
| 8. Collect | main.py fusion loop | lidar_thread.latest() | extend all_observations (future: also AIS, FLS) | merged observation list |
| 9. Predict | FusionTracker._predict_all | active tracks | x = F·x; P = F·P·F^T + Q per track; age_scans += 1 | predicted track states |
| 10. Associate | associate() | observations + predicted tracks | build cost matrix using squared Mahalanobis; gate at 9.21; scipy.optimize.linear_sum_assignment | matched / unmatched lists |
| 11. Update | FusionTracker._update_track | matched pair | Kalman update (Joseph-form covariance), size EMA, score reward, recover from COASTING | updated track |
| 12. Spawn / age | _spawn, miss-count bump | unmatched obs / tracks | new TENTATIVE track with make_P_init(); bump misses on unmatched | track set |
| 13. Lifecycle | _lifecycle | all tracks | TENTATIVE->CONFIRMED on min_hits_confirm; CONFIRMED->COASTING on max_misses_confirmed; COASTING->DEAD on cumulative misses; score decay on miss | pruned track set |
| 14. Build | WorldModelBuilder.build | tracks | keep only CONFIRMED/COASTING above min_confidence_output | WorldModel |
| 15. Publish | JsonPublisher.publish | WorldModel | serialise -> write tempfile -> os.replace | atomic JSON file |
| 16. Monitor | HealthMonitor.record, SensorWatchdog.check | CycleStats, sensor health | rolling metrics; system-health flag; log summary every 100 cycles | logs + status object |
| 17. Pace | main loop sleep | elapsed time | sleep to maintain configured loop_rate_hz; warn on overrun > 10 ms | next cycle |

---

## 4. Core Components

### 4.1 LiDAR Driver (RPLidarDriver)

**Purpose.** Talk to the Slamtec SDK without putting C++ FFI complexity inside Python.

**Key classes.** RPLidarDriver, dataclasses RawScanPoint, LidarScan.

**Key responsibilities.** Subprocess lifecycle (spawn, stdin-close shutdown, terminate/kill escalation); per-scan text protocol parsing; auto-reconnect up to max_reconnect_attempts; async stderr forwarding into the Python logger.

**Important design decision — subprocess boundary.** A segfault inside the Slamtec SDK kills only the publisher process; the perception system reconnects rather than crashing. It also makes debugging trivial — the binary runs standalone. This is the explicitly stated trade-off in driver.py lines 15–23.

### 4.2 LiDAR Pipeline (LidarPerceptionPipeline)

**Purpose.** Convert one raw scan to standardised observations and record per-scan diagnostics. Pure orchestration — no perception logic of its own.

**Key classes.** LidarPerceptionPipeline, LidarPipelineStats.

**Important design decision — split into 4 stateless or single-stateful stages.** Each stage has its own *Config dataclass with from_config(cfg), which keeps YAML parsing out of the algorithms and makes each stage independently unit-testable.

### 4.3 Adaptive Noise Filter

**Purpose.** Reject transient marine clutter (waves, spray, glint) that would otherwise spawn ghost tracks.

**Key classes.** AdaptiveNoiseFilter, internal _Cell.

**Important design decisions.**
- **Persistence grid over per-point statistics**: real obstacles re-illuminate the same cell across scans; clutter does not. Cell hit-count is a cheap, robust temporal feature.
- **Self-adapting threshold**: the threshold is the *base* in calm conditions and rises towards min_hits_max as the fraction of cells that are "new this scan" increases. An EMA on the threshold itself (clutter_adapt_rate) prevents jitter when sea state oscillates.
- **Vessel-frame grid is a known limitation**: a moving vessel makes static obstacles drift between cells. The module's docstring flags ego-motion compensation as a prerequisite for underway operation — see §8.

### 4.4 Segmentation

**Purpose.** Group reliable points into obstacle candidates.

**Key classes.** JumpDistanceSegmenter, Segment, SegmentationConfig.

**Important design decisions.**
- **Adaptive jump threshold** `jump_distance_m + range × adaptive_k` compensates for the natural angular spreading of points at long range (0.5° at 20 m >> 0.5° at 5 m).
- **KD-tree single-linkage merge** (scipy.spatial.cKDTree.query_pairs + union-find) handles fragmented hull returns in O(n log n), which matters because filtered scans can yield hundreds of micro-segments.

### 4.5 Obstacle Extractor

**Purpose.** Produce one DetectionObservation per cluster with an explainable confidence score.

**Key classes.** ObstacleExtractor, ExtractorConfig.

**Important design decisions.** Confidence is `pt_conf × range_penalty`, both linear and clamped. The docstring states an explicit preference for explainable heuristics over opaque ML so operators can reason about why a track has a given score after an incident.

### 4.6 Sensor Thread (LidarSensorThread)

**Purpose.** Decouple LiDAR acquisition (blocking, variable latency) from the deterministic 10 Hz fusion loop.

**Key classes.** LidarSensorThread implementing AbstractSensorPipeline.

**Important design decisions.**
- **Single-slot buffer (queue.Queue(maxsize=1))**: only the freshest observation set is exposed. The trade-off — possibly dropping a result if the main loop falls behind — is explicit: "a stale scan is worse than no scan."
- **Startup grace**: is_healthy() returns True for the first startup_grace_s (default 20 s, set after we observed the publisher needing motor warm-up + startScan retries — see commit cb2d635). This avoids spurious CRITICAL transitions during boot.
- **Errors are absorbed**: only the driver's terminal RuntimeError (gave up reconnecting) stops the thread; everything else is logged and retried after a 100 ms sleep.

### 4.7 Fusion Tracker

**Purpose.** Multi-object tracking across cycles.

**Key classes.** FusionTracker, ObjectTrack, TrackState, KalmanFilter.

**Key responsibilities.** Predict -> associate -> update -> lifecycle -> build world model. Owns next_id allocation, the active-track dict, and the shared (F, H, Q, R) Kalman matrices.

**Important design decisions.**
- **Kalman filter (constant-velocity, state [x, y, vx, vy])** — the industry standard for radar/maritime tracking; gives a principled covariance for Mahalanobis gating.
- **Hungarian assignment (linear_sum_assignment) over greedy nearest-neighbour** — prevents ID swaps when two tracks cross. O(n^3), negligible for the expected n <= ~20 tracks.
- **Mahalanobis chi-square gate at 9.21** (2-DOF, ~99.5%) — covariance-aware, not a fixed metric distance.
- **Coast mode** — CONFIRMED tracks that lose observations enter COASTING and continue to be predicted and published with decaying confidence, so a vessel briefly behind a headland doesn't vanish from the world model.
- **Track score [0,1]** with `hit_reward × (1 - score)` growth and `× miss_decay` decay — finer-grained than raw hit/miss counts. The safety layer can distinguish a 50/52 confirmed track from a 3/3 confirmed track via confidence.
- **One shared (F, H, Q, R) set** for all tracks — built once in __init__, reused.

### 4.8 World Model Builder

**Purpose.** Architectural boundary. Nothing downstream sees ObjectTrack — only WorldObject. This keeps perception and decision-making independently upgradeable (stated explicitly in world_model.py lines 8–12).

### 4.9 Health (Monitor + Watchdog)

**Two-way split, intentional.** The HealthMonitor answers *how is the system performing?* (rolling latency, scan rate, noise rejection %). The SensorWatchdog answers *is anything wrong?* and exposes a 3-level SystemHealth enum that the (external) safety layer is expected to consume.

### 4.10 JSON Publisher

**Purpose.** Boundary to consumers. Atomic write (tempfile + os.replace) so a partially-written file is never read. Output schema is rounded for readability; bitmask sources are emitted as a string list.

---

## 5. Runtime Architecture

**Threads.** Exactly two long-lived threads + one helper:

| Thread | Created in | Role |
|---|---|---|
| Main thread | process entry | runs the 10 Hz fusion loop, watchdog, monitor, publisher |
| lidar-sensor-thread | LidarSensorThread.start() | reads scans, runs LiDAR pipeline, drops result in single-slot queue |
| lidar-pub-stderr (daemon) | RPLidarDriver._connect_once | forwards publisher stderr lines into the Python logger |

Plus the external C++ lidar_publisher subprocess.

**Queues.** Only one inter-thread queue: `LidarSensorThread._buffer = queue.Queue(maxsize=1)`, drained non-blocking by latest(), overwritten on each new pipeline result. Communication with the C++ subprocess is via stdin (close-to-shutdown) and stdout (line-buffered text).

**Startup sequence (per main.py lines 69–141).**

```
1. setup_logging()                  -> console + rotating file at /tmp/maritime_perception.log
2. signal handlers                  -> SIGINT, SIGTERM flip _running = False
3. load_config()                    -> fail-fast YAML validation
4. construct components             -> driver, pipeline, sensor_thread, tracker,
                                       publisher, monitor, watchdog
5. lidar_thread.start()             -> driver.connect() (spawns C++ subprocess)
                                       then background thread begins reading
6. enter fusion loop                -> 10 Hz, see §3
```

**Shutdown sequence.** The `finally` block in main() calls lidar_thread.stop(), which sets _running=False, calls driver.disconnect() (close stdin -> wait -> terminate -> kill, escalating), joins the thread (3 s timeout), then logs the final monitor summary.

**Runtime flow.**

```
   +--------------- main thread (10 Hz) ---------------+
   |  t0 = now_ns()                                    |
   |  obs = lidar_thread.latest()      (non-blocking)  |
   |  world = tracker.update(obs, header, scan_id)     |
   |  world.latency_ms = elapsed_ms(t0)                |
   |  publisher.publish(world)                         |
   |  status = watchdog.check(lidar_thread)            |
   |  monitor.record(CycleStats(...))                  |
   |  sleep(1/loop_rate_hz - elapsed)                  |
   +---------------------------------------------------+
                       ^ latest()
                       |
   +---------- lidar-sensor-thread (free-running) -----+
   |  scan = driver.read_scan()        (blocks)        |
   |  obs  = pipeline.process(scan)                    |
   |  buffer.put(obs)  (drops old)                     |
   +---------------------------------------------------+
                       ^ stdout
                       |
   +--- lidar_publisher (C++ subprocess) --------------+
   |  SDK polls serial, emits SCAN frames              |
   +---------------------------------------------------+
```

---

## 6. Data Models

All in maritime_perception/models/. All have slots=True for memory/speed; Header is frozen.

### Header (common.py)
- **Purpose.** Metadata stamped on every message; immutable so original acquisition time is preserved end-to-end.
- **Fields.** timestamp_ns (monotonic), sensor_id, frame_id ("lidar" / "vessel" / "ned"), source (SensorSource bitmask).
- **Used in.** Every observation, every track, every world model, every world object.

### RawScanPoint (driver.py)
- **Purpose.** One unprocessed return as produced by the driver.
- **Fields.** angle_deg, distance_m, quality.
- **Used in.** LidarScan.points only; never crosses the preprocessor boundary.

### CartesianPoint (preprocessor.py)
- **Purpose.** Validated post-preprocess point in the vessel frame.
- **Fields.** angle_deg (mounting-corrected), distance_m, x, y, quality.
- **Used in.** Noise filter, segmentation. Lives only inside the LiDAR pipeline.

### DetectionObservation (observation.py)
- **Purpose.** *The universal sensor interface.* Every sensor module must produce exactly this, so fusion stays sensor-agnostic.
- **Fields.** header, position_x/y, size_m (bounding-circle radius), range_m, bearing_deg, point_count, confidence (clamped to [0,1] in __post_init__).
- **Used in.** Sensor thread output -> tracker input.

### ObjectTrack (track.py)
- **Purpose.** A persistent obstacle hypothesis maintained across scans.
- **Fields.** track_id, state (TrackState), state_vec [x, y, vx, vy], covariance 4x4, kalman (attached on spawn), size_m, confidence, score, hit_count, miss_count, age_scans, sources bitmask, last_update_ns. Derived: position_x/y, velocity_x/y, speed_ms, heading_deg, range_m, bearing_deg, is_dynamic (speed > 0.15 m/s), position_std_m.
- **Used in.** FusionTracker._tracks only. Internal — never published.

### WorldObject and WorldModel (world_model.py)
- **Purpose.** Public perception output. WorldModel is the single source of truth consumed by the safety/navigation layer.
- **WorldObject fields.** id, header, position_x/y, velocity_x/y, heading_deg, size_m, confidence, position_std_m, dynamic, coasting, sources. Property: safety_radius_m = size_m + position_std_m.
- **WorldModel fields.** header, objects, scan_id, latency_ms. Helpers: __len__, __iter__, closest().

### VesselPose (pose.py) — *defined, not used*
- Full NED pose + RTK fix quality. Scaffolding for the planned RTK/IMU ego-motion path (§9). No producer or consumer in current code.

---

## 7. Configuration

Source of truth: configs/default.yaml (tunables) and configs/vessel_profile.yaml (per-vessel calibration — must survive software updates). Loaded and validated by config.py.

| Group | Key parameters | Effect |
|---|---|---|
| LiDAR | port, baudrate, timeout_s, reconnect_delay_s, max_reconnect_attempts | Hardware connection; driver reconnect behaviour. |
| Preprocessing | range_min_m (0.30 — RPLidar S2 blind zone), range_max_m (30.0 — reliable detection range), min_intensity | Geometric and quality gating before anything else runs. |
| Noise filter | cell_size_m (0.25), min_hits_base (5) / min_hits_max (8), ttl_scans (15), grid_radius_m (35 — must exceed range_max_m), clutter_adapt_rate (0.1) | Sets the calm/rough-sea operating band; cell size trades resolution against persistence statistics. |
| Segmentation | jump_distance_m (0.5), adaptive_k (0.02), min_points (3), min_arc_deg (0.5), max_points (2000), merge_distance_m (1.0) | Cluster granularity; aggressive merge collapses fragmented hulls into one object. |
| Extraction | max_obstacle_radius_m (15 — discards walls), confidence_pts_ref (30), confidence_range_penalty (0.5) | Shape of the confidence curve. |
| Tracking | gate_mahalanobis (9.21 = chi-sq 99.5% @ 2-DOF), min_hits_confirm (3), max_misses_tentative (3), max_misses_confirmed (10), max_misses_coast (5), min_confidence_output (0.25), kalman.process_noise_pos/vel, kalman.measurement_noise | Sensitivity vs. stability trade-off; lower noise -> more spurious tracks. |
| Fusion / runtime | fusion.cycle_rate_hz (10), runtime.loop_rate_hz (10) | Drives Kalman dt = 1/rate. Must match runtime loop rate or motion model is wrong. |
| Health | max_scan_gap_s (5), startup_grace_s (20), max_latency_ms (80), stats_window_scans (100) | DEGRADED/CRITICAL thresholds and high-latency warnings. |
| Vessel profile | lidar_mounting.angle_offset_deg, forward_m, starboard_m; fov_mask (list of [start_deg, end_deg] sectors to blank); imu_mounting, rtk_mounting | Per-vessel calibration. fov_mask is the practical knob to silence self-returns from masts/rails. |

**Design decision — fail-fast validation.** _validate() checks the small set of inequalities that would silently produce wrong perception (range_min_m > 0, range_max_m > range_min_m, hit thresholds ordered correctly) and aborts startup with an aggregated message. Stated rationale: *"fail fast on startup with clear errors rather than silently using wrong defaults at sea."*

---

## 8. Known Limitations

| Limitation | Origin | Operational impact |
|---|---|---|
| **Single LiDAR; no horizontal redundancy.** | Only one RPLidarDriver is built in main.py. A CRITICAL watchdog state takes the world model offline. | No detection while the sensor is down. Safety layer must stop autonomous ops. |
| **No ego-motion compensation.** | Noise filter grid is in vessel/sensor frame (noise_filter.py lines 38–43). VesselPose is defined but unused; no IMU/RTK driver exists. | When the vessel is underway, stationary objects (buoys, piles) drift between cells and may fail the persistence threshold -> missed static obstacles. Filter is sound only at rest or very low speed. |
| **2D only.** | RPLidar S2 is a 2D scanner mounted at fixed height (0.9 m above waterline). | Objects below the scan plane (low debris, partly submerged) or significantly above it are invisible. Roll/pitch will alter the slice geometry. |
| **Constant-velocity motion model.** | make_F(dt) is purely CV. | Sharp turns / sudden accelerations cause transient prediction error -> wider gate excursions, occasional broken tracks. |
| **Detection range capped at 30 m.** | preprocessing.range_max_m. | Reaction time for fast oncoming traffic is limited by this range. |
| **Self-returns require manual FOV calibration.** | fov_mask is hand-edited per vessel. | Forgotten sector after rig change -> persistent ghost tracks. |
| **Vessel frame only.** | No conversion to NED in the output. | Downstream consumers must combine with pose externally to get a world-frame model. |
| **POSIX-only atomic write and /tmp paths.** | os.replace + hard-coded /tmp defaults in main.py. | Won't run as-is on bare Windows; intended target is Linux on the vessel. |
| **No multi-hypothesis tracking / JPDA.** | Hungarian gives a single best assignment per cycle. | Dense, ambiguous clutter (raft of buoys) can briefly cause swaps despite the gate. |
| **No persistence across restarts.** | Tracker state and noise grid live in memory only. | After a restart, every obstacle must re-confirm (min_hits_confirm scans). |

---

## 9. Future Extension Points

These are **supported by the existing architecture** — code or scaffolding is already in place.

| Extension | What's already there |
|---|---|
| **Additional sensors (AIS, FLS, MBES, RADAR)** | AbstractSensorPipeline contract (base.py) + SensorSource bitmask + DetectionObservation as the universal type. Empty directories sensors/AIS/, sensors/FLES/, sensors/MBES/ mark the intended layout. main.py documents the 3-step add procedure in its module docstring. The tracker's `sources |=` merge already handles multi-source tracks. |
| **GPS/IMU + ego-motion compensation** | VesselPose model with NED + RTK fix quality and vessel_profile.yaml blocks for imu_mounting and rtk_mounting exist with no consumers yet. The noise filter docstring explicitly identifies the insertion point. |
| **Track confidence in safety layer** | WorldObject already exposes confidence, position_std_m, coasting, dynamic, sources, and safety_radius_m. |
| **L-shape vessel fitting** | Extractor docstring states it's structured so an L-shape stage can be inserted after the bounding-circle computation. |
| **Real-time pub/sub output** | json_publisher.py is plug-replaceable; the file calls out a future zmq_publisher.py for multi-consumer ZeroMQ delivery. |
| **Replay / offline analysis** | Every WorldModel carries scan_id; every Header carries monotonic timestamp_ns. JSON output is sufficient for replay. |

ROS2 migration is *not* explicitly scaffolded in current code, so it is not listed here.

---

## 10. Developer Quick Start

**Main entry point.** maritime_perception/main.py — run from the repo root: `python -m maritime_perception.main`. Expects:
- A built C++ publisher at tools/lidar_publisher/lidar_publisher (or via RPLIDAR_PUBLISHER env var).
- The two YAMLs at configs/default.yaml and configs/vessel_profile.yaml.

**Processing pipeline (one-liner).**
RawScanPoint -> CartesianPoint -> (noise-filtered) CartesianPoint -> Segment -> DetectionObservation -> ObjectTrack -> WorldObject -> JSON.

**Important modules to keep in your head.**
- Boundary contracts: sensors/base.py, models/observation.py, models/world_model.py.
- Orchestration: main.py, sensors/lidar/pipeline.py, fusion/tracker.py.
- Algorithms: sensors/lidar/noise_filter.py, sensors/lidar/segmentation.py, fusion/association.py, fusion/motion_model.py.

**Common modifications and where they go.**

| You want to ... | Touch |
|---|---|
| Tune detection sensitivity, sea-state response, latency budget | configs/default.yaml only — _validate will catch obvious mistakes. |
| Silence a new self-return (mast, antenna) | Add a sector to fov_mask in configs/vessel_profile.yaml. |
| Add a new sensor | New sensors/<name>/ dir + class implementing AbstractSensorPipeline (base.py); append a SensorSource member in common.py; build and start it in main.py; extend its observations into all_observations. Tracker code does not change. |
| Change track-confirmation thresholds, coast behaviour, score curve | tracker.py (HIT_REWARD, MISS_DECAY, lifecycle) and tracking.* in YAML. |
| Add fields to the published JSON | json_publisher.py _serialise_object — but back the field with something already on WorldObject. |
| Add a new fault condition | health/watchdog.py — extend check() and the WatchdogStatus payload. |
| Switch to ZeroMQ or shared memory output | New publisher class with publish(world: WorldModel); swap the construction in main.py. |
| Replace the SDK / change the wire protocol | Stay inside tools/lidar_publisher/main.cpp and sensors/lidar/driver.py; preserve the public surface (RPLidarDriver, LidarScan, RawScanPoint). |

**Recommended reading order (~25 min).**

1. models/common.py — Header, SensorSource, timestamp convention.
2. models/observation.py and models/world_model.py — the system's two architectural boundaries.
3. sensors/base.py — the sensor contract.
4. main.py — see how the loop wires everything.
5. sensors/lidar/pipeline.py and each of its four stages in order: preprocessor -> noise_filter -> segmentation -> extractor.
6. sensors/lidar/sensor_thread.py — threading model and the single-slot buffer.
7. fusion/motion_model.py -> fusion/association.py -> fusion/tracker.py -> fusion/builder.py.
8. health/monitor.py, health/watchdog.py, interfaces/json_publisher.py.
9. Re-read configs/default.yaml with the code fresh — every parameter will now have a home.

---

*Assumptions explicitly flagged.* No tests were exercised for this review; performance numbers (latency budget = 80 ms, expected n <= 20 tracks) come from comments in tracker.py and health/monitor.py, not measurement. The C++ publisher was inspected only via the Python-side protocol — its internals are summarised from driver.py comments and the docstring contract.
