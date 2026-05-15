# Interfaces Module

The `interfaces/` package handles all outbound communication from the perception system. It consumes the `WorldModel` produced by the fusion layer and makes it available to external consumers — safety systems, navigation algorithms, logging tools, or monitoring dashboards.

---

## Table of Contents

- [Overview](#overview)
- [json_publisher.py — JsonPublisher](#json_publisherpy--jsonpublisher)
- [Output Format](#output-format)
- [Atomic Write Guarantee](#atomic-write-guarantee)
- [Error Handling](#error-handling)
- [Extending the Interface Layer](#extending-the-interface-layer)

---

## Overview

The interfaces layer sits at the boundary between the perception system and everything that consumes it. It follows two strict rules:

1. **Never raise.** A publishing failure must never crash or stall the perception pipeline. If the disk is full, the path is wrong, or serialisation fails, log the error and continue. Obstacle detection keeps running.
2. **Never block.** Writing to disk is slow compared to a 10 Hz cycle. The publisher uses an atomic rename pattern (write-to-temp then rename) which is the fastest safe write pattern available on POSIX systems.

---

## json_publisher.py — JsonPublisher

**Class:** `JsonPublisher`

Serialises a `WorldModel` to a JSON file on every fusion cycle. External processes read this file to get the current obstacle picture.

**Output path:** `~/Obstacle_Detection/world_model.json` (configurable via `configs/default.yaml → interfaces.output_path`)

### Usage

```python
publisher = JsonPublisher(output_path="~/Obstacle_Detection/world_model.json")

# Called once per 10 Hz cycle, inside main.py
publisher.publish(world_model)
```

### What it does each cycle

1. Serialises the `WorldModel` to a Python `dict`.
2. Serialises the dict to a JSON string (`json.dumps`).
3. Writes the JSON string to a **temporary file** in the same directory.
4. **Atomically renames** the temp file to the final output path.
5. Returns. If any step fails, logs the exception and returns without crashing.

---

## Output Format

The published JSON file has the following structure:

```json
{
  "header": {
    "timestamp_ns": 1715000000000000000,
    "sensor_id": "lidar_0",
    "frame_id": "vessel",
    "source": 1
  },
  "scan_id": 412,
  "latency_ms": 23.4,
  "object_count": 3,
  "objects": [
    {
      "id": 7,
      "position": {
        "x": 12.34,
        "y": -4.10
      },
      "velocity": {
        "vx": 0.42,
        "vy": -0.11
      },
      "heading_deg": 284.6,
      "range_m": 12.97,
      "bearing_deg": -18.5,
      "speed_ms": 0.43,
      "size_m": 1.80,
      "safety_radius_m": 2.45,
      "confidence": 0.91,
      "position_std_m": 0.65,
      "dynamic": true,
      "coasting": false,
      "sources": 1
    }
  ]
}
```

### Field Reference

**Top-level:**

| Field | Type | Description |
|---|---|---|
| `header.timestamp_ns` | integer | Monotonic nanosecond timestamp of this cycle |
| `header.sensor_id` | string | Primary sensor identifier |
| `header.frame_id` | string | Coordinate frame (always `"vessel"`) |
| `header.source` | integer | Bitmask of contributing `SensorSource` values |
| `scan_id` | integer | Monotonically increasing cycle counter |
| `latency_ms` | float | Time from LiDAR scan acquisition to world model creation |
| `object_count` | integer | Number of objects in this cycle's output |
| `objects` | array | Per-obstacle data (see below) |

**Per-object:**

| Field | Type | Unit | Description |
|---|---|---|---|
| `id` | integer | — | Stable track ID; same ID across consecutive scans for the same obstacle |
| `position.x` | float | metres | Forward offset from vessel CoR (bow = positive) |
| `position.y` | float | metres | Lateral offset (port = positive) |
| `velocity.vx` | float | m/s | Estimated forward velocity |
| `velocity.vy` | float | m/s | Estimated lateral velocity |
| `heading_deg` | float | degrees | Estimated direction of movement |
| `range_m` | float | metres | Distance from vessel origin |
| `bearing_deg` | float | degrees | Bearing from bow (clockwise positive) |
| `speed_ms` | float | m/s | Estimated speed magnitude |
| `size_m` | float | metres | Estimated physical radius of the obstacle |
| `safety_radius_m` | float | metres | `size_m + position_std_m` — use this for collision avoidance |
| `confidence` | float | 0–1 | Track quality score |
| `position_std_m` | float | metres | Position uncertainty (from Kalman covariance) |
| `dynamic` | bool | — | `true` if speed > 0.15 m/s |
| `coasting` | bool | — | `true` if track has no recent observation (predicted forward) |
| `sources` | integer | — | Bitmask of `SensorSource` values |

**`safety_radius_m` is the most important field for collision avoidance.** It inflates the physical size by the current position uncertainty — when the tracker is less certain about where an obstacle is, the exclusion zone grows to compensate.

### Coordinate Convention

The vessel coordinate frame:
- `+x` → bow (forward)
- `+y` → port (left)
- Bearings are measured clockwise from the bow

---

## Atomic Write Guarantee

The publisher never writes directly to the final output path. Instead:

```
1. Write JSON → world_model.json.tmp
2. os.rename("world_model.json.tmp", "world_model.json")
```

`os.rename()` is an atomic operation on POSIX filesystems. A reader of `world_model.json` will always see either the complete old file or the complete new file — never a partially-written file. This is critical for consumers that poll the file on every tick.

On Windows (non-POSIX), `os.replace()` is used instead, which provides the same atomicity guarantee.

---

## Error Handling

The publisher wraps the entire publish cycle in a `try/except`. If anything fails:

- The exception is logged at `ERROR` level with full traceback.
- The method returns normally.
- The fusion loop continues.
- The output file retains its last successful state.

This means a consumer reading the file will see a slightly stale world model until the next successful publish — acceptable behaviour compared to crashing the perception pipeline.

---

## Extending the Interface Layer

To add a new output interface (e.g., a UDP broadcast, a ROS2 topic publisher, a database writer):

1. Create a new file in `maritime_perception/interfaces/` (e.g., `udp_publisher.py`).
2. Define a class with a `publish(world_model: WorldModel) -> None` method.
3. Follow the same two rules: **never raise, never block**.
4. Instantiate it in `main.py` and call it in the fusion loop alongside the JSON publisher.

No changes to the tracker, models, or sensor layer are needed.
