# Sensors Module

The `sensors/` package contains every sensor driver and perception pipeline. Each sensor lives in its own subdirectory, implements the shared `AbstractSensorPipeline` contract, and produces `DetectionObservation` objects — the universal currency of the fusion layer.

---

## Table of Contents

- [Design Contract](#design-contract)
- [AbstractSensorPipeline — base.py](#abstractsensorpipeline--basepy)
- [Available Sensors](#available-sensors)
- [Planned / Future Sensors](#planned--future-sensors)
- [How the Main Loop Uses Sensors](#how-the-main-loop-uses-sensors)
- [Adding a New Sensor](#adding-a-new-sensor)

---

## Design Contract

The entire sensor architecture is built around one idea: **the fusion layer must never know which physical sensor produced a detection**.

Every sensor pipeline:

1. Runs in its own background thread (never blocks the 10 Hz fusion loop).
2. Exposes a non-blocking `latest()` method that returns whatever observations are ready — or an empty list if none are.
3. Reports a health boolean so the watchdog can detect failures.
4. Produces `DetectionObservation` objects — never raw sensor data.

This strict interface means:

- A new sensor can be added without touching the tracker, world model, publisher, or safety layer.
- A sensor can fail without crashing the system — the watchdog downgrades system health and the tracker coasts existing tracks.
- Unit tests for the fusion layer work with synthetic `DetectionObservation` lists, no hardware required.

---

## AbstractSensorPipeline — base.py

```
maritime_perception/sensors/base.py
```

Defines the three-method contract that every sensor must satisfy.

### Interface

```python
class AbstractSensorPipeline(ABC):

    @abstractmethod
    def latest(self) -> list[DetectionObservation]:
        """
        Return the newest set of observations since the last call.
        Must be non-blocking — if no data is ready, return [].
        Called by the main fusion loop on every 10 Hz tick.
        """

    @abstractmethod
    def is_healthy(self) -> bool:
        """
        Return True if the sensor is operating normally.
        Return False to trigger a watchdog alert.
        Called by SensorWatchdog on every cycle.
        """

    @abstractmethod
    def start(self) -> None:
        """Start the sensor (open port, launch thread, etc.)."""

    @abstractmethod
    def stop(self) -> None:
        """Gracefully stop the sensor (join thread, close port, etc.)."""
```

### Why non-blocking `latest()`?

The fusion loop runs at exactly 10 Hz. If `latest()` blocked waiting for the next hardware scan, the loop timing would drift — or stall entirely if the sensor disconnected. By returning immediately with whatever is ready (or `[]`), the fusion loop stays on schedule and the tracker simply coasts any tracks that had no new observations this cycle.

---

## Available Sensors

### LiDAR — `sensors/lidar/`

The primary sensor. A full five-stage perception pipeline processes raw RPLidar S2 point clouds into `DetectionObservation` objects.

See [lidar/LIDAR.md](lidar/LIDAR.md) for full documentation.

**Pipeline stages:**

```
RPLidarDriver → Preprocessor → AdaptiveNoiseFilter → JumpDistanceSegmenter → ObstacleExtractor
```

**Key capabilities:**
- 360° coverage at up to ~10 Hz scan rate
- Adaptive sea-clutter rejection (raises threshold in rough water)
- Jump-distance segmentation with KD-tree merge for hull fragmentation
- Analytic confidence model based on point count and range
- Auto-reconnect on serial port loss (up to 10 retries)

---

## Planned / Future Sensors

Three sensor directories are scaffolded and ready for implementation:

| Directory | Sensor | Role |
|---|---|---|
| `sensors/AIS/` | Automatic Identification System | Long-range vessel tracking via maritime transponders |
| `sensors/FLS/` | Forward-Looking Sonar | Close-range underwater obstacle detection |
| `sensors/MBES/` | Multibeam Echo Sounder | Bathymetric mapping + submerged obstacle detection |

Each of these will be implemented as a class extending `AbstractSensorPipeline`, producing the same `DetectionObservation` interface.

**AIS** is a particularly valuable fusion candidate because it provides long-range tracks with precise velocity from the other vessel's own GPS — complementing LiDAR's high-resolution short-range detections.

---

## How the Main Loop Uses Sensors

In `main.py`, all active sensor threads are held in a list:

```python
sensor_threads: list[AbstractSensorPipeline] = [
    lidar_sensor_thread,
    # ais_sensor_thread,   # when AIS is implemented
    # fls_sensor_thread,   # when FLS is implemented
]
```

Each 10 Hz tick collects observations from all sensors:

```python
all_observations: list[DetectionObservation] = []
for sensor in sensor_threads:
    all_observations.extend(sensor.latest())
```

Then the full batch is handed to the tracker at once. The tracker does not know or care how many sensors contributed.

The watchdog checks health independently:

```python
for sensor in sensor_threads:
    if not sensor.is_healthy():
        watchdog.report_fault(sensor)
```

---

## Adding a New Sensor

1. Create the directory: `maritime_perception/sensors/<name>/`
2. Add `__init__.py`
3. Create `pipeline.py` implementing `AbstractSensorPipeline`:

```python
from maritime_perception.sensors.base import AbstractSensorPipeline
from maritime_perception.models.observation import DetectionObservation

class MyNewSensorPipeline(AbstractSensorPipeline):

    def start(self) -> None:
        # Open hardware connection, start background thread
        ...

    def stop(self) -> None:
        # Signal thread to stop, join, close connection
        ...

    def latest(self) -> list[DetectionObservation]:
        # Drain your internal buffer — non-blocking
        # Return [] if nothing is ready
        ...

    def is_healthy(self) -> bool:
        # Check your thread is alive and data is arriving on time
        ...
```

4. Instantiate your pipeline in `main.py` and append it to `sensor_threads`.

No other changes are needed anywhere in the codebase.
