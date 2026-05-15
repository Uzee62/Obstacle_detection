# Health Module

The `health/` package provides runtime observability and fault detection for the perception system. It tracks performance metrics over rolling windows and monitors sensor health, allowing the system to degrade gracefully instead of failing silently.

---

## Table of Contents

- [Overview](#overview)
- [monitor.py — HealthMonitor](#monitorpy--healthmonitor)
- [watchdog.py — SensorWatchdog](#watchdogpy--sensorwatchdog)
- [System Health States](#system-health-states)
- [How the Main Loop Uses Health](#how-the-main-loop-uses-health)
- [Configuration Reference](#configuration-reference)

---

## Overview

The health layer answers two questions each cycle:

1. **How is the system performing?** (HealthMonitor)
   — latency, scan rate, noise rejection percentage, obstacle count — all averaged over a rolling window.

2. **Is the system still trustworthy?** (SensorWatchdog)
   — are sensors alive? are scans arriving on time? has the error rate spiked?

Separating these concerns keeps each class focused. The monitor is purely observational — it records and summarises. The watchdog makes a binary judgement: NOMINAL, DEGRADED, or CRITICAL.

---

## monitor.py — HealthMonitor

**Class:** `HealthMonitor`

Maintains a rolling window of per-cycle statistics and exposes summary metrics.

### `CycleStats` (per-cycle record)

Each fusion cycle, the main loop records one `CycleStats`:

| Field | Type | Description |
|---|---|---|
| `timestamp_ns` | `int` | Cycle start timestamp |
| `raw_points` | `int` | LiDAR returns before preprocessing |
| `observations` | `int` | `DetectionObservation` objects produced this cycle |
| `world_objects` | `int` | Obstacles in the published `WorldModel` |
| `latency_ms` | `float` | End-to-end time from scan acquisition to world model |
| `noise_rejection_pct` | `float` | Fraction of points discarded by the noise filter |

### Rolling Window

The monitor stores the last `stats_window_size` cycles (default 100) in a `collections.deque`. Older entries are automatically evicted. All summary properties compute over this window.

### Summary Properties

| Property | Description |
|---|---|
| `scan_rate_hz` | Estimated scan rate from timestamps of the last N cycles |
| `mean_latency_ms` | Average end-to-end latency across the window |
| `max_latency_ms` | Peak latency observed in the window |
| `mean_world_objects` | Average number of confirmed obstacles per cycle |
| `mean_noise_rejection_pct` | Average clutter rejection rate — useful for sea-state awareness |
| `uptime_s` | Time since first recorded cycle |
| `total_scans` | Cumulative cycle count since startup |

### Latency Warning

If `mean_latency_ms > latency_warn_ms` (default 80 ms), the monitor logs a warning. At 10 Hz, each cycle has 100 ms; sustained latency above 80 ms means the system is close to falling behind real-time.

---

## watchdog.py — SensorWatchdog

**Class:** `SensorWatchdog`

Makes the binary health judgement used by the safety layer to decide whether autonomous operation is safe.

### `SystemHealth` Enum

```
NOMINAL   — all sensors operating within normal parameters
DEGRADED  — one or more fault conditions detected; proceed with caution
CRITICAL  — primary sensor has failed; autonomous operations must pause
```

### Fault Conditions

The watchdog checks the following conditions each cycle (called by `main.py`):

**Condition 1 — Scan gap (timeout):**

```
if time_since_last_scan > max_scan_gap_s (default 1.0 s):
    fault = True
```

If no scan has arrived for more than 1 second, the sensor is assumed offline. At 10 Hz the normal inter-scan interval is 0.1 s — a 1.0 s gap means 10 consecutive misses.

**Condition 2 — Thread not alive:**

```
if not sensor_thread.is_alive():
    fault = True
```

The sensor background thread should never die during normal operation. If it does, all further `latest()` calls will return `[]` indefinitely.

**Condition 3 — Error rate spike:**

```
if recent_error_count > error_spike_threshold (default 3):
    fault = True
```

The sensor thread tracks exceptions. A sudden cluster of errors (e.g., 3 in the last 10 scans) indicates an unstable connection before a complete disconnect occurs — early warning before health degrades fully.

### Health State Transitions

```
All conditions OK
    → NOMINAL

Any fault condition active:
    → DEGRADED (primary fault but thread alive)
    → CRITICAL (thread dead or extended gap)
```

The watchdog records `degraded_since_s` — the timestamp when the system first entered a degraded state. This allows downstream systems to apply a grace period before escalating to a full stop.

### Recovery

When all fault conditions clear, the watchdog transitions back to NOMINAL. The `degraded_since_s` field is reset to `None`.

---

## System Health States

| State | Meaning | Recommended action |
|---|---|---|
| `NOMINAL` | All sensors operating normally | Autonomous operation permitted |
| `DEGRADED` | Fault detected; tracks coasting | Reduce speed, increase caution, alert operator |
| `CRITICAL` | Primary sensor offline | Suspend autonomous operations; require human intervention |

The safety layer is responsible for acting on these states. The perception system reports the state but does not enforce behaviour.

---

## How the Main Loop Uses Health

In `main.py`, each 10 Hz cycle:

```python
# 1. After pipeline runs — record stats
stats = CycleStats(
    timestamp_ns=now_ns(),
    raw_points=pipeline_stats.raw_points,
    observations=len(observations),
    world_objects=len(world_model.objects),
    latency_ms=pipeline_stats.process_time_ms,
    noise_rejection_pct=pipeline_stats.noise_rejection_pct,
)
health_monitor.record(stats)

# 2. Check watchdog
system_health = watchdog.check(lidar_sensor_thread)

if system_health == SystemHealth.CRITICAL:
    log.error("Primary sensor offline — safety layer must stop autonomous operations")
elif system_health == SystemHealth.DEGRADED:
    log.warning("Sensor degraded — coasting tracks; proceed with caution")
```

The separation means the monitor accumulates stats regardless of watchdog state, and the watchdog operates on the sensor thread directly — they do not depend on each other.

---

## Configuration Reference

```yaml
health:
  max_scan_gap_s: 1.0          # seconds without a scan before fault is declared
  latency_warn_ms: 80.0        # mean latency threshold for warning log
  stats_window_size: 100       # rolling window depth (number of cycles)
  error_spike_threshold: 3     # errors in recent window before declaring fault
```

**Tuning guidance:**

- **Too many false DEGRADED alerts:** Increase `max_scan_gap_s` or `error_spike_threshold`.
- **Missing real sensor outages:** Decrease `max_scan_gap_s` for faster detection.
- **Latency warnings too frequent:** Increase `latency_warn_ms`, or investigate pipeline performance.
- **Smoother metrics:** Increase `stats_window_size` (trades responsiveness for stability).
