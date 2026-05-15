"""
main.py

Entry point for the maritime perception system.

Responsibilities

1. Load config
2. Set up logging
3. Build all components
4. Start sensor threads
5. Run the fusion loop at configured rate
6. Handle SIGINT/SIGTERM for graceful shutdown
7. Publish world model each cycle
8. Run health monitoring

Threading model

- LiDAR sensor thread : background, reads hardware, runs LiDAR pipeline
- Main thread         : fusion loop at 10Hz, reads latest from sensor threads,
                        runs tracker, publishes world model

When adding a new sensor (AIS, FLS):
  1. Build its sensor thread
  2. Add to 'sensor_threads' list
  3. Feed its 'latest()' into 'all_observations' in the fusion loop
  No other changes needed.
"""

from __future__ import annotations

import logging
import signal
import sys
import time
from pathlib import Path

from maritime_perception.config import load_config
from maritime_perception.logging_config import setup_logging
from maritime_perception.models.common import Header, SensorSource, now_ns, elapsed_ms
from maritime_perception.sensors.lidar.driver import RPLidarDriver
from maritime_perception.sensors.lidar.pipeline import LidarPerceptionPipeline
from maritime_perception.sensors.lidar.sensor_thread import LidarSensorThread
from maritime_perception.fusion.tracker import FusionTracker
from maritime_perception.fusion.builder import WorldModelBuilder
from maritime_perception.interfaces.json_publisher import JsonPublisher
from maritime_perception.health.monitor import HealthMonitor, CycleStats
from maritime_perception.health.watchdog import SensorWatchdog, SystemHealth

log = logging.getLogger("main")


# Graceful shutdown flag


_running = True

def _handle_signal(signum, frame) -> None:
    global _running
    log.info("Signal %d received — shutting down ...", signum)
    _running = False


# Main Entry Point

# Initializes the system, starts sensor threads, and runs the fusion loop.


def main() -> None:
    global _running

    # --- logging ---
    setup_logging(log_file="/tmp/maritime_perception.log")
    log.info("=" * 60)
    log.info("  Maritime Perception System starting")
    log.info("=" * 60)

    # --- signals ---
    signal.signal(signal.SIGINT,  _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    # --- config ---
    cfg_path     = Path("configs/default.yaml")
    profile_path = Path("configs/vessel_profile.yaml")

    try:
        cfg = load_config(cfg_path, profile_path)
    except (FileNotFoundError, ValueError) as exc:
        log.critical("Config error: %s", exc)
        sys.exit(1)

    
    
    # build components 


    lidar_cfg = cfg.get("lidar", {})
    driver = RPLidarDriver(
        port                   = lidar_cfg.get("port", "/dev/ttyUSB0"),
        timeout_s              = float(lidar_cfg.get("timeout_s", 5.0)),
        reconnect_delay_s      = float(lidar_cfg.get("reconnect_delay_s", 2.0)),
        max_reconnect_attempts = int(lidar_cfg.get("max_reconnect_attempts", 10)),
    )

    lidar_pipeline = LidarPerceptionPipeline(cfg)
    lidar_thread   = LidarSensorThread(
        driver         = driver,
        pipeline       = lidar_pipeline,
        max_scan_gap_s = float(cfg.get("health", {}).get("max_scan_gap_s", 1.0)),
    )

    tracker   = FusionTracker(cfg)
    publisher = JsonPublisher(
        cfg.get("interfaces", {}).get("json_output_path", "/tmp/world_model.json")
    )
    monitor  = HealthMonitor(
        window_scans=int(cfg.get("health", {}).get("stats_window_scans", 100))
    )
    watchdog = SensorWatchdog(
        max_scan_gap_s=float(cfg.get("health", {}).get("max_scan_gap_s", 1.0))
    )

    loop_rate_hz  = float(cfg.get("runtime", {}).get("loop_rate_hz", 10.0))
    loop_period_s = 1.0 / loop_rate_hz
    scan_id       = 0

    
    # start background sensor threads
    # Each sensor thread continuously reads from its hardware source, 
    # processes data through its pipeline, and stores the latest observations. 
    # The main fusion loop will read these observations at each cycle. 

    try:
        lidar_thread.start()
    except RuntimeError as exc:
        log.critical("Failed to start LiDAR: %s", exc)
        sys.exit(1)

    log.info("Fusion loop starting at %.1f Hz", loop_rate_hz)

   
   
    # Main Fusion loop
    # This loop runs at a fixed rate (e.g., 10 Hz). Each cycle it:
        # 1. Collects latest observations from all sensor threads   
        # 2. Builds a cycle header with timestamp and metadata
        # 3. Runs the tracker update step to produce a new world model
        # 4. Publishes the world model to JSON
        # 5. Records health stats and checks watchdogs

    
    try:
        while _running:
            t0 = now_ns()
            scan_id += 1

            # --- collect observations from all sensor threads ---
            all_observations = []

            lidar_obs = lidar_thread.latest()
            all_observations.extend(lidar_obs)

            # when AIS/FLS are added:
            # ais_obs = ais_thread.latest()
            # all_observations.extend(ais_obs)

            # --- build cycle header ---
            cycle_header = Header(
                timestamp_ns = now_ns(),
                sensor_id    = "fusion",
                frame_id     = "vessel",
                source       = SensorSource.LIDAR,
            )

            # run tracker 
            world = tracker.update(
                observations = all_observations,
                header       = cycle_header,
                scan_id      = scan_id,
            )

            latency = elapsed_ms(t0)
            world.latency_ms = latency

            # publish 
            publisher.publish(world)

            #  health monitoring
            watchdog_status = watchdog.check(lidar_thread)
            if watchdog_status.system_health == SystemHealth.CRITICAL:
                log.error(
                    "CRITICAL: primary sensor offline — "
                    "world model may be stale"
                )

            lidar_stats = lidar_pipeline.last_stats
            if lidar_stats:
                monitor.record(CycleStats(
                    timestamp_s      = time.monotonic(),
                    scan_id          = scan_id,
                    raw_points       = lidar_stats.raw_points,
                    after_preprocess = lidar_stats.after_preprocess,
                    after_noise      = lidar_stats.after_noise,
                    observations     = lidar_stats.observations,
                    confirmed_tracks = tracker.confirmed_count,
                    world_objects    = len(world),
                    latency_ms       = latency,
                    noise_threshold  = lidar_stats.noise_threshold,
                ))

            # log summary every 100 cycles
            if scan_id % 100 == 0:
                monitor.log_summary()

            #  pace the loop 
            elapsed_s = elapsed_ms(t0) / 1000.0
            sleep_s   = loop_period_s - elapsed_s
            if sleep_s > 0:
                time.sleep(sleep_s)
            elif sleep_s < -0.01:
                log.warning(
                    "fusion loop overran by %.1fms on cycle %d",
                    -sleep_s * 1000, scan_id,
                )

    except Exception as exc:
        log.exception("Unhandled exception in fusion loop: %s", exc)

    finally:
        # graceful shutdown 
        log.info("Stopping sensor threads ...")
        lidar_thread.stop()
        monitor.log_summary()
        log.info("Maritime Perception System stopped. Total cycles: %d", scan_id)


if __name__ == "__main__":
    main()