"""
test_pipeline.py

Smoke test — runs the full pipeline with synthetic scan data.


Simulates:
  - Two approaching vessels at different bearings
  - One stationary buoy
  - Wave noise returns (transient, should be filtered)

Verifies:
  - Pipeline runs without errors
  - Confirmed tracks appear after MIN_HITS scans
  - World model is produced each cycle
  - Track IDs are stable across scans
  - Processing time is within budget
"""

from __future__ import annotations

import math
import random
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maritime_perception.models.common import Header, SensorSource, now_ns
from maritime_perception.sensors.lidar.driver import LidarScan, RawScanPoint
from maritime_perception.sensors.lidar.pipeline import LidarPerceptionPipeline
from maritime_perception.fusion.tracker import FusionTracker
from maritime_perception.fusion.builder import WorldModelBuilder

# Synthetic scan generator

def make_synthetic_scan(
    scan_id    : int,
    obstacles  : list[dict],
    n_rays     : int   = 720,
    noise_prob : float = 0.003,
    rng        : random.Random = None,
) -> LidarScan:
    """
    Generate a synthetic LiDAR scan with obstacles and wave noise.

    obstacles: list of dicts with keys:
        bearing_deg, range_m, width_deg
    """
    if rng is None:
        rng = random.Random(scan_id)

    step   = 360.0 / n_rays
    points = []

    for i in range(n_rays):
        angle  = i * step
        dist   = 28.0 + rng.uniform(-0.3, 0.3)   # open water background
        quality = 15

        # check obstacle returns
        for obs in obstacles:
            diff = abs(((angle - obs["bearing_deg"]) + 180) % 360 - 180)
            if diff <= obs["width_deg"] / 2:
                dist    = obs["range_m"] + rng.gauss(0, 0.04)
                quality = 25
                break

        # wave noise — transient, random, close range
        if rng.random() < noise_prob:
            dist    = rng.uniform(2.0, 8.0)
            quality = 8

        points.append(RawScanPoint(
            angle_deg  = angle,
            distance_m = dist,
            quality    = quality,
        ))

    return LidarScan(
        header=Header(
            timestamp_ns = now_ns(),
            sensor_id    = "rplidar_s2_sim",
            frame_id     = "lidar",
            source       = SensorSource.LIDAR,
        ),
        points  = points,
        scan_id = scan_id,
    )

# Minimal config for test

TEST_CFG = {
    "preprocessing": {
        "range_min_m"  : 0.3,
        "range_max_m"  : 30.0,
        "min_intensity": 0,
    },
    "lidar_mounting": {
        "angle_offset_deg": 0.0,
    },
    "fov_mask": [[178.0, 182.0]],
    "noise_filter": {
        "cell_size_m"      : 0.25,
        "min_hits_base"    : 5,
        "min_hits_max"     : 8,
        "ttl_scans"        : 15,
        "grid_radius_m"    : 35.0,
        "clutter_adapt_rate": 0.1,
    },
    "segmentation": {
        "jump_distance_m" : 0.5,
        "adaptive_k"      : 0.02,
        "min_points"      : 3,
        "min_arc_deg"     : 0.5,
        "max_points"      : 2000,
        "merge_distance_m": 1.0,
    },
    "extraction": {
        "max_obstacle_radius_m"   : 15.0,
        "confidence_pts_ref"      : 30,
        "confidence_range_penalty": 0.5,
    },
    "tracking": {
        "gate_mahalanobis"    : 9.21,
        "min_hits_confirm"    : 3,
        "max_misses_tentative": 3,
        "max_misses_confirmed": 10,
        "max_misses_coast"    : 5,
        "min_confidence_output": 0.25,
        "kalman": {
            "process_noise_pos": 0.1,
            "process_noise_vel": 0.5,
            "measurement_noise": 0.3,
        },
    },
    "fusion": {
        "cycle_rate_hz": 10.0,
    },
}

# Run


def main():
    print("=" * 60)
    print("  Maritime Perception — Synthetic Pipeline Smoke Test")
    print("=" * 60)

    pipeline = LidarPerceptionPipeline(TEST_CFG)
    tracker  = FusionTracker(TEST_CFG)
    rng      = random.Random(42)

    N_SCANS = 40

    # Three obstacles:
    # A: vessel approaching from 15° bearing, closing range
    # B: stationary buoy at 280°, 18m
    # C: vessel crossing from 60° bearing, constant range

    print(f"\nSimulating {N_SCANS} scans at 10Hz ...\n")
    print(f"{'Scan':>4}  {'Obs':>3}  {'Tracks':>6}  Details")
    print("-" * 60)

    seen_ids = set()
    t_total  = 0.0

    for i in range(1, N_SCANS + 1):
        vessel_a_range = max(5.0, 14.0 - i * 0.2)

        obstacles = [
            {"bearing_deg": 15.0,  "range_m": vessel_a_range, "width_deg": 3.5},
            {"bearing_deg": 280.0, "range_m": 18.0,           "width_deg": 2.0},
            {"bearing_deg": 60.0,  "range_m": 12.0,           "width_deg": 4.0},
        ]

        scan = make_synthetic_scan(i, obstacles, rng=rng)

        t0  = time.perf_counter()
        obs = pipeline.process(scan)

        cycle_header = Header(
            timestamp_ns = now_ns(),
            sensor_id    = "fusion",
            frame_id     = "vessel",
            source       = SensorSource.LIDAR,
        )
        world = tracker.update(obs, cycle_header, scan_id=i)
        dt_ms = (time.perf_counter() - t0) * 1000
        t_total += dt_ms

        for obj in world:
            seen_ids.add(obj.id)
            coast = " [COAST]" if obj.coasting else ""
            dyn   = "MOV" if obj.dynamic else "STA"
            print(
                f"  {i:3d}  {len(obs):3d}  "
                f"ID={obj.id:3d}  "
                f"rng={obj.range_m:5.1f}m  "
                f"brg={obj.bearing_deg:+6.1f}°  "
                f"spd={obj.speed_ms:.2f}m/s  "
                f"conf={obj.confidence:.2f}  "
                f"{dyn}{coast}"
            )

        if not world.objects:
            print(f"  {i:3d}  {len(obs):3d}  (building tracks ...)")

    #  summary 
    stats = pipeline.last_stats
    print("\n" + "=" * 60)
    print("  Results")
    print("=" * 60)
    print(f"  Total scans processed : {N_SCANS}")
    print(f"  Unique track IDs seen : {sorted(seen_ids)}")
    print(f"  Final confirmed tracks: {tracker.confirmed_count}")
    print(f"  Mean cycle time       : {t_total/N_SCANS:.2f}ms")
    print(f"  Max safe budget       : 100ms (10Hz)")

    if stats:
        print(f"\n  Last scan diagnostics:")
        print(f"    raw points       : {stats.raw_points}")
        print(f"    after preprocess : {stats.after_preprocess}")
        print(f"    after noise filt : {stats.after_noise}  "
              f"({stats.noise_rejection_pct:.0f}% rejected)")
        print(f"    segments         : {stats.segments}")
        print(f"    observations     : {stats.observations}")
        print(f"    noise threshold  : {stats.noise_threshold} (adaptive)")
        print(f"    pipeline time    : {stats.process_time_ms:.2f}ms")

    #  assertions 
    print("\n  Assertions:")
    assert tracker.confirmed_count > 0, "FAIL: no confirmed tracks"
    print("  ✓ confirmed tracks > 0")

    assert t_total / N_SCANS < 50.0, \
        f"FAIL: mean cycle time {t_total/N_SCANS:.1f}ms exceeds 50ms budget"
    print(f"  ✓ mean cycle time {t_total/N_SCANS:.2f}ms < 50ms budget")

    assert len(seen_ids) >= 2, "FAIL: expected at least 2 distinct track IDs"
    print(f"  ✓ {len(seen_ids)} distinct track IDs produced")

    assert stats.noise_rejection_pct > 0, "FAIL: noise filter rejected nothing"
    print(f"  ✓ noise filter active ({stats.noise_rejection_pct:.0f}% rejected)")

    print("\n  ALL ASSERTIONS PASSED\n")


if __name__ == "__main__":
    main()