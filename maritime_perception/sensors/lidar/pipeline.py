"""
sensors/lidar/pipeline.py

LiDAR perception pipeline orchestrator.

pipeline.py is the orchestration layer that runs one LiDAR scan through preprocessing, 
noise filtering, segmentation, and extraction, then returns standardized 
 DetectionObservation objects along with detailed per-scan diagnostics.

Wires all LiDAR-specific stages in order:
    LidarScan
      → LidarPreprocessor     (range gate, FOV mask, polar→cart)
      → AdaptiveNoiseFilter   (temporal persistence, sea-state adaptive)
      → JumpDistanceSegmenter (split + KD-tree merge)
      → ObstacleExtractor     (centroid, radius, confidence)
      → list[DetectionObservation]

This class contains NO perception logic.
Its only job is to call each stage in order and pass the output along.
All logic lives in the stage modules.


Thread safety

This pipeline is stateful (noise filter holds a grid).
It must be called from a single thread.
The sensor_thread.py wrapper ensures this.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

from maritime_perception.models.common import elapsed_ms, now_ns
from maritime_perception.models.observation import DetectionObservation

from .driver import LidarScan
from .preprocessor import LidarPreprocessor, PreprocessorConfig
from .noise_filter import AdaptiveNoiseFilter, NoiseFilterConfig
from .segmentation import JumpDistanceSegmenter, SegmentationConfig
from .extractor import ObstacleExtractor, ExtractorConfig

log = logging.getLogger(__name__)


# Per-scan diagnostics (This dataclass stores diagnostics for one processed scan)


@dataclass(slots=True)
class LidarPipelineStats:
    scan_id            : int   = 0
    raw_points         : int   = 0
    after_preprocess   : int   = 0
    after_noise        : int   = 0
    segments           : int   = 0
    observations       : int   = 0
    process_time_ms    : float = 0.0
    noise_threshold    : int   = 0


# Computes how many preprocessed points were removed by the noise filter.
    @property
    def noise_rejection_pct(self) -> float:
        if self.after_preprocess == 0:
            return 0.0
        return 100.0 * (1.0 - self.after_noise / self.after_preprocess)


# Lidar perception pipeline (main class that wires all stages together)

class LidarPerceptionPipeline:
    """
    Full LiDAR obstacle detection pipeline.
    Input : LidarScan (from driver)
    Output: list[DetectionObservation] (to fusion engine)
    """

    def __init__(self, cfg: dict[str, Any]) -> None:
        self._preprocessor = LidarPreprocessor(
            PreprocessorConfig.from_config(cfg)
        )
        self._noise_filter = AdaptiveNoiseFilter(
            NoiseFilterConfig.from_config(cfg)
        )
        self._segmenter = JumpDistanceSegmenter(
            SegmentationConfig.from_config(cfg)
        )
        self._extractor = ObstacleExtractor(
            ExtractorConfig.from_config(cfg)
        )
        self._last_stats: LidarPipelineStats | None = None
        log.info("LidarPerceptionPipeline initialised")

    def process(self, scan: LidarScan) -> list[DetectionObservation]:
        """
        Run one scan through the full pipeline.
        Returns DetectionObservations ready for the fusion engine.
        """
        
        t0    = now_ns()              # start timer for diagnostics
        stats = LidarPipelineStats(
            scan_id   = scan.scan_id,
            raw_points= len(scan),
        )

        # Stage 1: preprocess
        cart_points = self._preprocessor.process(scan)
        stats.after_preprocess = len(cart_points)

        if not cart_points:
            self._last_stats = stats
            return []

        # Stage 2: noise filter
        filtered = self._noise_filter.update(cart_points, scan.scan_id)
        stats.after_noise      = len(filtered)
        stats.noise_threshold  = self._noise_filter.current_threshold

        # Stage 3: segmentation
        segments = self._segmenter.segment(filtered)
        stats.segments = len(segments)

        # Stage 4: extraction
        observations = self._extractor.extract(segments, scan.header)
        stats.observations    = len(observations)
        stats.process_time_ms = elapsed_ms(t0)

        self._last_stats = stats

        log.info(
            "lidar | scan=%04d raw=%4d pre=%4d filt=%4d "
            "seg=%2d obs=%2d thr=%d | %.1fms | noise=%.0f%%",
            stats.scan_id,
            stats.raw_points,
            stats.after_preprocess,
            stats.after_noise,
            stats.segments,
            stats.observations,
            stats.noise_threshold,
            stats.process_time_ms,
            stats.noise_rejection_pct,
        )
        return observations

    @property
    def last_stats(self) -> LidarPipelineStats | None:
        return self._last_stats

    def reset(self) -> None:
        """Reset stateful components. Call on session restart."""
        self._noise_filter.reset()
        log.info("LidarPerceptionPipeline reset")