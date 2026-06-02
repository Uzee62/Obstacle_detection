"""
interfaces/json_publisher.py

Publishes the WorldModel to a JSON file.

This is the current output interface — simple, debuggable, works with
any consumer that can read a file (dashboard, safety layer, logger).

Next upgrade: zmq_publisher.py — ZeroMQ pub/sub for multi-consumer
real-time distribution without file IO overhead.

The file is written atomically (write to temp, rename) so consumers
never read a partially written file.
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
from pathlib import Path
from typing import Any

from maritime_perception.models.common import SensorSource
from maritime_perception.models.world_model import WorldModel, WorldObject

log = logging.getLogger(__name__)


class JsonPublisher:

    def __init__(self, output_path: str) -> None:
        self._path = Path(output_path).expanduser()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        log.info("JsonPublisher: output → %s", self._path)

    def publish(self, world: WorldModel) -> None:
        """
        Serialise WorldModel to JSON and write atomically.
        Never raises — logs error and returns on failure.
        """
        try:
            payload = self._serialise(world)
            self._atomic_write(json.dumps(payload, indent=2))
        except Exception as exc:
            log.error("JsonPublisher: failed to write: %s", exc)

    # ------------------------------------------------------------------

    def _serialise(self, world: WorldModel) -> dict[str, Any]:
        return {
            "header": {
                "timestamp_ns": world.header.timestamp_ns,
                "sensor_id"   : world.header.sensor_id,
                "frame_id"    : world.header.frame_id,
            },
            "scan_id"    : world.scan_id,
            "latency_ms" : round(world.latency_ms, 2),
            "object_count": len(world),
            "objects"    : [self._serialise_object(o) for o in world],
        }

    @staticmethod
    def _serialise_object(obj: WorldObject) -> dict[str, Any]:
        return {
            "id"            : obj.id,
            "position"      : {"x": round(obj.position_x, 3),
                               "y": round(obj.position_y, 3)},
            "velocity"      : {"x": round(obj.velocity_x, 3),
                               "y": round(obj.velocity_y, 3)},
            "heading_deg"   : round(obj.heading_deg, 1),
            "range_m"       : round(obj.range_m, 2),
            "bearing_deg"   : round(obj.bearing_deg, 1),
            "speed_ms"      : round(obj.speed_ms, 3),
            "size_m"        : round(obj.size_m, 2),
            "safety_radius_m": round(obj.safety_radius_m, 2),
            "confidence"    : round(obj.confidence, 3),
            "position_std_m": round(obj.position_std_m, 3),
            "dynamic"       : obj.dynamic,
            "coasting"      : obj.coasting,
            "sources"       : [s.name for s in SensorSource
                               if s in obj.sources],
        }

    def _atomic_write(self, content: str) -> None:
        """Write to temp file then rename — atomic on POSIX systems."""
        dir_  = self._path.parent
        fd, tmp = tempfile.mkstemp(dir=dir_, suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(content)
            os.replace(tmp, self._path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise