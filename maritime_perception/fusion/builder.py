"""
fusion/builder.py

Builds a WorldModel from the current set of confirmed/coasting tracks.

This is the boundary between internal tracking state and the public
perception output. Nothing downstream ever sees ObjectTrack — only
WorldObject via WorldModel.
"""

from __future__ import annotations

import logging

from maritime_perception.models.common import Header
from maritime_perception.models.track import ObjectTrack, TrackState
from maritime_perception.models.world_model import WorldModel, WorldObject

log = logging.getLogger(__name__)


class WorldModelBuilder:

    def build(
        self,
        tracks   : dict[int, ObjectTrack],
        header   : Header,
        scan_id  : int   = 0,
        min_conf : float = 0.0,
        latency_ms: float = 0.0,
    ) -> WorldModel:
        """
        Convert confirmed + coasting tracks into a WorldModel snapshot.

        Parameters
        ----------
        tracks    : all active tracks from the tracker
        header    : cycle header (carries timestamp)
        scan_id   : monotonic cycle counter
        min_conf  : tracks below this confidence are not published
        latency_ms: pipeline processing time for this cycle

        Returns
        -------
        WorldModel — immutable snapshot for safety/navigation consumption.
        """
        objects = []

        for track in tracks.values():
            # only publish confirmed and coasting tracks
            if track.state not in (TrackState.CONFIRMED, TrackState.COASTING):
                continue

            # apply confidence threshold
            if track.confidence < min_conf:
                continue

            objects.append(WorldObject(
                id             = track.track_id,
                header         = header,
                position_x     = track.position_x,
                position_y     = track.position_y,
                velocity_x     = track.velocity_x,
                velocity_y     = track.velocity_y,
                heading_deg    = track.heading_deg,
                size_m         = track.size_m,
                confidence     = track.confidence,
                position_std_m = track.position_std_m,
                dynamic        = track.is_dynamic,
                coasting       = track.state == TrackState.COASTING,
                sources        = track.sources,
            ))

        log.debug(
            "builder: %d/%d tracks → world model (min_conf=%.2f)",
            len(objects), len(tracks), min_conf,
        )

        return WorldModel(
            header     = header,
            objects    = objects,
            scan_id    = scan_id,
            latency_ms = latency_ms,
        )