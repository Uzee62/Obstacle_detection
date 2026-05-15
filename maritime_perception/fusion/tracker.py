"""
fusion/tracker.py

Multi-object tracker — the fusion layer's core.

Lifecycle

TENTATIVE  → needs min_hits_confirm consecutive hits → CONFIRMED
TENTATIVE  → miss_count > max_misses_tentative       → DEAD (pruned quietly)
CONFIRMED  → miss_count > max_misses_confirmed       → COASTING
COASTING   → observation received                    → CONFIRMED (recovered)
COASTING   → miss_count > max_misses_coast           → DEAD

Coast mode

A confirmed track that temporarily loses observations (vessel went
behind headland, brief sensor occlusion) enters COASTING state.
It continues to be predicted by the Kalman filter and published to
the world model, but with decaying confidence.
This prevents real vessels from vanishing from the world model during
brief occlusions — a critical safety property.

Track score
-----------
A continuous quality metric in [0, 1] that evolves on each scan:
  hit  → score = score + hit_reward  × (1 - score)   (asymptotes to 1)
  miss → score = score × miss_decay                   (exponential decay)

This is more informative than raw hit/miss counters. A track seen
50/52 scans has a score near 0.95. A track seen 3/3 scans has a
score near 0.5. Both are "confirmed" but the safety layer can
distinguish them via the confidence field in WorldObject.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

from maritime_perception.models.common import Header, SensorSource, now_ns
from maritime_perception.models.observation import DetectionObservation
from maritime_perception.models.track import ObjectTrack, TrackState
from maritime_perception.models.world_model import WorldModel, WorldObject

from .association import associate
from .motion_model import (
    KalmanFilter,
    make_F, make_H, make_Q, make_R, make_P_init,
)
from .builder import WorldModelBuilder

log = logging.getLogger(__name__)


class FusionTracker:
    """
    Multi-object tracker with Kalman filter and full lifecycle management.
    Accepts DetectionObservations from any number of sensors.
    Produces a WorldModel each cycle.
    """

    HIT_REWARD  = 0.15   # score increase per hit
    MISS_DECAY  = 0.80   # score multiplier per miss

    def __init__(self, cfg: dict[str, Any]) -> None:
        """
        Initialize the tracker from configuration.

        Configuration includes:
        - Association gate threshold
        - Track confirmation thresholds
        - Maximum allowed misses
        - Kalman filter noise parameters
        - Fusion loop rate

        One shared set of Kalman matrices (F, H, Q, R) is created and reused
        by all tracks for efficiency.
        """


        tr = cfg.get("tracking", {})
        kf = tr.get("kalman", {})

        #Association and lifecycle parameters


        # Maximum squared Mahalanobis distance allowed for a valid match.
        # 9.21 corresponds to ~99% confidence for a 2D chi-square distribution.

        self._gate            = float(tr.get("gate_mahalanobis", 9.21))

        # Number of hits required to promote TENTATIVE -> CONFIRMED.
        self._min_hits        = int(tr.get("min_hits_confirm", 3))

        # Number of misses allowed before deleting a tentative track.
        self._max_miss_tent   = int(tr.get("max_misses_tentative", 3))
        # Number of misses allowed before CONFIRMED -> COASTING.
        self._max_miss_conf   = int(tr.get("max_misses_confirmed", 10))

        #Additional misses allowed in COASTING before deletion.
        self._max_miss_coast  = int(tr.get("max_misses_coast", 5))

        # Minimum confidence required for a track to appear in WorldModel.
        self._min_conf_out    = float(tr.get("min_confidence_output", 0.25))

        # Fusion update frequency (e.g. 10 Hz => dt = 0.1 s).
        self._scan_rate_hz    = float(cfg.get("fusion", {}).get("cycle_rate_hz", 10.0))

        dt = 1.0 / self._scan_rate_hz  #time between 2 consecutive fusion cycles



        # Shared Kalman matrices
        # These matrices are identical for every track and therefore computed
        # once during initialization rather than on every update.

        # pre-compute shared Kalman matrices (same for all tracks)
        self._F = make_F(dt)
        self._H = make_H()
        self._Q = make_Q(
            float(kf.get("process_noise_pos", 0.1)),
            float(kf.get("process_noise_vel", 0.5)),
        )
        self._R = make_R(float(kf.get("measurement_noise", 0.3)))

        self._tracks   : dict[int, ObjectTrack] = {}
        self._next_id  : int = 1
        self._scan_id  : int = 0
        self._builder  = WorldModelBuilder()

        log.info(
            "FusionTracker initialised: gate=%.2f min_hits=%d "
            "max_miss_conf=%d coast=%d",
            self._gate, self._min_hits,
            self._max_miss_conf, self._max_miss_coast,
        )

   #Main interface 

    # These methods and properties are intended to be used by the rest of
    # the application (e.g., main.py, tests, monitoring tools).
    #
    # update()          -> Runs one complete fusion cycle and returns a WorldModel
    # reset()           -> Clears all active tracks and resets internal counters
    # track_count       -> Total number of active tracks
    # confirmed_count   -> Number of publishable tracks

    def update(
        self,
        observations : list[DetectionObservation],
        header       : Header,
        scan_id      : int = 0,
    ) -> WorldModel:
        """
        Run one full tracking cycle.

        Parameters
        ----------
        observations : all DetectionObservations from all sensors this cycle
        header       : cycle header (timestamp, frame_id)
        scan_id      : monotonic cycle counter

        Returns
        -------
        WorldModel — confirmed + coasting tracks above confidence threshold.
        """
        self._scan_id = scan_id

        # 1. Predict all tracks forward
        self._predict_all()

        # 2. Associate observations to tracks
        matched, unmatched_obs, unmatched_trk = associate(
            observations, self._tracks, self._gate
        )

        # 3. Update matched tracks
        for obs_idx, trk_id in matched:
            self._update_track(self._tracks[trk_id], observations[obs_idx])

        # 4. Increment miss count on unmatched tracks
        for trk_id in unmatched_trk:
            self._tracks[trk_id].miss_count += 1

        # 5. Spawn tentative tracks for unmatched observations
        for obs_idx in unmatched_obs:
            self._spawn(observations[obs_idx])

        # 6. Lifecycle transitions and pruning
        self._lifecycle()

        # 7. Build world model
        world = self._builder.build(
            tracks     = self._tracks,
            header     = header,
            scan_id    = scan_id,
            min_conf   = self._min_conf_out,
        )

        log.debug(
            "tracker cycle=%04d: total=%d confirmed=%d coasting=%d "
            "obs_in=%d matched=%d world_out=%d",
            scan_id,
            len(self._tracks),
            sum(1 for t in self._tracks.values() if t.state == TrackState.CONFIRMED),
            sum(1 for t in self._tracks.values() if t.state == TrackState.COASTING),
            len(observations),
            len(matched),
            len(world),
        )
        return world

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1
        self._scan_id = 0
        log.info("FusionTracker: reset")

    @property
    def track_count(self) -> int:
        return len(self._tracks)

    @property
    def confirmed_count(self) -> int:
        return sum(
            1 for t in self._tracks.values()
            if t.state in (TrackState.CONFIRMED, TrackState.COASTING)
        )

    # Internal processing stages


    # Helper methods used only inside FusionTracker to implement the
    # tracking algorithm. These are not called directly by external code.
    #
    # _predict_all()    -> Propagate all tracks forward using Kalman predict
    # _update_track()   -> Apply a matched observation to an existing track
    # _spawn()          -> Create a new tentative track from an observation
    # _lifecycle()      -> Handle state transitions and delete stale tracks

    def _predict_all(self) -> None:
        for track in self._tracks.values():
            track.kalman.predict()
            track.age_scans += 1

    
    # update matched track
    # This is called for every matched track-observation pair. It applies the
    # Kalman update step to the track's state vector and covariance, and updates
    

    def _update_track(
        self,
        track : ObjectTrack,
        obs   : DetectionObservation,
    ) -> None:
        z = np.array([obs.position_x, obs.position_y], dtype=np.float64)
        track.kalman.update(z)

        # sync model fields from Kalman state
        track.state_vec  = track.kalman.x
        track.covariance = track.kalman.P

        # metadata
        track.hit_count    += 1
        track.miss_count    = 0
        track.last_update_ns= now_ns()
        track.sources      |= obs.header.source
        track.size_m        = (
            0.7 * obs.size_m + 0.3 * track.size_m   # EMA on size
        )

        # score update
        track.score = track.score + self.HIT_REWARD * (1.0 - track.score)
        track.confidence = track.score

        # recover from coast
        if track.state == TrackState.COASTING:
            track.state = TrackState.CONFIRMED
            log.debug("Track #%d recovered from COASTING", track.track_id)

    # spawn new tentative track
    # This is called for every unmatched observation. It creates a new track
    # in TENTATIVE state with an initial Kalman filter based on the observation.
    

    def _spawn(self, obs: DetectionObservation) -> None:
        trk_id = self._next_id
        self._next_id += 1

        x0 = np.array(
            [obs.position_x, obs.position_y, 0.0, 0.0],
            dtype=np.float64,
        )
        P0 = make_P_init()
        kf = KalmanFilter(x0, P0, self._F, self._H, self._Q, self._R)

        track = ObjectTrack(
            track_id       = trk_id,
            state          = TrackState.TENTATIVE,
            header         = obs.header,
            state_vec      = x0,
            covariance     = P0,
            size_m         = obs.size_m,
            confidence     = obs.confidence,
            score          = 0.3,
            hit_count      = 1,
            sources        = obs.header.source,
            last_update_ns = now_ns(),
        )
        # attach kalman filter to track
        track.kalman = kf

        self._tracks[trk_id] = track
        log.debug(
            "Track #%d TENTATIVE spawned at (%.1f, %.1f)",
            trk_id, obs.position_x, obs.position_y,
        )

    # internal lifecycle management

    # Handles track state transitions and removal of stale tracks.

    def _lifecycle(self) -> None:
        """
         Responsibilities:
            Promote TENTATIVE tracks to CONFIRMED after enough hits
            Move CONFIRMED tracks to COASTING after repeated misses
            Recover COASTING tracks back to CONFIRMED when observed again
            Delete tracks that have been missing for too long
            Decay confidence scores when observations are missed
        """


        to_delete = []

        for trk_id, track in self._tracks.items():
            state = track.state

            if state == TrackState.TENTATIVE:
                if track.hit_count >= self._min_hits:
                    track.state = TrackState.CONFIRMED
                    log.info(
                        "Track #%d CONFIRMED — range=%.1fm brg=%.1f°",
                        trk_id, track.range_m, track.bearing_deg,
                    )
                elif track.miss_count >= self._max_miss_tent:
                    to_delete.append(trk_id)

            elif state == TrackState.CONFIRMED:
                if track.miss_count >= self._max_miss_conf:
                    track.state = TrackState.COASTING
                    log.info("Track #%d → COASTING", trk_id)
                else:
                    # apply miss score decay
                    if track.miss_count > 0:
                        track.score      *= self.MISS_DECAY
                        track.confidence  = track.score

            elif state == TrackState.COASTING:
                track.score      *= self.MISS_DECAY
                track.confidence  = track.score
                if track.miss_count >= self._max_miss_conf + self._max_miss_coast:
                    to_delete.append(trk_id)
                    log.info(
                        "Track #%d DEAD after %d total misses",
                        trk_id, track.miss_count,
                    )

        for trk_id in to_delete:
            del self._tracks[trk_id]