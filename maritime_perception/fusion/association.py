"""
fusion/association.py

Track-to-observation association using the Hungarian algorithm
with Mahalanobis distance gating.

Why Hungarian instead of nearest-neighbour
------------------------------------------
Nearest-neighbour greedily assigns each observation to the closest
track. This breaks when two tracks cross — IDs swap, velocity spikes,
safety layer sees false alarms.

The Hungarian algorithm (linear sum assignment) finds the globally
optimal assignment — the one that minimises total Mahalanobis distance
across ALL track-observation pairs simultaneously. No swaps.

scipy.optimize.linear_sum_assignment implements the Jonker-Volgenant
algorithm — O(n³) but n is track count (rarely > 20 in maritime).
Negligible cost in practice.

Mahalanobis gating
------------------
Before building the cost matrix, each (track, observation) pair is
tested against a Mahalanobis distance gate. Pairs outside the gate
are assigned cost=INF and will never be matched.

The gate threshold is chi-squared distributed:
  2-DOF (x, y position), 99.5% confidence → threshold = 9.21

This means: if an observation is more than ~3σ from a track's
predicted position (accounting for that track's uncertainty), they
cannot be matched. This is principled, not arbitrary.

Output
------
matched     : list of (obs_idx, track_id) pairs
unmatched_obs : observation indices with no track match (→ new track)
unmatched_trk : track IDs with no observation (→ increment miss count)
"""

from __future__ import annotations

import logging

import numpy as np
from scipy.optimize import linear_sum_assignment

from maritime_perception.models.observation import DetectionObservation
from maritime_perception.models.track import ObjectTrack

log = logging.getLogger(__name__)

INF = 1e9


def associate(
    observations      : list[DetectionObservation],
    tracks            : dict[int, ObjectTrack],
    gate_mahalanobis  : float = 9.21,
) -> tuple[list[tuple[int, int]], list[int], list[int]]:
    """
    Associate observations to tracks using Hungarian + Mahalanobis gate.

    Parameters
    ----------
    observations     : current scan's DetectionObservations
    tracks           : active tracks {track_id: ObjectTrack}
    gate_mahalanobis : squared Mahalanobis distance threshold

    Returns
    -------
    matched       : list of (obs_idx, track_id)
    unmatched_obs : obs indices not assigned to any track
    unmatched_trk : track IDs not assigned to any observation
    """
    if not observations or not tracks:
        return [], list(range(len(observations))), list(tracks.keys())

    track_ids  = list(tracks.keys())
    n_obs      = len(observations)
    n_trk      = len(track_ids)

    # build cost matrix [n_obs × n_trk]
    cost = np.full((n_obs, n_trk), INF, dtype=np.float64)

    for i, obs in enumerate(observations):
        z = np.array([obs.position_x, obs.position_y], dtype=np.float64)
        for j, tid in enumerate(track_ids):
            track = tracks[tid]
            d_sq  = track.kalman.mahalanobis(z)
            if d_sq <= gate_mahalanobis:
                cost[i, j] = d_sq

    # Hungarian assignment
    row_ind, col_ind = linear_sum_assignment(cost)

    matched       : list[tuple[int, int]] = []
    unmatched_obs : list[int]             = list(range(n_obs))
    unmatched_trk : list[int]             = list(track_ids)

    for r, c in zip(row_ind, col_ind):
        if cost[r, c] < INF:
            tid = track_ids[c]
            matched.append((r, tid))
            if r in unmatched_obs:
                unmatched_obs.remove(r)
            if tid in unmatched_trk:
                unmatched_trk.remove(tid)

    log.debug(
        "association: %d obs, %d tracks → %d matched, "
        "%d unmatched_obs, %d unmatched_trk",
        n_obs, n_trk, len(matched),
        len(unmatched_obs), len(unmatched_trk),
    )
    return matched, unmatched_obs, unmatched_trk