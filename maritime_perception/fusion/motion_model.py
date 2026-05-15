"""
fusion/motion_model.py

Kalman filter — constant velocity motion model for 2D obstacle tracking.

State vector: [x, y, vx, vy]
    x, y   : position in vessel frame (metres)
    vx, vy : velocity in vessel frame (m/s)

This is the industry standard for maritime radar and vessel tracking.
It has been used in production maritime systems for 40+ years.

Equations
---------
Prediction (time update):
    x_pred = F × x
    P_pred = F × P × Fᵀ + Q

Update (measurement update):
    y    = z - H × x_pred          (innovation)
    S    = H × P_pred × Hᵀ + R    (innovation covariance)
    K    = P_pred × Hᵀ × S⁻¹      (Kalman gain)
    x    = x_pred + K × y
    P    = (I - K × H) × P_pred

Matrices
--------
F : state transition (constant velocity, dt=1/scan_rate_hz)
H : observation model (we observe position only, not velocity)
Q : process noise (how much we trust the constant velocity assumption)
R : measurement noise (how accurately LiDAR measures position)
P : state covariance (uncertainty of our current estimate)

All implemented with numpy — fast, no external dependencies beyond numpy.
"""

from __future__ import annotations

import numpy as np


def make_F(dt: float) -> np.ndarray:
    """
    State transition matrix for constant velocity model.
    dt = time between scans in seconds.
    """
    return np.array([
        [1, 0, dt, 0 ],
        [0, 1, 0,  dt],
        [0, 0, 1,  0 ],
        [0, 0, 0,  1 ],
    ], dtype=np.float64)


def make_H() -> np.ndarray:
    """
    Observation matrix — we observe (x, y) position only.
    Velocity is not directly observed.
    """
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
    ], dtype=np.float64)


def make_Q(sigma_pos: float, sigma_vel: float) -> np.ndarray:
    """
    Process noise covariance.
    sigma_pos : position uncertainty per scan (metres)
    sigma_vel : velocity uncertainty per scan (m/s)
    Higher values = trust observations more, model less.
    """
    return np.diag([
        sigma_pos**2,
        sigma_pos**2,
        sigma_vel**2,
        sigma_vel**2,
    ]).astype(np.float64)


def make_R(sigma_meas: float) -> np.ndarray:
    """
    Measurement noise covariance.
    sigma_meas : LiDAR position measurement noise (metres).
    Reflects centroid accuracy of the cluster — typically 0.2–0.5m.
    """
    return np.diag([sigma_meas**2, sigma_meas**2]).astype(np.float64)


def make_P_init(sigma_pos: float = 2.0, sigma_vel: float = 2.0) -> np.ndarray:
    """
    Initial state covariance for a new track.
    High uncertainty — we don't know much about a new target yet.
    """
    return np.diag([
        sigma_pos**2,
        sigma_pos**2,
        sigma_vel**2,
        sigma_vel**2,
    ]).astype(np.float64)


class KalmanFilter:
    """
    2D constant velocity Kalman filter for one track.
    One instance per ObjectTrack.
    """

    I4 = np.eye(4, dtype=np.float64)   # class-level constant

    def __init__(
        self,
        initial_x  : np.ndarray,   # shape (4,) — [x, y, vx, vy]
        initial_P  : np.ndarray,   # shape (4, 4)
        F          : np.ndarray,   # shape (4, 4) — shared, precomputed
        H          : np.ndarray,   # shape (2, 4) — shared, precomputed
        Q          : np.ndarray,   # shape (4, 4) — shared, precomputed
        R          : np.ndarray,   # shape (2, 2) — shared, precomputed
    ) -> None:
        self.x = initial_x.copy()
        self.P = initial_P.copy()
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R

    def predict(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Time update — advance state by one scan period.
        Returns (predicted_x, predicted_P).
        Updates internal state.
        """
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x, self.P

    def update(self, z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Measurement update — incorporate a new observation.
        z : observation vector shape (2,) — [obs_x, obs_y]
        Returns (updated_x, updated_P).
        Updates internal state.
        """
        # innovation
        y = z - self.H @ self.x

        # innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # state update
        self.x = self.x + K @ y

        # covariance update (Joseph form — numerically stable)
        I_KH   = self.I4 - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        return self.x, self.P

    def mahalanobis(self, z: np.ndarray) -> float:
        """
        Mahalanobis distance between observation z and predicted position.
        Used by the association gate to decide if an observation belongs
        to this track. Accounts for track uncertainty.

        Returns squared Mahalanobis distance.
        Compare to chi-squared threshold (e.g. 9.21 for 99.5% at 2-DOF).
        """
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        return float(y.T @ np.linalg.inv(S) @ y)