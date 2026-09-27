"""Extended Kalman Filter for the fusion tracker (sim reference implementation).

State:  x = [px, py, vx, vy]   (world frame, metres / m·s^-1), constant-velocity.
Radar  measurement:  z = [px, py]        -> LINEAR update (H is constant).
Camera measurement:  z = atan2(py, px)   -> NONLINEAR update (this is what makes
                                            the filter an EKF).

This mirrors the C firmware to come; the sim is where we tune Q/R and prove
statistical consistency (NEES/NIS) before porting.
"""
import numpy as np


def wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class EKF:
    def __init__(self, dt, q, x0, P0, F=None, Q=None):
        """CV model by default; pass F/Q to use a different motion model (e.g. CT)."""
        self.dt = float(dt)
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        if F is None:
            F = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        self.F = np.array(F, dtype=float)
        if Q is None:
            # Continuous white-noise-acceleration process noise (spectral density q).
            dt3, dt2 = dt ** 3 / 3.0, dt ** 2 / 2.0
            Q = q * np.array([[dt3, 0, dt2, 0], [0, dt3, 0, dt2],
                              [dt2, 0, dt, 0], [0, dt2, 0, dt]], dtype=float)
        self.Q = np.array(Q, dtype=float)
        self._I = np.eye(4)

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_radar(self, z, R):
        """z = [px, py] Cartesian position. Returns (innovation, S)."""
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=float)
        y = np.asarray(z, float) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (self._I - K @ H) @ self.P
        return y, S

    def update_camera(self, z_bearing, R):
        """z_bearing = atan2(py, px) measurement (rad). Returns (innovation, S)."""
        px, py = self.x[0], self.x[1]
        r2 = px * px + py * py
        h = np.arctan2(py, px)
        H = np.array([[-py / r2, px / r2, 0, 0]], dtype=float)
        y = np.array([wrap(z_bearing - h)])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).ravel()
        self.P = (self._I - K @ H) @ self.P
        return y, S
