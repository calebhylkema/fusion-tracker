"""Interacting Multiple Model (IMM) estimator for maneuvering targets.

Runs a bank of EKFs with different motion models (e.g. constant-velocity +
coordinated-turn) and Markov-switches between them. During straight motion the
CV model dominates; during a turn the IMM shifts weight to a CT model and keeps
tracking where a single-model filter would lag/overshoot.

State ordering for all models: [px, py, vx, vy].
"""
import numpy as np


def F_cv(dt):
    return np.array([[1, 0, dt, 0], [0, 1, 0, dt],
                     [0, 0, 1, 0], [0, 0, 0, 1]], float)


def F_ct(omega, dt):
    """Coordinated-turn transition for turn rate omega (rad/s)."""
    if abs(omega) < 1e-6:
        return F_cv(dt)
    w, s, c = omega, np.sin(omega * dt), np.cos(omega * dt)
    return np.array([[1, 0, s / w, -(1 - c) / w],
                     [0, 1, (1 - c) / w, s / w],
                     [0, 0, c, -s],
                     [0, 0, s, c]], float)


def _gauss_likelihood(y, S):
    d = len(y)
    return float(np.exp(-0.5 * y @ np.linalg.inv(S) @ y)
                 / np.sqrt((2 * np.pi) ** d * np.linalg.det(S)))


class IMM:
    def __init__(self, models, P_trans, mu0):
        self.models = models                 # list of EKF instances
        self.M = len(models)
        self.P = np.array(P_trans, float)     # Markov transition matrix (MxM)
        self.mu = np.array(mu0, float)        # mode probabilities

    def step(self, zr, R_radar, zb=None, R_cam=None):
        # 1) interaction / mixing -------------------------------------------
        cbar = self.P.T @ self.mu                          # predicted mode prob
        mix = (self.P * self.mu[:, None]) / cbar[None, :]  # mix[i,j]
        xs = [m.x.copy() for m in self.models]
        Ps = [m.P.copy() for m in self.models]
        for j, m in enumerate(self.models):
            x0 = sum(mix[i, j] * xs[i] for i in range(self.M))
            P0 = np.zeros((4, 4))
            for i in range(self.M):
                dx = (xs[i] - x0).reshape(-1, 1)
                P0 += mix[i, j] * (Ps[i] + dx @ dx.T)
            m.x, m.P = x0, P0

        # 2) model-matched filtering + likelihood ---------------------------
        L = np.zeros(self.M)
        for j, m in enumerate(self.models):
            m.predict()
            y, S = m.update_radar(zr, R_radar)
            L[j] = _gauss_likelihood(y, S)
            if zb is not None:
                m.update_camera(zb, R_cam)

        # 3) mode-probability update ----------------------------------------
        self.mu = cbar * L
        s = self.mu.sum()
        self.mu = self.mu / s if s > 0 else np.ones(self.M) / self.M

        # 4) combine estimate -----------------------------------------------
        x = sum(self.mu[j] * self.models[j].x for j in range(self.M))
        P = np.zeros((4, 4))
        for j in range(self.M):
            dx = (self.models[j].x - x).reshape(-1, 1)
            P += self.mu[j] * (self.models[j].P + dx @ dx.T)
        return x, P
