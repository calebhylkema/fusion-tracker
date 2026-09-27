"""Live animation of the EKF fusion tracker.

Run:  ..\\.venv\\Scripts\\python.exe live_fusion.py     (from the sim/ folder)

A window opens and plays in real time: the target (black) walks across the
field of view, noisy radar measurements (grey x) come in, and you watch the
RADAR-ONLY estimate (blue) vs the RADAR+CAMERA fused estimate (red) track it.
The red ellipse is the filter's 2-sigma position uncertainty (watch it shrink
as measurements come in). Live error readout top-left. Close the window to quit.
"""
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.animation import FuncAnimation

from ekf import EKF

DT = 0.1
N = 200
SIG_RX, SIG_RY = 0.40, 0.15
SIG_BEARING = np.deg2rad(1.0)
R_RADAR = np.diag([SIG_RX ** 2, SIG_RY ** 2])
R_CAM = np.array([[SIG_BEARING ** 2]])
Q = 0.2

# --- ground truth walk + measurements (fixed seed for a repeatable demo) ---
rng = np.random.default_rng(1)
F = np.array([[1, 0, DT, 0], [0, 1, 0, DT], [0, 0, 1, 0], [0, 0, 0, 1]], float)
a, b = DT ** 3 / 3, DT ** 2 / 2
Qmat = Q * np.array([[a, 0, b, 0], [0, a, 0, b], [b, 0, DT, 0], [0, b, 0, DT]])
truth = np.zeros((N, 4)); zr = np.zeros((N, 2)); zb = np.zeros(N)
x = np.array([-4.0, 6.0, 1.0, 0.0])
for k in range(N):
    truth[k] = x
    zr[k] = x[:2] + rng.normal(0, [SIG_RX, SIG_RY])
    zb[k] = np.arctan2(x[1], x[0]) + rng.normal(0, SIG_BEARING)
    x = F @ x + rng.multivariate_normal(np.zeros(4), Qmat)

ekf_r = EKF(DT, Q, [truth[0, 0], truth[0, 1], 0, 0], np.diag([1, 1, 4, 4]))
ekf_f = EKF(DT, Q, [truth[0, 0], truth[0, 1], 0, 0], np.diag([1, 1, 4, 4]))

fig, ax = plt.subplots(figsize=(8, 7))
ax.set_xlim(-5, 5); ax.set_ylim(-0.5, 9); ax.set_aspect("equal"); ax.grid(alpha=0.3)
ax.set_xlabel("x  cross-range (m)"); ax.set_ylabel("y  range (m)")
ax.plot(0, 0, "g^", ms=14, label="sensor")
truth_line, = ax.plot([], [], "k-", lw=2, label="ground truth")
truth_dot, = ax.plot([], [], "ko", ms=8)
meas_dot, = ax.plot([], [], "x", color="0.6", label="radar measurement")
r_line, = ax.plot([], [], "b--", lw=1.3, label="radar-only EKF")
f_line, = ax.plot([], [], "r-", lw=1.6, label="radar+camera EKF")
ell = Ellipse((0, 0), 0, 0, fc="r", alpha=0.15); ax.add_patch(ell)
txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", fontfamily="monospace")
ax.legend(loc="lower left")
ax.set_title("EKF fusion — live (radar-only vs radar+camera)")

r_hist, f_hist = [], []


def set_ellipse(mean, P2, n=2.0):
    vals, vecs = np.linalg.eigh(P2)        # eigenvalues ascending; [:,1] = major axis
    ell.set_center((mean[0], mean[1]))
    ell.width = 2 * n * np.sqrt(max(vals[1], 1e-9))   # major
    ell.height = 2 * n * np.sqrt(max(vals[0], 1e-9))  # minor
    ell.angle = np.degrees(np.arctan2(vecs[1, 1], vecs[0, 1]))


def step(k):
    ekf_r.predict(); ekf_r.update_radar(zr[k], R_RADAR)
    ekf_f.predict(); ekf_f.update_radar(zr[k], R_RADAR); ekf_f.update_camera(zb[k], R_CAM)
    r_hist.append(ekf_r.x[:2].copy()); f_hist.append(ekf_f.x[:2].copy())
    truth_line.set_data(truth[:k + 1, 0], truth[:k + 1, 1])
    truth_dot.set_data([truth[k, 0]], [truth[k, 1]])
    meas_dot.set_data([zr[k, 0]], [zr[k, 1]])
    r = np.array(r_hist); f = np.array(f_hist)
    r_line.set_data(r[:, 0], r[:, 1]); f_line.set_data(f[:, 0], f[:, 1])
    set_ellipse(ekf_f.x[:2], ekf_f.P[:2, :2])
    er = np.linalg.norm(ekf_r.x[:2] - truth[k, :2])
    ef = np.linalg.norm(ekf_f.x[:2] - truth[k, :2])
    txt.set_text(f"t = {k*DT:4.1f} s\nradar-only err = {er:4.2f} m\nfused      err = {ef:4.2f} m")
    return truth_line, truth_dot, meas_dot, r_line, f_line, ell, txt


ani = FuncAnimation(fig, step, frames=N, interval=80, blit=False, repeat=False)
plt.show()
