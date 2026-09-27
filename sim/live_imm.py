"""Live animation of the IMM (multi-model) tracker on a maneuvering target.

Run:  ..\\.venv\\Scripts\\python.exe live_imm.py       (from the sim/ folder)

Top panel: the target (black) goes straight -> turns -> straight, with the IMM
estimate (red) tracking it. Bottom panel: the live mode probabilities. Watch the
bars swing from "CV" to a "CT" (turn) model exactly when the target starts
turning, then back. Close the window to quit.
"""
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from ekf import EKF
from imm import IMM, F_cv, F_ct

DT = 0.1
N = 260
SIG_RX, SIG_RY = 0.40, 0.15
SIG_BEARING = np.deg2rad(1.0)
R_RADAR = np.diag([SIG_RX ** 2, SIG_RY ** 2])
R_CAM = np.array([[SIG_BEARING ** 2]])
OMEGA = 0.6
TURN = (int(6.0 / DT), int(16.0 / DT))

rng = np.random.default_rng(2)
a, b = DT ** 3 / 3, DT ** 2 / 2
Qt = 0.05 * np.array([[a, 0, b, 0], [0, a, 0, b], [b, 0, DT, 0], [0, b, 0, DT]])
truth = np.zeros((N, 4)); zr = np.zeros((N, 2)); zb = np.zeros(N)
x = np.array([-7.0, 5.0, 1.4, 0.0])
for k in range(N):
    truth[k] = x
    zr[k] = x[:2] + rng.normal(0, [SIG_RX, SIG_RY])
    zb[k] = np.arctan2(x[1], x[0]) + rng.normal(0, SIG_BEARING)
    Fk = F_ct(OMEGA, DT) if TURN[0] <= k < TURN[1] else F_cv(DT)
    x = Fk @ x + rng.multivariate_normal(np.zeros(4), Qt)

P0 = np.diag([1, 1, 4, 4]); x0 = [truth[0, 0], truth[0, 1], 0, 0]
models = [EKF(DT, 0.05, x0, P0.copy(), F=F_cv(DT)),
          EKF(DT, 0.10, x0, P0.copy(), F=F_ct(+OMEGA, DT)),
          EKF(DT, 0.10, x0, P0.copy(), F=F_ct(-OMEGA, DT))]
Pt = np.array([[0.95, 0.025, 0.025], [0.05, 0.90, 0.05], [0.05, 0.05, 0.90]])
imm = IMM(models, Pt, [0.8, 0.1, 0.1])

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 9),
                               gridspec_kw={"height_ratios": [3, 1]})
ax1.set_xlim(-8, 5); ax1.set_ylim(3, 10); ax1.set_aspect("equal"); ax1.grid(alpha=0.3)
ax1.plot(0, 0, "g^", ms=12, label="sensor")
tl, = ax1.plot([], [], "k-", lw=2, label="ground truth")
td, = ax1.plot([], [], "ko", ms=8)
il, = ax1.plot([], [], "r-", lw=1.6, label="IMM estimate")
ax1.legend(loc="upper right")
ax1.set_title("IMM live — trajectory")
bars = ax2.bar(["CV", "CT left", "CT right"], [0, 0, 0], color=["C0", "C1", "C2"])
ax2.set_ylim(0, 1); ax2.set_ylabel("mode probability")

est_hist = []


def step(k):
    xe, _ = imm.step(zr[k], R_RADAR, zb[k], R_CAM)
    est_hist.append(xe[:2].copy())
    tl.set_data(truth[:k + 1, 0], truth[:k + 1, 1])
    td.set_data([truth[k, 0]], [truth[k, 1]])
    e = np.array(est_hist); il.set_data(e[:, 0], e[:, 1])
    for bar, h in zip(bars, imm.mu):
        bar.set_height(h)
    turning = TURN[0] <= k < TURN[1]
    ax2.set_title(f"mode probabilities   t={k*DT:4.1f}s"
                  f"{'   --- TARGET TURNING ---' if turning else ''}")
    return (tl, td, il, *bars)


ani = FuncAnimation(fig, step, frames=N, interval=60, blit=False, repeat=False)
plt.tight_layout()
plt.show()
