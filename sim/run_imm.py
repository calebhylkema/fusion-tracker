"""IMM vs single-model EKF on a maneuvering target.

Truth: straight -> coordinated turn -> straight. A single CV-EKF lags/overshoots
in the turn; the IMM (CV + CT-left + CT-right) shifts weight to a turn model and
keeps tracking. Uses the same radar+camera fusion as run_fusion.py.

Outputs: printed RMSE (overall + during the turn) + plots (trajectory, mode probs).
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from ekf import EKF
from imm import IMM, F_cv, F_ct

DT = 0.1
N = 220
SIG_RX, SIG_RY = 0.40, 0.15
SIG_BEARING = np.deg2rad(1.0)
R_RADAR = np.diag([SIG_RX ** 2, SIG_RY ** 2])
R_CAM = np.array([[SIG_BEARING ** 2]])

OMEGA_TRUE = 0.6                     # turn rate during the maneuver (rad/s)
TURN = (int(6.0 / DT), int(16.0 / DT))   # turn happens between these steps


def make_Q(q, dt):
    dt3, dt2 = dt ** 3 / 3.0, dt ** 2 / 2.0
    return q * np.array([[dt3, 0, dt2, 0], [0, dt3, 0, dt2],
                         [dt2, 0, dt, 0], [0, dt2, 0, dt]], float)


def simulate(seed):
    """straight -> turn -> straight, with light process noise + measurements."""
    rng = np.random.default_rng(seed)
    Qt = make_Q(0.05, DT)
    x = np.array([-7.0, 5.0, 1.4, 0.0])
    truth = np.zeros((N, 4))
    zr = np.zeros((N, 2))
    zb = np.zeros(N)
    for k in range(N):
        truth[k] = x
        zr[k] = x[:2] + rng.normal(0, [SIG_RX, SIG_RY])
        zb[k] = np.arctan2(x[1], x[0]) + rng.normal(0, SIG_BEARING)
        F = F_ct(OMEGA_TRUE, DT) if TURN[0] <= k < TURN[1] else F_cv(DT)
        x = F @ x + rng.multivariate_normal(np.zeros(4), Qt)
    return truth, zr, zb


def run_cv(truth, zr, zb, q):
    ekf = EKF(DT, q=q, x0=[truth[0, 0], truth[0, 1], 0, 0], P0=np.diag([1, 1, 4, 4]))
    est = np.zeros((N, 4))
    for k in range(N):
        ekf.predict()
        ekf.update_radar(zr[k], R_RADAR)
        ekf.update_camera(zb[k], R_CAM)
        est[k] = ekf.x
    return est


def run_imm(truth, zr, zb):
    P0 = np.diag([1, 1, 4, 4])
    x0 = [truth[0, 0], truth[0, 1], 0, 0]
    models = [
        EKF(DT, q=0.05, x0=x0, P0=P0.copy(), F=F_cv(DT)),          # constant velocity
        EKF(DT, q=0.10, x0=x0, P0=P0.copy(), F=F_ct(+OMEGA_TRUE, DT)),  # turn left
        EKF(DT, q=0.10, x0=x0, P0=P0.copy(), F=F_ct(-OMEGA_TRUE, DT)),  # turn right
    ]
    P_trans = np.array([[0.95, 0.025, 0.025],
                        [0.05, 0.90, 0.05],
                        [0.05, 0.05, 0.90]])
    imm = IMM(models, P_trans, mu0=[0.8, 0.1, 0.1])
    est = np.zeros((N, 4))
    mu = np.zeros((N, 3))
    for k in range(N):
        x, _ = imm.step(zr[k], R_RADAR, zb[k], R_CAM)
        est[k] = x
        mu[k] = imm.mu
    return est, mu


def rmse(est, truth, sl=slice(None)):
    d = est[sl, :2] - truth[sl, :2]
    return np.sqrt(np.mean(np.sum(d ** 2, axis=1)))


# ------------------------------------------------------- Monte Carlo ---------
M = 200
cv_all, cv_turn, imm_all, imm_turn = [], [], [], []
turn_sl = slice(*TURN)
for m in range(M):
    truth, zr, zb = simulate(m)
    e_cv = run_cv(truth, zr, zb, q=0.2)
    e_imm, _ = run_imm(truth, zr, zb)
    cv_all.append(rmse(e_cv, truth)); cv_turn.append(rmse(e_cv, truth, turn_sl))
    imm_all.append(rmse(e_imm, truth)); imm_turn.append(rmse(e_imm, truth, turn_sl))

print("=" * 60)
print(f"Maneuvering target: {M} Monte Carlo runs")
print("-" * 60)
print(f"Position RMSE overall   single-CV: {np.mean(cv_all):.3f} m   "
      f"IMM: {np.mean(imm_all):.3f} m")
print(f"Position RMSE in turn   single-CV: {np.mean(cv_turn):.3f} m   "
      f"IMM: {np.mean(imm_turn):.3f} m  "
      f"({100*(1-np.mean(imm_turn)/np.mean(cv_turn)):.0f}% better)")
print("=" * 60)

# ------------------------------------------------------- plots ---------------
truth, zr, zb = simulate(0)
e_cv = run_cv(truth, zr, zb, 0.2)
e_imm, mu = run_imm(truth, zr, zb)

plt.figure(figsize=(7, 6))
plt.plot(truth[:, 0], truth[:, 1], "k-", lw=2, label="ground truth")
plt.plot(e_cv[:, 0], e_cv[:, 1], "b--", lw=1.5, label="single-CV EKF")
plt.plot(e_imm[:, 0], e_imm[:, 1], "r-", lw=1.5, label="IMM (CV+CT)")
plt.scatter([0], [0], marker="^", c="g", s=120, label="sensor")
plt.xlabel("x - cross-range (m)"); plt.ylabel("y - range (m)")
plt.title("IMM vs single-CV EKF on a maneuvering target")
plt.legend(); plt.axis("equal"); plt.grid(True, alpha=0.3); plt.tight_layout()
plt.savefig("imm_trajectory.png", dpi=120)

t = np.arange(N) * DT
plt.figure(figsize=(8, 4))
plt.plot(t, mu[:, 0], label="CV")
plt.plot(t, mu[:, 1], label="CT left")
plt.plot(t, mu[:, 2], label="CT right")
plt.axvspan(TURN[0] * DT, TURN[1] * DT, color="orange", alpha=0.15, label="true turn")
plt.xlabel("time (s)"); plt.ylabel("mode probability")
plt.title("IMM mode probabilities (shifts to CT during the turn)")
plt.legend(); plt.grid(True, alpha=0.3); plt.tight_layout()
plt.savefig("imm_modes.png", dpi=120)
print("saved imm_trajectory.png and imm_modes.png")
