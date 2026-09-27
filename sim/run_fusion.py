"""Fusion-tracker EKF demo & validation.

Scenario: one person walks left->right across the sensor's field of view, ~6 m out.
Sensors:
  - Radar: Cartesian (x, y). Realistically, radar CROSS-RANGE (x) error is much
    larger than its RANGE (y) error (coarse angular resolution).
  - Camera: precise BEARING atan2(py, px) (~1 deg), but no range on its own.

We show:
  1. Radar-only vs radar+camera-fused position RMSE (the camera bearing fixes the
     radar's poor cross-range) -> the value of fusion.
  2. NEES / NIS consistency (Monte Carlo) -> the filter is statistically sound.
     Ground truth is generated with the SAME process-noise model the filter
     assumes (standard consistency-test setup), so a correct filter yields
     NEES ~= state dim (4) and NIS ~= radar meas dim (2).

Outputs: printed metrics + two PNGs (trajectory, NEES).
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.stats import chi2

from ekf import EKF

# ---------------------------------------------------------------- scenario ---
DT = 0.1
T = 12.0
N = int(T / DT)
X0_TRUE = np.array([-3.0, 6.0, 1.0, 0.0])   # start left, walk +x at 1 m/s, ~6 m out
F = np.array([[1, 0, DT, 0], [0, 1, 0, DT], [0, 0, 1, 0], [0, 0, 0, 1]], float)

# ---------------------------------------------------------------- sensors ----
SIG_RX, SIG_RY = 0.40, 0.15          # radar: cross-range (x) worse than range (y)
SIG_BEARING = np.deg2rad(1.0)        # camera: precise bearing
R_RADAR = np.diag([SIG_RX ** 2, SIG_RY ** 2])
R_CAM = np.array([[SIG_BEARING ** 2]])
Q_SPECTRAL = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2  # tuned via NEES (see below)


def make_Q(q, dt):
    dt3, dt2 = dt ** 3 / 3.0, dt ** 2 / 2.0
    return q * np.array([[dt3, 0, dt2, 0], [0, dt3, 0, dt2],
                         [dt2, 0, dt, 0], [0, dt2, 0, dt]], float)


def simulate(seed, q):
    """Generate one truth realization (CV + process noise) and its measurements."""
    rng = np.random.default_rng(seed)
    Q = make_Q(q, DT)
    truth = np.zeros((N, 4))
    zr = np.zeros((N, 2))
    zb = np.zeros(N)
    x = X0_TRUE.copy()
    for k in range(N):
        truth[k] = x
        zr[k] = x[:2] + rng.normal(0, [SIG_RX, SIG_RY])
        zb[k] = np.arctan2(x[1], x[0]) + rng.normal(0, SIG_BEARING)
        x = F @ x + rng.multivariate_normal(np.zeros(4), Q)
    return truth, zr, zb


def run_filter(truth, zr, zb, use_camera, q):
    ekf = EKF(DT, q=q, x0=[truth[0, 0], truth[0, 1], 0.0, 0.0],
              P0=np.diag([1.0, 1.0, 4.0, 4.0]))
    est = np.zeros((N, 4))
    nees = np.zeros(N)
    nis = np.zeros(N)
    for k in range(N):
        ekf.predict()
        y, S = ekf.update_radar(zr[k], R_RADAR)
        nis[k] = y @ np.linalg.inv(S) @ y
        if use_camera:
            ekf.update_camera(zb[k], R_CAM)
        est[k] = ekf.x
        e = truth[k] - ekf.x
        nees[k] = e @ np.linalg.inv(ekf.P) @ e
    return est, nees, nis


def rmse(est, truth):
    return np.sqrt(np.mean(np.sum((est[:, :2] - truth[:, :2]) ** 2, axis=1)))


# -------------------------------------------------------- Monte Carlo runs ---
M = 300
rmse_r, rmse_f = [], []
nees_acc = np.zeros(N)
nis_acc = np.zeros(N)
for m in range(M):
    truth, zr, zb = simulate(m, Q_SPECTRAL)
    est_r, _, _ = run_filter(truth, zr, zb, use_camera=False, q=Q_SPECTRAL)
    est_f, nees_f, nis_f = run_filter(truth, zr, zb, use_camera=True, q=Q_SPECTRAL)
    rmse_r.append(rmse(est_r, truth))
    rmse_f.append(rmse(est_f, truth))
    nees_acc += nees_f
    nis_acc += nis_f
nees_avg = nees_acc / M
nis_avg = nis_acc / M

# steady state = after the initial convergence transient (first 2 s)
ss = slice(int(2.0 / DT), N)

print("=" * 66)
print(f"Monte Carlo: {M} runs, {N} steps @ {1/DT:.0f} Hz, Q={Q_SPECTRAL}")
print("-" * 66)
print(f"Position RMSE  radar-only : {np.mean(rmse_r):.3f} m")
print(f"Position RMSE  radar+cam  : {np.mean(rmse_f):.3f} m")
print(f"Fusion improvement        : {100*(1-np.mean(rmse_f)/np.mean(rmse_r)):.1f}%")
print("-" * 66)
n_state, n_meas = 4, 2
nees_lo, nees_hi = chi2.ppf([0.025, 0.975], M * n_state) / M
nis_lo, nis_hi = chi2.ppf([0.025, 0.975], M * n_meas) / M
frac_nees = np.mean((nees_avg[ss] >= nees_lo) & (nees_avg[ss] <= nees_hi))
frac_nis = np.mean((nis_avg[ss] >= nis_lo) & (nis_avg[ss] <= nis_hi))
print(f"NEES (steady) mean={nees_avg[ss].mean():.2f} (expect {n_state}) "
      f"bounds=[{nees_lo:.2f},{nees_hi:.2f}]  in-bounds={100*frac_nees:.0f}%")
print(f"NIS  (steady) mean={nis_avg[ss].mean():.2f} (expect {n_meas}) "
      f"bounds=[{nis_lo:.2f},{nis_hi:.2f}]  in-bounds={100*frac_nis:.0f}%")
print("=" * 66)

# -------------------------------------------------------------- plots --------
truth0, zr0, zb0 = simulate(0, Q_SPECTRAL)
est_r0, _, _ = run_filter(truth0, zr0, zb0, False, Q_SPECTRAL)
est_f0, _, _ = run_filter(truth0, zr0, zb0, True, Q_SPECTRAL)
plt.figure(figsize=(7, 6))
plt.scatter(zr0[:, 0], zr0[:, 1], s=10, c="0.7", label="radar measurements")
plt.plot(truth0[:, 0], truth0[:, 1], "k-", lw=2, label="ground truth")
plt.plot(est_r0[:, 0], est_r0[:, 1], "b--", lw=1.5, label="radar-only EKF")
plt.plot(est_f0[:, 0], est_f0[:, 1], "r-", lw=1.5, label="radar+camera EKF")
plt.scatter([0], [0], marker="^", c="g", s=120, label="sensor")
plt.xlabel("x - cross-range (m)")
plt.ylabel("y - range (m)")
plt.title("Fusion tracker: ground truth vs radar-only vs fused")
plt.legend()
plt.axis("equal")
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("trajectory.png", dpi=120)

t = np.arange(N) * DT
plt.figure(figsize=(8, 4))
plt.plot(t, nees_avg, "b-", lw=1, label="avg NEES (fused)")
plt.axhline(n_state, color="k", ls=":", label=f"expected = {n_state}")
plt.axhspan(nees_lo, nees_hi, color="g", alpha=0.15, label="95% chi-square band")
plt.xlabel("time (s)")
plt.ylabel("NEES")
plt.title(f"Filter consistency (NEES), {M} Monte Carlo runs, Q={Q_SPECTRAL}")
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("nees.png", dpi=120)
print("saved trajectory.png and nees.png")
