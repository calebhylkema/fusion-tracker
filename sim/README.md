# sim/ — EKF/IMM Python sandbox

Prototype and validate the fusion filter here **before** porting to C on the STM32.
Pure software, runs on the dev PC. This is where differentiator #1 (quantified
validation) is earned.

## Files
- `ekf.py` — reference EKF. State `[px, py, vx, vy]` (constant velocity). Radar is a
  **linear** Cartesian position update; camera is a **nonlinear** bearing update
  (`atan2`) — the camera is what makes it an EKF. Mirrors the C firmware to come.
- `run_fusion.py` — scenario, Monte-Carlo validation, and plots.

## Run
```bash
python -m venv ../.venv && ../.venv/Scripts/python.exe -m pip install -r requirements.txt
../.venv/Scripts/python.exe run_fusion.py            # tuned Q=0.2
../.venv/Scripts/python.exe run_fusion.py 0.5         # override Q for tuning
```

## Scenario
One person walks across the sensor field of view ~6 m out. Radar reports Cartesian
`(x, y)` with **cross-range (x) error much larger than range (y) error** (coarse
angular resolution); the camera reports a **precise bearing** (~1°) but no range.

## Results (300 Monte-Carlo runs, 10 Hz)
| Metric | Radar-only | Radar + camera |
|---|---|---|
| Position RMSE | 0.218 m | **0.141 m** (**−35%**) |

- **NEES = 3.99** (expected 4), **NIS = 2.00** (expected 2), ~99% inside the 95%
  chi-square band → the filter is **statistically consistent** (not over/under-confident).
- The camera's bearing corrects the radar's weak cross-range — the fusion value, quantified.

![trajectory](trajectory.png)
![NEES consistency](nees.png)

## Tuning note (the validation loop)
The first attempt generated ground truth as *perfectly* constant-velocity (zero
process noise) while the filter assumed `Q > 0` → the filter was over-conservative
and **NEES failed at 2.17**. Fix: generate the truth with the *same* process-noise
model the filter assumes (standard consistency-test setup, Bar-Shalom), then tune the
process-noise density `Q` until NEES sits at the state dimension. This NEES-driven
tuning is exactly how you justify the covariance choices to a reviewer.

## IMM — maneuvering targets (`imm.py`, `run_imm.py`)
Bank of models (constant-velocity + coordinated-turn left/right) with Markov
switching. On a straight -> turn -> straight path (200 Monte-Carlo runs):

| Position RMSE | single-CV EKF | IMM |
|---|---|---|
| during the turn | 0.142 m | **0.123 m** (**13% better**) |

The mode probabilities ride CV on the straights and shift to the coordinated-turn
model through the maneuver (`imm_modes.png`); the advantage grows with sharper
maneuvers / sparser measurements.

![IMM trajectory](imm_trajectory.png)
![IMM modes](imm_modes.png)

## Live viewers (interactive — a window opens and plays)
```bash
../.venv/Scripts/python.exe live_fusion.py   # target + radar-only vs fused + covariance ellipse
../.venv/Scripts/python.exe live_imm.py      # maneuvering target + live mode-probability bars
```
`live_fusion.py` — watch the fused estimate (red) hug the truth while the radar-only
estimate (blue) wanders; the red ellipse is the 2-sigma uncertainty. `live_imm.py`
— watch the mode-probability bars swing from CV to a turn model exactly when the
target starts turning.

## Next
- Multi-target data association (GNN / Hungarian gating) with track init/delete.
- Port the filters to C in the firmware `Fusion/` module, consuming `ld2450_frame_t`
  and the BNO085 yaw for the world-frame rotation.
