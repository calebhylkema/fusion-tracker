# sim/ — EKF/IMM Python sandbox

Prototype and validate the filter here **before** porting to C on the STM32. Pure software, runs on
the dev PC. This is where differentiator #1 (quantified validation) is earned.

Planned contents (Phase 2):
- `trajectory.py` — synthetic target trajectories (constant-velocity, coordinated-turn) + noise.
- `ekf.py` — reference EKF (radar linear + camera bearing nonlinear) to mirror the C implementation.
- `imm.py` — Interacting Multiple Model filter (CV + CA/CT).
- `consistency.py` — NEES / NIS chi-square tests for Q/R tuning.
- `run_*.py` — experiments that produce the plots/numbers cited in `docs/`.

Setup: see repo `SETUP.md` §2. TL;DR: `python -m venv .venv` → activate → `pip install -r sim/requirements.txt`.
