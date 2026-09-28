# Requirements & Verification Matrix

Written defense-style: numbered requirements, each with an explicit verification method and a result
field backed by evidence. This traceability is a first-class deliverable (differentiator #3).

Verification method key: **T** = test (automated), **D** = demonstration, **A** = analysis, **I** = inspection.
Status key: ✅ verified · 🟡 partial / stretch remaining · ⬜ planned.

## Functional requirements

| ID | Requirement | Method | Result |
|----|-------------|--------|--------|
| FR-1 | Parse HLK-LD2450 UART frames and output up to 3 targets as (x, y, speed) in SI units. | T/I | ✅ Header-synced, checksum-validated parser with sign-magnitude decode; live targets verified. `firmware/.../ld2450.c` |
| FR-2 | Read BNO085 and produce a yaw estimate used to rotate measurements into the world frame. | D | ✅ UART-RVC parser; yaw tracks rig rotation, used for world-frame rotation (startup-zeroed). `firmware/.../bno085.c`, `fusion.c` |
| FR-3 | Run YOLOv8-nano on the Pi and emit bearing+class detections over UART. | T/D | ✅ YOLOv8-n → person bearing over GPIO UART (`/dev/ttyAMA0`); received and fused live. `pi_app/detect_and_send.py` |
| FR-4 | Fuse radar (linear) + camera (nonlinear bearing) in a hand-written EKF producing (px,py,vx,vy). | T/A | ✅ Runs live on the MCU and in sim; **35.3% RMSE improvement** vs. radar-only. `fusion.c`, `sim/run_fusion.py` |
| FR-5 | Handle maneuvering targets with an IMM filter (CV + coordinated-turn). | T/A | ✅ IMM beats single-CV by **13% in turns** (0.123 m vs. 0.142 m). `sim/imm.py`, `sim/run_imm.py` |
| FR-6 | Associate measurements to tracks (gating) with clutter rejection. | T | 🟡 Nearest-neighbor gating rejects radar ghosts/multipath (single target). Full GNN/Hungarian multi-target = stretch. `fusion.c` |
| FR-7 | Broadcast fused track state on CAN 2.0B. | T/D | 🟡 CAN1 bring-up + loopback verified; live fused-track broadcast is the next integration step. `firmware/.../can_bus.c` |
| FR-8 | Live dashboard overlays fused track, camera bearing, and IMU heading on a bird's-eye plot. | D | ✅ Host-PC matplotlib dashboard over ST-LINK VCP telemetry; radar dots + fused track + trail + camera ray + heading. `dashboard/live_dashboard.py` |

## Performance / real-time requirements

| ID | Requirement | Method | Result |
|----|-------------|--------|--------|
| PR-1 | Fusion loop runs deterministically (timer-driven super-loop, no RTOS). | T/A | 🟡 Super-loop runs; telemetry emitted at 10 Hz. Formal loop-jitter measurement pending. |
| PR-2 | Worst-case loop execution time has margin below budget. | A | ⬜ Cycle-count instrumentation pending (M4@180 MHz has ample headroom for a 4-state EKF). |
| PR-3 | End-to-end position estimate meets an RMSE target vs. tape-measured ground truth. | A | ⬜ Real-world tape-measure ground-truth run pending. |
| PR-4 | Fusion improves accuracy vs. radar-only. | A | ✅ Ablation in sim: 0.218 m (radar-only) → 0.141 m (fused) = **35.3%**. `sim/run_fusion.py` |

## Robustness requirements (differentiator #6)

| ID | Requirement | Method | Result |
|----|-------------|--------|--------|
| RR-1 | Tolerate sensor dropout: coast the track and drop it cleanly instead of diverging. | T/D | 🟡 Track coasts through short dropouts, then deletes after `COAST_MAX`; per-sensor OK flags in telemetry. |
| RR-2 | Recover automatically when a link glitches or a sensor returns. | D | ✅ UART overrun-error auto-recovery (abort + clear ORE + re-arm); links resume without reset. `ld2450.c`, `bno085.c`, `camera_link.c` |

## Quality / consistency requirements (differentiator #1)

| ID | Requirement | Method | Result |
|----|-------------|--------|--------|
| QR-1 | EKF/IMM is statistically consistent (NEES within chi-square bounds). | A | ✅ NEES mean **3.99** (expect 4.0), 99% in bounds over the run. `sim/run_fusion.py` → `sim/nees.png` |
| QR-2 | Innovations are consistent (NIS within bounds). | A | ✅ NIS mean **2.00** (expect 2.0), 98% in bounds in sim. Live-data NIS logging pending. |

---

### Remaining / stretch work
- Live CAN broadcast of the real fused track (FR-7) and a CAN sniffer decode demo.
- Real-world RMSE vs. tape-measured ground truth (PR-3) and loop-timing instrumentation (PR-1/PR-2).
- Full GNN/Hungarian multi-target association with M-of-N track init/deletion (FR-6).
- V2 stretch: raw-point-cloud radar (TI IWRL6432) with CFAR → clustering written from scratch.
