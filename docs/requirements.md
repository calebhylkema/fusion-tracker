# Requirements & Verification Matrix

Written defense-style: numbered requirements, each with an explicit verification method and a result
field filled in as the project progresses. This traceability is a first-class deliverable (differentiator #3).

Verification method key: **T** = test (automated), **D** = demonstration, **A** = analysis, **I** = inspection.

## Functional requirements

| ID | Requirement | Verify | Method | Result |
|----|-------------|--------|--------|--------|
| FR-1 | Parse HLK-LD2450 UART frames and output up to 3 targets as (x, y, speed) in SI units. | Feed recorded/synthetic frames; compare parsed values to known input. | T | _TBD_ |
| FR-2 | Read BNO085 and produce a yaw estimate used to rotate measurements into the world frame. | Rotate rig by a known angle; yaw tracks within tolerance. | D | _TBD_ |
| FR-3 | Run YOLOv8-nano on the Pi and emit timestamped bearing+class detections over UART. | Known target at known bearing; detection bearing within tolerance. | T/D | _TBD_ |
| FR-4 | Fuse radar (linear) + camera (nonlinear bearing) in a hand-written EKF producing (px,py,vx,vy). | Synthetic-trajectory unit tests; RMSE vs. ground truth. | T/A | _TBD_ |
| FR-5 | Handle maneuvering targets with an IMM filter (CV + CA/coordinated-turn). | Turning-target trajectory; IMM beats single-CV on RMSE. | T/A | _TBD_ |
| FR-6 | Associate measurements to tracks (Mahalanobis gating + GNN/Hungarian) with M-of-N init & deletion. | Multi-target + clutter sim; correct assignments, no ID swaps. | T | _TBD_ |
| FR-7 | Broadcast fused track state on CAN 2.0B per the message definition. | CAN sniffer decodes TRACK_STATE/COVARIANCE/STATUS correctly. | T/D | _TBD_ |
| FR-8 | Live Pi dashboard overlays fused tracks + covariance on the camera feed and a bird's-eye plot. | Visual demo with moving targets. | D | _TBD_ |

## Performance / real-time requirements

| ID | Requirement | Verify | Method | Result |
|----|-------------|--------|--------|--------|
| PR-1 | Fusion loop runs deterministically at 50 Hz. | Measure loop period over a run; jitter within budget. | T/A | _TBD_ |
| PR-2 | Worst-case loop execution time has margin below the 20 ms budget. | Cycle-count / timer instrumentation, log max. | A | _TBD_ |
| PR-3 | End-to-end position estimate meets an RMSE target vs. tape-measured ground truth. | Marked positions; compute RMSE. | A | _TBD_ |
| PR-4 | Fusion improves accuracy vs. radar-only. | Ablation: radar-only vs. fused RMSE on same run. | A | _TBD_ |

## Robustness requirements (differentiator #6)

| ID | Requirement | Verify | Method | Result |
|----|-------------|--------|--------|--------|
| RR-1 | Detect and flag sensor dropout (radar/IMU/camera) and enter a safe degraded mode. | Fault injection (unplug/stall each sensor); system flags + coasts. | T/D | _TBD_ |
| RR-2 | Recover automatically when a dropped sensor returns. | Restore sensor; tracking resumes. | D | _TBD_ |

## Quality / consistency requirements (differentiator #1)

| ID | Requirement | Verify | Method | Result |
|----|-------------|--------|--------|--------|
| QR-1 | EKF/IMM is statistically consistent (NEES within chi-square bounds on sim). | Monte-Carlo NEES over N runs. | A | _TBD_ |
| QR-2 | Innovations are consistent (NIS within bounds) on real data. | NIS from logged innovations. | A | _TBD_ |

> Fill the **Result** column with a value + link to the test/plot as each phase completes. A requirement
> is "verified" only when Result is populated with evidence.
