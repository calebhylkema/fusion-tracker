# Multi-Sensor Fusion Tracker — Project Specification

## Purpose

A defense-portfolio embedded systems project demonstrating real-time multi-sensor fusion: 24 GHz radar, camera, and IMU data merged via an Extended Kalman Filter on an STM32, with a Raspberry Pi 5 handling camera-based detection and live visualization. The rig is mounted on a **hand-held / panning platform** so the IMU has a real job (rotating measurements into a stabilized world frame), which is also what makes the fusion non-trivial. Designed to showcase skills directly relevant to perception/autonomy roles at Anduril, L3Harris, Northrop Grumman, and Raytheon.

---

## Resume / Portfolio Framing

**Target one-liner:**
> Designed and built a real-time multi-sensor fusion tracker fusing 24 GHz radar, camera (YOLOv8), and 9-DOF IMU through a hand-written Extended Kalman Filter on an STM32F446RE (bare-metal C, no RTOS), with CAN 2.0B output and a live visualization dashboard on a Raspberry Pi 5.

**Key talking points for interviews:**

- **Sensor fusion architecture** — chose complementary modalities (radar for range/velocity, camera for classification + bearing, IMU for frame stabilization / ego-motion compensation), defined measurement models, and tuned process/measurement noise covariances.
- **EKF implemented from scratch** — no library black-box for the filter: hand-wrote the predict/update loop, state vector, Jacobians, and gating in C. Peripheral drivers use ST HAL/LL; the *fusion math* is all mine.
- **Honest nonlinearity story** — the radar reports Cartesian target positions (linear measurement), so the filter is an EKF because the **camera bearing** (`atan2`) and **IMU yaw rotation** of the measurement frame are nonlinear. I can explain exactly why an EKF is warranted rather than a plain linear KF.
- **Cross-modal data association** — associate camera detections with radar tracks in a shared, calibrated coordinate frame; Global Nearest Neighbor (GNN) with Mahalanobis gating, plus track initiation/deletion logic.
- **Real-time embedded constraints** — deterministic 50 Hz loop on STM32 using a hardware-timer-driven superloop, DMA-driven UART for radar ingest, interrupt-driven IMU reads.
- **CAN bus output** — fused track state broadcast over CAN 2.0B, demonstrating integration into vehicle/platform bus architectures.
- **Camera-in-the-loop detection** — YOLOv8-nano on Pi 5 for bounding-box detections, projected into the radar frame for cross-modal association with explicit timestamping/latency handling.
- **Full-stack ownership** — from hardware BOM and wiring to firmware, detection pipeline, fusion algorithm, calibration, and visualization.

> **Scope honesty (important for interviews):** the HLK-LD2450 does its own on-board tracking and reports up to **3** Cartesian targets. This project does **not** claim to invent radar tracking; the contribution is the **fusion layer** — stabilizing measurements with the IMU, associating camera detections to radar tracks in a common frame, running a hand-written EKF, and shipping the result over CAN with a live dashboard.

---

## Project Scope — V1 (build now) vs. V2 (stretch)

**Decision (2026-09-18):** build **V1 with the HLK-LD2450 already on hand** and make it excellent
through rigorous validation and V&V. Do **not** swap radars up front — the raw-radar signal-processing
path is a V2 stretch, not the thing that makes this project impressive.

**V1 — the project that ships:**
- LD2450 (Cartesian tracks) + camera (YOLOv8 bearing) + BNO085 (IMU frame stabilization) → hand-written EKF on STM32 → CAN + Pi dashboard.
- Maneuvering targets handled with an **IMM** (Interacting Multiple Model) filter — the one "harder than textbook" algorithm for V1.
- Definition of done = the 7 differentiators: (1) quantified validation (NEES/NIS, radar-only-vs-fused RMSE), (2) real-time evidence (worst-case loop time/jitter, on-target unit tests, CI), (3) requirements→V&V traceability matrix, (4) clean write-up, (5) IMM, (6) graceful degradation / fault handling, (7) mission-relevant framing.

**V2 — only after V1 ships, if time allows:**
- Swap to a raw-point-cloud radar (TI **IWRL6432BOOST**, ~$50 from DigiKey — not reliably on Amazon) and write detection from scratch: CFAR → clustering → association → tracker. A *bonus on a working project*, never a prerequisite. (Acconeer XM125 is Amazon-available and raw, but range-only/no bearing, so it would reshape the fusion architecture.)
- Rationale for not doing it first: adds a second toolchain (Code Composer Studio + mmWave SDK) and a new rabbit hole to an already-full project; the biggest risk to a portfolio project is not finishing it.

---

## Hardware BOM (Exact Parts Purchased)

### Sensors
| Part | Description | Interface | Price |
|------|-------------|-----------|-------|
| JMT HLK-LD2450 | 24 GHz radar, on-board multi-target tracking (≤3 targets), reports Cartesian x/y + speed | UART (256000 baud) | ~$15 |
| BNO085 9-DOF IMU module (EC Buying) | Accel + Gyro + Mag, on-chip sensor fusion, game rotation vector | I2C or SPI | ~$20.49 |

### Compute
| Part | Description | Price |
|------|-------------|-------|
| NUCLEO-F446RE (STMicroelectronics) | STM32F446RE Nucleo-64 dev board — ARM Cortex-M4 @ 180 MHz, FPU, 512 KB flash, 128 KB SRAM, 2× bxCAN, ST-LINK debugger | $37.99 |
| Raspberry Pi 5 8GB | Quad-core Cortex-A76 @ 2.4 GHz, 8 GB LPDDR4X — runs YOLOv8 detection + visualization dashboard | ~$80 |

### Camera
| Part | Description | Price |
|------|-------------|-------|
| Raspberry Pi Camera Module 3 (SC0872) | 12 MP Sony IMX708, autofocus, CSI-2 connector, ~66° horizontal FOV (75° wide variant) | ~$25 |

### Communication / Adapters
| Part | Description | Price |
|------|-------------|-------|
| Waveshare SN65HVD230 CAN Board (2-pack) | 3.3V CAN transceiver breakout — direct connection to Nucleo CAN peripheral, no level shifting, ESD protection | $16.99 |
| CP2102 USB-TTL Serial Adapter, 3-pack (HJHYUL) | USB 2.0 to 5-pin UART, 3.3V/5V selectable — for radar UART ↔ PC debug and Pi ↔ STM32 UART bridge | $9.99 |

### Support / Infrastructure
| Part | Description | Est. Price |
|------|-------------|------------|
| 32 GB micro SD card | Pi OS + YOLOv8 model + application code | ~$8 |
| USB-C power supply | Pi 5 power — **official 27 W (5V/5A) recommended**; 5V/3A works only with limited USB draw | ~$10–15 |
| Pi Camera ribbon cable | CSI connection (if not included with Camera Module 3) | ~$5 |
| Breadboard + jumper wire kit | Prototyping interconnects | ~$12 |
| 120 Ω resistor assortment | CAN bus termination resistors | ~$6 |
| Mounting hardware | Rigid bracket holding radar + camera + IMU in fixed relative alignment (needed for calibration); handheld/panning grip | ~$10–15 |

**Estimated Total: ~$250–270**

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        SENSOR LAYER                             │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐ │
│  │  HLK-LD2450  │  │   BNO085     │  │  Pi Camera Module 3    │ │
│  │  24 GHz radar│  │   9-DOF IMU  │  │  (on Raspberry Pi 5)   │ │
│  │  x,y,speed   │  │              │  │                        │ │
│  │  UART 256k   │  │  I2C / SPI   │  │  CSI-2 → Pi GPU/ISP   │ │
│  └──────┬───────┘  └──────┬───────┘  └───────────┬────────────┘ │
│         │                 │                      │              │
└─────────┼─────────────────┼──────────────────────┼──────────────┘
          │                 │                      │
          ▼                 ▼                      ▼
┌─────────────────────────────────┐  ┌────────────────────────────┐
│      STM32F446RE (Nucleo)       │  │     Raspberry Pi 5 8GB     │
│      ─────────────────────      │  │     ──────────────────     │
│                                 │  │                            │
│  • Radar UART parser (DMA RX)   │  │  • YOLOv8-nano inference   │
│  • IMU driver (I2C/SPI + INT)   │  │  • Bounding box → bearing  │
│  • Frame stabilization (IMU yaw)│  │    projection (+timestamp) │
│  • EKF predict/update loop      │  │  • Send detections to STM32│
│  • Data association (GNN)       │  │    via UART                │
│  • Track management             │  │  • Receive fused tracks    │
│  • CAN TX: fused track state    │  │    from STM32 via UART     │
│  • UART TX: fused tracks → Pi   │  │  • Live visualization      │
│                                 │  │    dashboard (PyQt/Pygame) │
│  Timing: hardware timer-driven  │  │                            │
│  superloop @ 50 Hz              │  │                            │
└────────────┬────────────────────┘  └────────────────────────────┘
             │
             ▼
     ┌───────────────┐
     │   CAN Bus     │
     │   (CAN 2.0B)  │
     │  Fused track  │
     │  broadcast    │
     │  for external │
     │  consumers    │
     └───────────────┘
```

### Data Flow Summary

1. **Radar → STM32**: HLK-LD2450 streams a fixed 30-byte target frame over UART at 256000 baud (header `AA FF 03 00`, three 8-byte target slots, tail `55 CC`). DMA receives into a buffer with idle-line detection. Parser extracts up to 3 × (x, y, speed), converting **mm→m** and **cm/s→m/s** at the driver boundary. These are **Cartesian** measurements.
2. **IMU → STM32**: BNO085 provides the game/AR-VR-stabilized rotation vector (quaternion) at ~100 Hz. Yaw is used to rotate sensor-frame measurements into the stabilized world frame (and, if the rig translates, accel for ego-motion in the prediction step).
3. **Camera → Pi**: Camera Module 3 feeds frames into YOLOv8-nano. Each detection becomes a **bearing-only** measurement (azimuth from pixel column via camera intrinsics) plus a class label, **timestamped** on the Pi. Pi sends these to STM32 over a dedicated UART link.
4. **EKF on STM32**: Runs at 50 Hz. Prediction uses a constant-velocity model (IMU-aided when the rig moves). Radar updates are **linear** Cartesian position/velocity; camera updates are **nonlinear** bearing (`atan2`) — this is what makes it an EKF. Sequential update per measurement, gated by Mahalanobis distance.
5. **STM32 → CAN**: Fused track state (ID, x, y, vx, vy, classification, covariance summary) broadcast on CAN 2.0B.
6. **STM32 → Pi (UART)**: Same fused track state sent to Pi for the dashboard overlay on the camera feed.

---

## Coordinate Frames & Units

- **World frame (W):** radar-centric, right-handed. **x forward** (radar boresight), **y left**, z up. This is the frame the EKF state lives in.
- **Sensor frame (S):** rigidly attached to the bracket; rotates with the handheld rig. IMU yaw `ψ` gives the S→W rotation about z. Radar reports in S; measurements are rotated into W before the update (or the measurement Jacobian carries the rotation).
- **Camera frame (C):** related to S by a fixed extrinsic transform found during calibration (see below).
- **Units:** the filter is **SI throughout** — meters, m/s, radians. LD2450 native mm and cm/s are converted at the driver boundary; camera pixels → radians via intrinsics. Only the CAN packing layer uses scaled integers (cm, 0.1 m/s).

---

## Pin / Wiring Map (NUCLEO-F446RE)

> **Rationale:** USART2 (PA2/PA3) is wired to the ST-LINK Virtual COM Port on the Nucleo, so it's reserved for printf debug. Radar moves to USART1 and the Pi link stays on USART6. CAN1 stays on PA11/PA12 because PB8/PB9 are consumed by I2C1 (they are the only other CAN1 option on this package).

| Peripheral | STM32 Pin(s) | AF | Connection | Notes |
|---|---|---|---|---|
| **USART1 (Radar RX)** | PB6 (TX), **PB7 (RX)** | AF7 | HLK-LD2450 TX→PB7 | 256000 baud, 8N1. **DMA2 Stream 2 or 5, Ch 4** for USART1_RX. Idle-line IRQ frames each 30-byte packet. Radar TX from STM32 (PB6) only needed for config. |
| **I2C1 (IMU)** | PB8 (SCL), PB9 (SDA) | AF4 | BNO085 SCL/SDA | 400 kHz fast mode. INT → **PB5 (EXTI5)** data-ready; **RST → PB4**. Watch clock-stretching (see risk). |
| **USART6 (Pi link)** | PC6 (TX), PC7 (RX) | AF8 | Pi GPIO UART RX/TX | 115200 baud. Fused tracks out + camera detections in. |
| **CAN1** | PA11 (RX), PA12 (TX) | AF9 | SN65HVD230 breakout | 500 kbps, 120 Ω termination at both physical ends of the bus. |
| **Debug UART (USART2)** | PA2 (TX), PA3 (RX) | AF7 | ST-LINK VCP (USB) | printf debug output — kept free by moving radar to USART1. |
| **Loop tick** | TIM (e.g. TIM6) | — | internal | 50 Hz update interrupt drives the fusion superloop. |

**Power notes:**
- Nucleo powered via USB from PC (ST-LINK) or 5V on VIN.
- SN65HVD230 powered from Nucleo 3.3V — no level shifting.
- BNO085 module has an on-board 3.3V regulator; run from 3.3V or 5V per the breakout. Set PS0/PS1 for the chosen interface (I2C vs SPI) and honor the reset/boot sequence.
- HLK-LD2450 needs 5V supply (Nucleo 5V pin or separate). Its UART logic is 3.3V — safe into STM32 RX.

---

## Software Architecture

### STM32 Firmware (C, STM32CubeIDE + HAL/LL, bare-metal superloop)

Recommended build: **STM32CubeIDE + CubeMX pin config + HAL/LL drivers**, with a **hardware-timer-driven superloop (no RTOS)** for a deterministic 50 Hz loop. This satisfies "get familiar with CubeIDE," keeps timing deterministic, and leaves the EKF as the hand-written centerpiece. FreeRTOS is a stretch option only.

```
firmware/
├── Core/
│   ├── main.c                  // System init, 50 Hz superloop
│   ├── sysclk_config.c         // 180 MHz clock tree (overdrive) — CubeMX generated
│   └── it_handlers.c           // IRQ handlers (UART DMA/idle, I2C, EXTI, TIM)
├── Drivers/
│   ├── hlk_ld2450.c/.h         // Radar UART parser — 30-byte frame sync, x/y/speed, mm→m
│   ├── bno085.c/.h             // IMU driver — SHTP protocol, rotation-vector (quaternion) mode
│   ├── can_output.c/.h         // CAN TX — message packing, arbitration ID scheme
│   └── uart_pi_link.c/.h       // Pi ↔ STM32 protocol — detection RX, track TX
├── Fusion/
│   ├── ekf.c/.h                // EKF core — state, predict, update, Jacobians (hand-written)
│   ├── measurement_models.c/.h // Radar (linear Cartesian), camera (nonlinear bearing)
│   ├── frame_transform.c/.h    // IMU yaw → S↔W rotation of measurements
│   ├── data_association.c/.h   // GNN gating + assignment (Hungarian/auction)
│   └── track_manager.c/.h      // Track init (M-of-N), confirmation, coasting, deletion
├── App/
│   ├── fusion_task.c           // 50 Hz loop — predict, associate, update, output
│   └── system_monitor.c        // Watchdog, timing stats, error counters
└── CMakeLists.txt / .ioc       // CubeIDE project + CubeMX config
```

### Raspberry Pi Software (Python)

```
pi_app/
├── detection/
│   ├── yolo_detector.py        // YOLOv8-nano inference (Ultralytics / ncnn export)
│   ├── camera_capture.py       // Picamera2 CSI capture pipeline
│   └── bearing_projection.py   // Bounding-box center px → azimuth via intrinsics (+timestamp)
├── calib/
│   ├── intrinsics.py           // Camera calibration (OpenCV checkerboard) → K matrix
│   └── extrinsics.py           // Camera→radar alignment solve + stored transform
├── comms/
│   ├── uart_bridge.py          // Serial link to STM32 — send detections, receive tracks
│   └── protocol.py             // Packet framing, CRC, serialize/deserialize, timestamps
├── viz/
│   ├── dashboard.py            // PyQt6 or Pygame real-time display
│   ├── track_overlay.py        // Draw fused tracks + covariance ellipses on camera feed
│   └── radar_plot.py           // Bird's-eye plot: raw radar vs fused tracks
└── main.py                     // Entry point — start capture, detection, comms, viz threads
```

---

## EKF Design

### State Vector

```
x = [px, py, vx, vy]^T    (4-state, constant-velocity model, world frame, SI units)
```

One filter instance per confirmed track. For maneuvering targets, V1 runs an **IMM (Interacting Multiple Model)** filter — a bank of models (e.g., constant-velocity + constant-acceleration / coordinated-turn) with Markov mode-switching — rather than a single CV model. This is the one deliberately non-textbook algorithm in V1 and directly supports the "handles maneuvering targets" talking point.

### Prediction

```
x̂(k|k-1) = F · x̂(k-1|k-1)   [+ B·u(k) when rig translates]
P(k|k-1) = F · P(k-1|k-1) · F^T + Q
```

Constant-velocity `F`. When the handheld rig **translates**, IMU acceleration feeds `u(k)` for ego-motion compensation. When the rig only **rotates** (panning), IMU yaw rotates incoming measurements into the world frame rather than entering `F`.

### Measurement Models

**Radar (HLK-LD2450) — LINEAR:** the sensor reports Cartesian position (and speed), so after rotating S→W the measurement is a linear function of the state:
```
z_radar = [px, py]^T                (optionally include a speed/velocity term)
H_radar = [[1,0,0,0],[0,1,0,0]]     (identity block — no Jacobian nonlinearity)
```
> This is the key correction from the draft: the LD2450 does **not** emit raw range/azimuth/range-rate, so modeling `h_radar = [range, azimuth, range_rate]` would be wrong for this sensor.

**Camera (YOLOv8 bearing) — NONLINEAR:** bearing-only measurement; this is what makes the filter an EKF:
```
z_cam = [azimuth]
h_cam(x) = atan2(py, px)
H_cam = ∂h/∂x = [ -py/(px²+py²),  px/(px²+py²),  0,  0 ]
```
Camera provides bearing + classification label. Fusing bearing with the radar's range/position resolves the range ambiguity of a monocular detection and attaches a class to a radar track.

**IMU:** not a direct EKF measurement of target state — it supplies the S→W rotation (yaw) applied to radar/camera measurements, and optional ego-motion in prediction.

### Data Association

- **Gating**: Mahalanobis distance below a chi-squared threshold (95%) with DoF matching the measurement dimension.
- **Assignment**: Global Nearest Neighbor (GNN) via the Hungarian algorithm across the ≤3 radar targets + camera detections.
- **Track lifecycle**: M-of-N initiation (e.g., 3-of-5), coast up to K missed updates, then delete.
- **Cross-modal**: camera detections are associated to existing tracks by bearing gate; a matched camera detection contributes its class label and a bearing update.

### Handling Asynchronous / Delayed Camera Measurements

YOLOv8-n on the Pi 5 CPU runs ~8–15 fps with ~100 ms latency — far slower than the 50 Hz EKF and asynchronous to it. Each detection carries a **Pi-side timestamp**; the STM32 applies it against a short buffer of recent states (retrodiction / out-of-sequence-measurement handling) or, at minimum, a fixed latency offset. Radar (fast, low-latency) drives the loop cadence; camera refines bearing/class when it arrives.

---

## Sensor Calibration

Cross-modal association only works if camera and radar live in one frame.

1. **Camera intrinsics** — OpenCV checkerboard calibration → focal length + principal point (`K`). Pixel column `u` → azimuth `θ = atan2(u − cx, fx)`.
2. **Camera → radar extrinsics** — with the bracket fixed, place a strong reflector/target at several known positions, record radar (x,y) and camera bearing simultaneously, and solve the rotation/translation that aligns camera bearings to radar positions. Store the transform.
3. **IMU alignment** — capture the IMU yaw offset when the rig points along the world x-axis (boresight), so yaw=0 corresponds to boresight.

---

## CAN Bus Message Definition

| CAN ID | Name | Payload (8 bytes) | Rate |
|--------|------|-------------------|------|
| 0x100 | TRACK_STATE | track_id (u8), px (i16, cm), py (i16, cm), vx (i8, 0.1 m/s), vy (i8, 0.1 m/s), class (u8) | 50 Hz per track |
| 0x101 | TRACK_COVARIANCE | track_id (u8), σ_px (u16, mm), σ_py (u16, mm), σ_vx (u16, mm/s), ρ_xy (i8, ×0.01) | 10 Hz per track |
| 0x1F0 | SYSTEM_STATUS | num_tracks (u8), loop_time_us (u16), sensor_ok bitfield (radar/imu/cam), flags | 1 Hz |

Notes: all TRACK_STATE frames share ID **0x100** with `track_id` in the payload (optionally use ID `0x100 + track_id` so consumers can filter by arbitration ID). i16 cm gives ±327 m; i8 at 0.1 m/s gives ±12.7 m/s — ample for pedestrians. Covariance fields are 1-σ in the units listed; `ρ_xy` is the px/py correlation coefficient scaled ×0.01.

---

## EKF Validation

- **Synthetic trajectories**: generate known paths (constant-velocity, turning) with injected noise; confirm the filter tracks and the covariance behaves.
- **Consistency (NEES/NIS)**: run Normalized Estimation Error Squared (with ground truth) and Normalized Innovation Squared (measurement-only) chi-square tests to tune Q/R — the standard, defensible way to show the filter is *consistent*, not just visually plausible.
- **Ground truth**: tape-measure / floor-marked target positions for a real-world accuracy number (position RMSE), plus radar-only vs. fused comparison for the demo.

---

## Risk Register

| Risk | Impact | Mitigation |
|---|---|---|
| LD2450 does its own tracking → less EKF novelty | Weakens the "fusion from scratch" story | Frame the contribution as the **fusion/association/IMU-stabilization layer**; keep EKF hand-written and lean on the camera-bearing nonlinearity. |
| BNO085 I2C clock-stretching / SHTP quirks | IMU unreliable or stalls the bus | Prefer **SPI** if I2C is flaky; honor reset/boot + PS0/PS1; add bus timeout + recovery. |
| Camera↔radar calibration error | Bad cross-modal association | Dedicated calibration step + rigid bracket; validate with a known target grid. |
| YOLO/Pi latency & low fps | Stale camera updates corrupt tracks | Timestamp detections; OOSM/retrodiction or fixed-latency offset; radar drives loop cadence. |
| PA2/PA3 USART2 ↔ ST-LINK VCP conflict | Lose debug or radar link | Radar on **USART1**, debug on **USART2/VCP** (already reflected in pin map). |
| Handheld rig too shaky | IMU stabilization insufficient | Start on a **static tripod** for bringup; add panning only once the static case works. |

---

## Timing Budget (50 Hz loop = 20 ms window, M4 @ 180 MHz)

| Stage | Est. cost | Notes |
|---|---|---|
| Radar frame parse | few µs | 30-byte fixed frame, DMA + idle IRQ |
| IMU read + quaternion→yaw | tens of µs | I2C/SPI transaction dominates |
| Predict (per track) | < 5 µs | 4×4 matrix ops, FPU |
| Associate (GNN) | tens of µs | Hungarian over ≤3 targets — trivial |
| Update (per measurement) | < 10 µs | small matrices, FPU |
| CAN + UART TX | tens of µs | non-blocking / interrupt-driven |
| **Total** | **≪ 20 ms** | Large headroom — compute is not the constraint; determinism is. |

---

## Development Phases

### Phase 0 — Requirements & Repo (Week 0–1)
- [ ] Write `docs/requirements.md`: numbered requirements + a **verification matrix** (each requirement → how it's verified → result). Fill results as phases complete.
- [ ] Repo scaffold, README, CI stub (build firmware + run Pi/sim unit tests).

### Phase 1 — Bringup & Drivers (Week 1–2)
- [ ] CubeMX/CubeIDE project: 180 MHz clock tree, GPIO, USART1/USART2/USART6, I2C1, CAN1, TIM tick
- [ ] HLK-LD2450 UART parser: 30-byte frame sync via DMA + idle IRQ, extract (x, y, speed), mm→m
- [ ] BNO085 driver: rotation-vector (quaternion) mode, verify yaw stream (try I2C, fall back to SPI)
- [ ] CAN loopback test with SN65HVD230 transceivers, 120 Ω termination
- [ ] Pi Camera Module 3 capture with Picamera2
- [ ] YOLOv8-nano inference benchmark on Pi 5 (measure fps + latency; consider ncnn export)

### Phase 2 — EKF Core (Week 3–4)
- [ ] Hand-written EKF predict/update in C with FPU matrix ops
- [ ] Radar **linear** Cartesian measurement model (H = identity block)
- [ ] Camera **nonlinear** bearing model + Jacobian
- [ ] IMU yaw S→W frame transform applied to measurements
- [ ] Unit-test EKF with synthetic trajectories; NEES/NIS consistency; tune Q and R

### Phase 3 — Data Association & Track Management (Week 5)
- [ ] Mahalanobis gating
- [ ] Hungarian GNN assignment (radar targets + camera detections)
- [ ] M-of-N track initiation, coasting, deletion
- [ ] **IMM** filter (CV + CA / coordinated-turn) for maneuvering targets; compare vs. single-CV on a turning trajectory
- [ ] Multi-target stress test (up to 3 radar targets) with simulated clutter

### Phase 4 — Integration, Calibration & Visualization (Week 6–7)
- [ ] Camera intrinsics + camera→radar extrinsics calibration
- [ ] Pi ↔ STM32 UART protocol with timestamps: detections downstream, fused tracks upstream
- [ ] CAN output with defined message IDs
- [ ] Live dashboard: camera feed + track overlay + bird's-eye plot + covariance ellipses
- [ ] End-to-end test with real targets (start static tripod, then panning)

### Phase 5 — Polish & Documentation (Week 8)
- [ ] Demo video: multiple people walking, radar-only vs. fused
- [ ] GitHub README: architecture diagram, build instructions, calibration, results
- [ ] Metrics: loop timing, position RMSE vs. ground truth, latency, NEES/NIS plots
- [ ] Clean code, comments, push to public repo

---

## Portfolio Deliverables

1. **GitHub repo** — clean, well-documented, architecture diagrams in README
2. **Demo video** (60–90 sec) — live dashboard tracking multiple targets, radar-only vs. fused
3. **Technical write-up** — 2–3 page PDF: design decisions, EKF derivation, calibration, NEES/NIS results
4. **Resume bullet** — concise line + expanded portfolio version
5. **Architecture diagram** — polished block diagram for presentations

---

## Key Datasheets & References

- **STM32F446RE**: [ST Reference Manual RM0390](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- **NUCLEO-F446RE**: [User Manual UM1724](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf) — confirms USART2↔ST-LINK VCP solder-bridge wiring
- **HLK-LD2450**: "HLK-LD2450 serial communication protocol" — 30-byte report frame, header `AA FF 03 00`, Cartesian x/y (mm) + speed (cm/s), ≤3 targets
- **BNO085 / BNO08x**: [Datasheet + SH-2 / SHTP reference](https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf)
- **SN65HVD230**: [TI Datasheet](https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf)
- **Pi Camera Module 3**: [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)
- **YOLOv8**: [Ultralytics Docs](https://docs.ultralytics.com/)
- **Filter consistency (NEES/NIS)**: Bar-Shalom, *Estimation with Applications to Tracking and Navigation*
