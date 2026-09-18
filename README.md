# Multi-Sensor Fusion Tracker

Real-time multi-sensor fusion demonstrator: a 24 GHz radar, a camera (YOLOv8), and a 9-DOF IMU are
fused through a **hand-written Extended Kalman Filter on an STM32F446RE** (bare-metal C, no RTOS),
with fused tracks broadcast over **CAN 2.0B** and shown on a live **Raspberry Pi 5** dashboard.

> Portfolio project targeting perception / autonomy / embedded roles in defense & aerospace.
> The contribution is the **fusion layer** — IMU frame stabilization, cross-modal (radar↔camera)
> association, a hand-written EKF/IMM, quantified validation, and CAN output. The radar module does
> its own low-level tracking; this project does not claim to reinvent that (see honest framing below).

## Architecture (V1)

```
Radar (HLK-LD2450, UART) ─┐
IMU  (BNO085, I2C)  ──────┼──► STM32F446RE ──► EKF/IMM + GNN assoc + track mgmt ──► CAN 2.0B
                          │        ▲                                             └► UART ──► Pi
Camera (Pi Cam 3) ► YOLOv8 on Pi ─┘ (bearing + class, timestamped, over UART)      Pi dashboard
```

- **Radar** → Cartesian target positions (linear measurement).
- **Camera** → bearing-only detections (`atan2`, nonlinear) + class label. This is what makes it an EKF.
- **IMU** → yaw used to rotate measurements into a stabilized world frame (rig is handheld/panning).

## Scope: V1 (build now) vs V2 (stretch)

- **V1** — LD2450 + camera + IMU fusion, **IMM** filter for maneuvering targets, full validation & V&V.
  This is what ships. Definition of done = the 7 differentiators in [`docs/requirements.md`](docs/requirements.md).
- **V2 (only after V1 ships)** — swap to a raw-point-cloud radar (TI IWRL6432BOOST) and write detection
  from scratch (CFAR → clustering → association). A bonus, never a prerequisite.

## Repo layout

```
fusion-tracker/
├── docs/          # spec, requirements + V&V matrix, design notes, results
├── firmware/      # STM32CubeIDE project (STM32F446RE) — generated during Phase 1
├── pi_app/        # Raspberry Pi 5: YOLOv8 detection, comms, visualization
├── sim/           # Python EKF/IMM sandbox — synthetic trajectories, NEES/NIS tuning
└── SETUP.md       # dev-environment setup checklist
```

## Status

🚧 Phase 0 — repo scaffold & requirements. See [`docs/fusion-tracker-project-spec.md`](docs/fusion-tracker-project-spec.md)
for the full spec and [`SETUP.md`](SETUP.md) to get the toolchain running.

## Hardware

NUCLEO-F446RE · HLK-LD2450 24 GHz radar · BNO085 IMU · Raspberry Pi 5 8GB + Camera Module 3 ·
SN65HVD230 CAN transceivers. Full BOM in the spec.
