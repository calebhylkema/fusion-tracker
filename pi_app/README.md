# pi_app/ — Raspberry Pi 5 (detection + comms + visualization)

Runs on the Pi 5 (Raspberry Pi OS 64-bit, Bookworm). Set up on the Pi, not the dev PC — `picamera2`
and hardware access are Pi-only.

Planned layout (see the spec):
```
detection/   yolo_detector.py, camera_capture.py, bearing_projection.py
calib/        intrinsics.py, extrinsics.py           # camera + camera→radar calibration
comms/        uart_bridge.py, protocol.py            # link to STM32 (detections up, tracks down)
viz/          dashboard.py, track_overlay.py, radar_plot.py
main.py       # start capture, detection, comms, viz threads
```

Setup: see repo `SETUP.md` §3. Key deps: `ultralytics`, `picamera2`, `pyserial`, `pyqt6`.

First task (Phase 1): capture a frame with `picamera2`, run `yolov8n`, and **record fps + latency** —
those numbers drive the asynchronous-measurement handling in the fusion design.
