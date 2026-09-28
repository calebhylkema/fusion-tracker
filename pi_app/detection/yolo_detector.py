"""YOLOv8-nano person detection on the Pi Camera Module 3.

Run on the Raspberry Pi 5 (Raspberry Pi OS Bookworm). Captures frames, detects
people, and prints each person's bounding-box center + confidence along with the
inference rate (fps) and latency. This is the first camera milestone: prove the
capture -> detect pipeline and measure how fast YOLOv8n runs on the Pi 5 CPU.

Setup (on the Pi):
    sudo apt update && sudo apt install -y python3-picamera2
    python3 -m venv --system-site-packages ~/ft
    source ~/ft/bin/activate
    pip install ultralytics
    python3 yolo_detector.py

Ctrl-C to stop.
"""
import time
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO

W, H = 640, 480          # capture size (smaller = faster inference)
CONF = 0.40              # detection confidence threshold
PERSON = 0               # COCO class id for "person"

model = YOLO("yolov8n.pt")   # auto-downloads on first run

picam = Picamera2()
picam.configure(picam.create_preview_configuration(
    main={"format": "RGB888", "size": (W, H)}))
picam.start()
time.sleep(1.0)          # let auto-exposure settle

print(f"Camera {W}x{H} + YOLOv8n ready. Ctrl-C to stop.")
window_t = time.time()
frames = 0
lat_sum = 0.0
try:
    while True:
        frame = picam.capture_array()                # HxWx3 RGB
        t0 = time.time()
        res = model.predict(frame, classes=[PERSON], conf=CONF, verbose=False)[0]
        lat_sum += time.time() - t0

        dets = []
        for b in res.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            dets.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0, float(b.conf[0])))

        frames += 1
        if time.time() - window_t >= 1.0:            # report once per second
            fps = frames / (time.time() - window_t)
            lat_ms = 1000.0 * lat_sum / frames
            people = "  ".join(f"[cx={cx:4.0f} cy={cy:4.0f} conf={c:.2f}]"
                               for cx, cy, c in dets)
            print(f"{fps:4.1f} fps | {lat_ms:5.1f} ms | {len(dets)} person(s)  {people}")
            window_t = time.time()
            frames = 0
            lat_sum = 0.0
except KeyboardInterrupt:
    picam.stop()
    print("\nstopped")
