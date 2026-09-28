"""Camera pipeline: capture -> YOLOv8 person detection -> bearing -> UART to STM32.

Run on the Pi 5 from the pi_app/ folder:
    cd ~/fusion-tracker/pi_app
    pip install pyserial            # (once, in your venv)
    python3 detect_and_send.py

Detects the most-confident person, projects the bbox center to an azimuth
bearing, and streams a 6-byte frame to the STM32 over the Pi UART. It also PRINTS
the bearing, so you can verify it works even with the STM32 unplugged.

Enable the Pi UART first (once):
    sudo raspi-config  ->  Interface Options -> Serial Port
      "login shell over serial?"  -> No
      "serial port hardware enabled?" -> Yes
    then reboot.  Port = /dev/serial0.
"""
import time
import serial
from picamera2 import Picamera2
from ultralytics import YOLO

from detection.bearing_projection import pixel_to_bearing
from comms.protocol import pack_camera

W, H = 640, 480
PORT = "/dev/serial0"
BAUD = 115200
PERSON = 0

model = YOLO("yolov8n.pt")
picam = Picamera2()
picam.configure(picam.create_preview_configuration(
    main={"format": "RGB888", "size": (W, H)}))
picam.start()
time.sleep(1.0)

ser = serial.Serial(PORT, BAUD, timeout=0)
print(f"Streaming camera bearings to {PORT} @ {BAUD}. Ctrl-C to stop.")

try:
    while True:
        frame = picam.capture_array()
        res = model.predict(frame, classes=[PERSON], conf=0.4, verbose=False)[0]

        best_cx, best_conf = None, 0.0
        for b in res.boxes:
            c = float(b.conf[0])
            if c > best_conf:
                x1, _, x2, _ = b.xyxy[0].tolist()
                best_cx, best_conf = (x1 + x2) / 2.0, c

        if best_cx is not None:
            bearing = pixel_to_bearing(best_cx, W)
            ser.write(pack_camera(bearing, True))
            print(f"person bearing = {bearing:+6.1f} deg   (conf {best_conf:.2f})")
        else:
            ser.write(pack_camera(0.0, False))
            print("no person")
except KeyboardInterrupt:
    ser.close()
    picam.stop()
    print("\nstopped")
