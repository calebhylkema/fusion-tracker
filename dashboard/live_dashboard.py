"""Live hardware dashboard for the fusion tracker.

Reads the STM32 telemetry (`TEL,...` lines) from the ST-LINK Virtual COM Port and
draws a real-time bird's-eye view: radar targets, the fused track (position +
velocity arrow + trail), the camera-bearing ray, and the IMU heading — plus a
numeric side panel. This is the "see it all" view, on real hardware data.

Usage (on the laptop, from the repo root):
    .\.venv\Scripts\python.exe dashboard\live_dashboard.py COM5
Find your COM port in Device Manager -> Ports (the "STLink Virtual COM Port"),
and CLOSE the CubeIDE serial console first so the port is free.
"""
import sys
import math
import collections
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=0)
except Exception as e:
    print(f"Could not open {PORT}: {e}\n"
          f"Pass your COM port (e.g. COM5) and close the CubeIDE serial console.")
    sys.exit(1)

fig = plt.figure(figsize=(11, 7))
ax = fig.add_axes([0.06, 0.08, 0.60, 0.86])
tp = fig.add_axes([0.70, 0.08, 0.28, 0.86]); tp.axis("off")
ax.set_xlim(-4, 4); ax.set_ylim(-0.5, 7); ax.set_aspect("equal"); ax.grid(alpha=0.3)
ax.set_xlabel("x  cross-range (m)"); ax.set_ylabel("y  range (m)")
ax.set_title("Fusion tracker — live hardware")

radar_dots, = ax.plot([], [], "o", color="0.6", ms=9, label="radar targets")
fused_dot, = ax.plot([], [], "ro", ms=11, label="fused track")
fused_trail, = ax.plot([], [], "r-", lw=1, alpha=0.5)
cam_ray, = ax.plot([], [], "b--", lw=1.5, label="camera bearing")
boresight, = ax.plot([], [], "g-", lw=2.5, alpha=0.5, label="sensor heading (yaw)")
ax.plot(0, 0, "g^", ms=16)
vel = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                  arrowprops=dict(arrowstyle="-|>", color="r", lw=2))
txt = tp.text(0.0, 1.0, "", va="top", family="monospace", fontsize=11)
ax.legend(loc="upper left", fontsize=8)

trail = collections.deque(maxlen=40)


def update(_):
    latest = None
    while ser.in_waiting:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if line.startswith("TEL"):
            latest = line
    if latest is None:
        return
    p = latest.split(",")
    if len(p) < 18:
        return
    try:
        v = [int(x) for x in p[1:18]]
    except ValueError:
        return
    cam_bytes = p[18] if len(p) >= 19 else "n/a"   # optional diagnostic field

    rx, ry = [], []
    for t in range(3):
        x, y, ok = v[t * 3], v[t * 3 + 1], v[t * 3 + 2]
        if ok:
            rx.append(x / 1000.0); ry.append(y / 1000.0)
    radar_dots.set_data(rx, ry)

    yaw, cam, camok = v[9], v[10], v[11]
    fpx, fpy = v[12] / 100.0, v[13] / 100.0
    fvx, fvy, fok = v[14] / 100.0, v[15] / 100.0, v[16]

    if fok:
        fused_dot.set_data([fpx], [fpy])
        trail.append((fpx, fpy))
        tr = np.array(trail); fused_trail.set_data(tr[:, 0], tr[:, 1])
        vel.set_position((fpx, fpy)); vel.xy = (fpx + fvx, fpy + fvy)
    else:
        fused_dot.set_data([], []); fused_trail.set_data([], [])
        vel.set_position((0, 0)); vel.xy = (0, 0); trail.clear()

    if camok:
        cr = math.radians(cam)
        cam_ray.set_data([0, 7.0 * math.sin(cr)], [0, 7.0 * math.cos(cr)])
    else:
        cam_ray.set_data([], [])

    yr = math.radians(yaw)
    boresight.set_data([0, -1.5 * math.sin(yr)], [0, 1.5 * math.cos(yr)])

    lines = ["RADAR targets (mm):"]
    if rx:
        for t in range(3):
            x, y, ok = v[t * 3], v[t * 3 + 1], v[t * 3 + 2]
            if ok:
                lines.append(f"  T{t+1}: x={x:+5d}  y={y:5d}")
    else:
        lines.append("  (none)")
    lines += ["", f"IMU  yaw : {yaw:+d} deg",
              f"CAMERA   : {cam:+d} deg {'PERSON' if camok else '(none)'}",
              f"cam bytes: {cam_bytes}",
              "", "FUSED track:"]
    if fok:
        lines += [f"  pos = ({fpx:+.2f}, {fpy:+.2f}) m",
                  f"  vel = ({fvx:+.2f}, {fvy:+.2f}) m/s",
                  f"  speed = {math.hypot(fvx, fvy):.2f} m/s"]
    else:
        lines += ["  (no track)"]
    txt.set_text("\n".join(lines))


ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()
