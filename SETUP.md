# Dev Environment Setup

Checklist to get from zero to "I can build firmware and run the sim." Tick items as you go.

## Already installed on this machine ✅
- **git** 2.55
- **Python** 3.11.0

## 1. STM32 firmware toolchain

- [ ] **Install STM32CubeIDE** (bundles arm-none-eabi-gcc, OpenOCD/ST-LINK GDB server, and CubeMX):
      https://www.st.com/en/development-tools/stm32cubeide.html (free ST account required).
- [ ] Install / update **ST-LINK drivers** (usually handled by the CubeIDE installer).
- [ ] Plug in the NUCLEO-F446RE over USB; confirm it enumerates as an **ST-LINK** device and a
      **Virtual COM Port** (Device Manager → Ports). Note the COM port number for debug output.
- [ ] In CubeIDE: create a new **STM32 project → board = NUCLEO-F446RE**, let CubeMX initialize, build
      the empty project, and flash it (Run) to confirm the full build+flash+debug chain works.
      → The generated project lives in `firmware/`.

> We chose **CubeIDE + HAL/LL, timer-driven superloop (no RTOS)**. arm-gcc from CubeIDE is enough — no
> separate GNU Arm toolchain install needed.

## 2. Python sim environment (runs on this Windows PC — for the EKF/IMM sandbox)

```powershell
cd C:\Users\caleb\Documents\Career\Projects\fusion-tracker
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r sim\requirements.txt
```
- [ ] venv created and activated
- [ ] `numpy`, `scipy`, `matplotlib` installed (for synthetic trajectories + NEES/NIS plots)

## 3. Raspberry Pi 5 (detection + dashboard — set up on the Pi, not this PC)

- [ ] Flash **Raspberry Pi OS (64-bit, Bookworm)** to the microSD with Raspberry Pi Imager;
      set hostname, enable SSH + a user in the imager.
- [ ] Boot, `sudo apt update && sudo apt full-upgrade`, connect the Camera Module 3 to the CSI port.
- [ ] Verify camera: `rpicam-hello` (preview) and `rpicam-still -o test.jpg`.
- [ ] Python env on the Pi: `python3 -m venv --system-site-packages ~/ft && source ~/ft/bin/activate`
      then `pip install ultralytics picamera2 pyserial pyqt6` (picamera2 is easiest via system packages).
- [ ] Benchmark YOLOv8-nano: download `yolov8n.pt`, time inference on a frame — **record fps + latency**
      (this feeds the async-measurement design). Consider an `ncnn` export if too slow.
- [ ] Enable the Pi UART (`raspi-config` → Interface → Serial: login shell **off**, hardware **on**)
      for the STM32 link on GPIO14/15.

## 4. Hardware bringup order (do NOT wire everything at once)

1. NUCLEO alone → blink + printf over VCP.
2. Radar on USART1 (PB6/PB7) → parse and print target x/y/speed.
3. IMU on I2C1 → print yaw (try I2C, fall back to SPI if flaky).
4. CAN loopback with the two SN65HVD230 transceivers + 120 Ω termination.
5. Pi ↔ STM32 UART link (USART6).
6. Integrate.

## 5. Git

- [x] `git init` (done during scaffold)
- [ ] Create a private/public GitHub repo and `git remote add origin ...` when ready to push.

---
See `docs/fusion-tracker-project-spec.md` for the full spec, pinout, and CAN message definitions.
