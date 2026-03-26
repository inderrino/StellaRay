# StellaRay — Setup Guide

Complete guide to replicating the StellaRay hardware and software stack from scratch.

---

## Table of Contents

1. [Hardware Requirements](#1-hardware-requirements)
2. [Wiring & Assembly](#2-wiring--assembly)
3. [Pixhawk Configuration](#3-pixhawk-configuration)
4. [Raspberry Pi Setup](#4-raspberry-pi-setup)
5. [MAVLink Communication](#5-mavlink-communication)
6. [Camera Setup](#6-camera-setup)
7. [Object Detection Model](#7-object-detection-model)
8. [Running the Stack](#8-running-the-stack)
9. [Mission Planner Integration](#9-mission-planner-integration)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. Hardware Requirements

| Component | Spec | Notes |
|---|---|---|
| Frame | F450 or equivalent | 450mm wheelbase minimum |
| Flight Controller | Pixhawk 2.4.8 / Cube | ArduCopter firmware |
| Onboard Computer | Raspberry Pi 4B (4GB) | 8GB works too |
| Motors | 920–1000KV brushless | 4x |
| ESC | 30A, BLHeli or SimonK | 4x |
| GPS Module | M8N with compass | Mount away from motors |
| Camera | Pi Camera v2 or USB cam | USB recommended for simplicity |
| Battery | 4S 5000mAh LiPo | 3S minimum |
| Power Module | Pixhawk PM02 or equivalent | For voltage/current sensing |
| Telemetry Radio | SiK 915MHz (optional) | For Mission Planner link |
| RC Transmitter | Any MAVLink-compatible | Minimum 6 channels |
| MicroSD | 32GB+ Class 10 | For RPi OS |

---

## 2. Wiring & Assembly

### Motor order (ArduCopter default — X configuration)

```
    FRONT
  4     2
    X
  3     1
    REAR

Motor 1 — Front Right — CW
Motor 2 — Rear Right  — CCW
Motor 3 — Front Left  — CCW
Motor 4 — Rear Left   — CW
```

### ESC to Pixhawk

Connect ESC signal wires to Pixhawk **MAIN OUT** ports:

```
MAIN OUT 1 → Motor 1 ESC
MAIN OUT 2 → Motor 2 ESC
MAIN OUT 3 → Motor 3 ESC
MAIN OUT 4 → Motor 4 ESC
```

### GPS

Connect GPS module to Pixhawk **GPS1** port (6-pin JST-GH).
Mount on a mast at least 10cm above the frame, facing forward.

### Raspberry Pi to Pixhawk (UART)

Connect RPi GPIO UART to Pixhawk **TELEM2** port:

```
RPi GPIO 14 (TX)  →  Pixhawk TELEM2 RX
RPi GPIO 15 (RX)  →  Pixhawk TELEM2 TX
RPi GND           →  Pixhawk TELEM2 GND
```

> Do **not** connect 5V between RPi and Pixhawk. Power them separately.

### Power

- Battery → Power module → Pixhawk **POWER** port
- BEC (5V) or power module output → Raspberry Pi 5V/GND (via GPIO or USB)
- Do not draw RPi power from Pixhawk servo rail

---

## 3. Pixhawk Configuration

### Flash ArduCopter firmware

1. Open **Mission Planner** → Initial Setup → Install Firmware
2. Select **ArduCopter** for your Pixhawk version
3. Follow the flashing wizard

### Essential parameters (set via Mission Planner Full Parameter List)

| Parameter | Value | Reason |
|---|---|---|
| `SERIAL2_PROTOCOL` | 2 | MAVLink 2 on TELEM2 |
| `SERIAL2_BAUD` | 57 | 57600 baud (matches RPi code) |
| `BRD_SER2_RTSCTS` | 0 | Disable flow control |
| `FS_THR_ENABLE` | 1 | Throttle failsafe → RTL |
| `FS_BATT_ENABLE` | 2 | Battery failsafe → Land |
| `FENCE_ENABLE` | 1 | Geofence (recommended) |
| `ARMING_CHECK` | 1 | All arming checks enabled |

### Calibration sequence (do in order)

1. **Accelerometer** — Initial Setup → Mandatory Hardware → Accel Calibration
2. **Compass** — Initial Setup → Mandatory Hardware → Compass
3. **Radio** — Initial Setup → Mandatory Hardware → Radio Calibration
4. **ESC** — Initial Setup → Optional Hardware → ESC Calibration
5. **Flight modes** — Set at minimum: Stabilize, AltHold, Loiter, RTL, Auto, Guided

### Export your parameter file

After tuning, go to **Config → Full Parameter List → Save to file**.
Save as `firmware/ardupilot_params.param` in the repo.

---

## 4. Raspberry Pi Setup

### Flash OS

Download **Raspberry Pi OS Lite 64-bit** (no desktop needed for headless use).
Flash with [Raspberry Pi Imager](https://www.raspberrypi.com/software/).

Before flashing, in Imager settings:
- Enable SSH
- Set hostname: `stellaray`
- Set username/password
- Configure WiFi (for initial setup)

### Enable UART

SSH into the Pi, then:

```bash
sudo raspi-config
```

Navigate to: **Interface Options → Serial Port**
- "Would you like a login shell to be accessible over serial?" → **No**
- "Would you like the serial port hardware to be enabled?" → **Yes**

Reboot:

```bash
sudo reboot
```

Verify UART is available:

```bash
ls /dev/ttyAMA0
```

### Install dependencies

```bash
sudo apt update && sudo apt upgrade -y

# Python and pip
sudo apt install -y python3 python3-pip python3-venv git

# OpenCV system dependencies
sudo apt install -y libopencv-dev python3-opencv

# Create project virtual environment
python3 -m venv ~/stellaray-env
source ~/stellaray-env/bin/activate

# Clone repo
git clone https://github.com/<your-username>/StellaRay.git
cd StellaRay/rpi

# Install Python dependencies
pip install pymavlink dronekit opencv-python tflite-runtime
```

> Use `tflite-runtime` instead of full TensorFlow on RPi — significantly smaller and faster.

### Auto-start on boot (optional)

Create a systemd service to start the mission controller on boot:

```bash
sudo nano /etc/systemd/system/stellaray.service
```

```ini
[Unit]
Description=StellaRay Mission Controller
After=network.target

[Service]
ExecStart=/home/pi/stellaray-env/bin/python /home/pi/StellaRay/rpi/mission_control.py
WorkingDirectory=/home/pi/StellaRay/rpi
Restart=on-failure
User=pi

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable stellaray
sudo systemctl start stellaray
```

---

## 5. MAVLink Communication

Test the MAVLink link before any flight:

```bash
source ~/stellaray-env/bin/activate
cd ~/StellaRay/rpi
python mavlink_interface.py
```

Expected output:

```
Connecting to Pixhawk...
Connected — System: 1, Component: 1
Heartbeat: <MAVLink HEARTBEAT ...>
GPS: {'lat': 28.461200, 'lon': 76.578400, 'alt': 230.1, 'fix_type': 3, ...}
Attitude: {'roll': 0.002, 'pitch': -0.001, 'yaw': 1.57}
Battery: {'voltage_V': 16.4, 'current_A': 0.3, 'remaining_%': 95}
```

If no connection:
- Check UART wires (TX/RX not swapped?)
- Verify `SERIAL2_PROTOCOL=2` and `SERIAL2_BAUD=57` in Pixhawk params
- Try `connection_string="/dev/ttyACM0"` if using USB instead of GPIO UART

---

## 6. Camera Setup

### USB Camera

Plug in and verify:

```bash
ls /dev/video*
# Should show /dev/video0
```

Test capture:

```bash
python -c "import cv2; cap = cv2.VideoCapture(0); print('Opened:', cap.isOpened())"
```

### Raspberry Pi Camera (V4L2)

```bash
sudo raspi-config
# Interface Options → Camera → Enable

# Install V4L2 driver
sudo modprobe bcm2835-v4l2
echo "bcm2835-v4l2" | sudo tee -a /etc/modules
```

Then use `camera_src="/dev/video0"` in VisionPipeline.

---

## 7. Object Detection Model

### Download pretrained TFLite model

```bash
cd ~/StellaRay/rpi
mkdir -p models && cd models

# EfficientDet-Lite0 (recommended)
wget https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
unzip coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip

mv detect.tflite detect.tflite
mv labelmap.txt labelmap.txt
```

Or download EfficientDet-Lite0 from:
[https://tfhub.dev/tensorflow/lite-model/efficientdet/lite0/detection/metadata/1](https://tfhub.dev/tensorflow/lite-model/efficientdet/lite0/detection/metadata/1)

Rename to `detect.tflite` and place in `rpi/models/`.

### Custom model (optional)

If you train a custom model (e.g., for specific target classes):
1. Export from TensorFlow as TFLite with INT8 quantization
2. Replace `models/detect.tflite`
3. Update `models/labelmap.txt` with your class names (one per line)

---

## 8. Running the Stack

### Vision pipeline only (ground test)

```bash
source ~/stellaray-env/bin/activate
cd ~/StellaRay/rpi
python vision_pipeline.py
```

### Full mission (indoor simulation — USB connection)

Edit `mission_control.py` test block:
- Set `connection_string="/dev/ttyACM0"` for USB
- Set `display=False` for headless RPi
- Replace waypoint GPS coords with your actual test location

```bash
python mission_control.py
```

### Safe test sequence before first outdoor flight

1. Props **off** — test MAVLink arm/disarm cycle
2. Props **off** — test takeoff command (motors spin up, verify direction)
3. Outdoor, open area — 2m hover test in GUIDED mode
4. Full mission at low altitude (5m) with short waypoints

---

## 9. Mission Planner Integration

Mission Planner can run alongside the RPi stack for monitoring.

### Telemetry radio link

If using SiK telemetry radios:
- Air module → Pixhawk **TELEM1**
- Ground module → laptop USB
- Mission Planner auto-detects at 57600 baud

### UDP passthrough from RPi (no radio needed)

On RPi, forward MAVLink over WiFi to Mission Planner:

```bash
mavproxy.py --master=/dev/ttyAMA0 --baudrate 57600 --out udp:<laptop-ip>:14550
```

On Mission Planner: Connect → UDP → port 14550.

---

## 10. Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| No MAVLink heartbeat | Wrong port or baud | Check `SERIAL2_BAUD=57`, try USB |
| GPS `fix_type < 3` | Indoors or cold start | Move outside, wait 2–3 min |
| Motors don't arm | Arming check failure | Check Mission Planner arming status |
| RPi can't open camera | Wrong device index | Try `src=1` or check `/dev/video*` |
| TFLite import error | Wrong Python version | Use Python 3.9–3.11, install `tflite-runtime` |
| High inference latency | Too many threads | Set `num_threads=4` (RPi4 max) |
| Drone drifts in GUIDED | Poor compass calibration | Redo compass cal away from metal |
| Velocity command ignored | Not in GUIDED mode | Call `set_mode("GUIDED")` first |

---

## Notes

- Always do maiden flights in Stabilize mode before enabling autonomy
- Keep a safety pilot with RC override ready during any autonomous test
- Log all flights — `logs/` directory stores MAVLink and detection CSVs
- Back up your `.param` file after every tuning session

---

*StellaRay — Built by Inderpal Singh, Manipal University Jaipur*
