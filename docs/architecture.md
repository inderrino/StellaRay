# StellaRay — System Architecture

Technical breakdown of the StellaRay autonomous quadcopter platform. Covers hardware architecture, software stack, data flow, communication protocols, and design decisions.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Dual-Computer Architecture](#2-dual-computer-architecture)
3. [Hardware Layer](#3-hardware-layer)
4. [Software Stack](#4-software-stack)
5. [Communication Architecture](#5-communication-architecture)
6. [Vision & Detection Pipeline](#6-vision--detection-pipeline)
7. [Mission Execution Flow](#7-mission-execution-flow)
8. [Data Flow Diagram](#8-data-flow-diagram)
9. [Autonomy Design](#9-autonomy-design)
10. [Design Decisions & Trade-offs](#10-design-decisions--trade-offs)

---

## 1. System Overview

StellaRay is a dual-computer autonomous quadcopter. The system is split across two processors with distinct responsibilities — a **Pixhawk** flight controller handles all real-time flight-critical tasks, while a **Raspberry Pi 4** runs the high-level intelligence layer: vision, detection, planning, and telemetry.

This separation is intentional. Flight stability is deterministic and runs on dedicated hardware with ArduPilot firmware. The compute-heavy AI stack runs on Linux without affecting flight loop timing.

```
┌─────────────────────────────────────────────────────────────────┐
│                        StellaRay System                         │
│                                                                 │
│   ┌─────────────────────┐          ┌────────────────────────┐   │
│   │   Raspberry Pi 4    │◄────────►│   Pixhawk FC           │   │
│   │   (High-Level AI)   │  MAVLink │   (ArduPilot Firmware) │   │
│   │                     │  UART    │                        │   │
│   │  Vision Pipeline    │          │  Attitude Control      │   │
│   │  Object Detection   │          │  Motor Mixing          │   │
│   │  Mission Control    │          │  Sensor Fusion         │   │
│   │  Path Planning      │          │  GPS Hold / RTL        │   │
│   │  Data Logging       │          │  Failsafes             │   │
│   └─────────┬───────────┘          └──────────┬─────────────┘   │
│             │                                 │                 │
│           Camera                        ESCs + Motors           │
│           GPS overlay                   IMU / Baro / Compass    │
│           WiFi / SSH                    GPS Module              │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Dual-Computer Architecture

### Why two computers?

A single-board computer like the Raspberry Pi running Linux is **not real-time**. The OS scheduler, memory management, and I/O can introduce millisecond-level jitter — unacceptable for a flight controller that needs to run stabilization loops at 400Hz.

ArduPilot on Pixhawk runs a **bare-metal RTOS** (ChibiOS) with hard real-time guarantees. It handles everything that must not miss a deadline: motor outputs, IMU reads, attitude estimation, PID loops.

The RPi handles everything that benefits from a full OS: OpenCV, TensorFlow Lite, Python scripting, file I/O, networking.

| Responsibility | Pixhawk (ArduPilot) | Raspberry Pi 4 |
|---|---|---|
| Motor PWM output | ✅ | ❌ |
| IMU / sensor fusion | ✅ | ❌ |
| Attitude / rate PID | ✅ | ❌ |
| GPS navigation | ✅ | ❌ |
| Failsafe / RTL | ✅ | ❌ |
| Camera capture | ❌ | ✅ |
| Object detection | ❌ | ✅ |
| Path planning | ❌ | ✅ |
| Mission logic | ❌ | ✅ |
| Data logging | ❌ | ✅ |
| Telemetry to GCS | Passthrough | ✅ |

---

## 3. Hardware Layer

### Flight stack

```
Battery (4S LiPo)
    │
    ├──► Power Module ──► Pixhawk (POWER port)
    │         │
    │         └──► Voltage/Current sensing → Pixhawk ADC
    │
    └──► BEC (5V) ──► Raspberry Pi 4 (5V GPIO or USB)

Pixhawk MAIN OUT
    ├── OUT1 → ESC1 → Motor 1 (Front Right, CW)
    ├── OUT2 → ESC2 → Motor 2 (Rear Right, CCW)
    ├── OUT3 → ESC3 → Motor 3 (Front Left, CCW)
    └── OUT4 → ESC4 → Motor 4 (Rear Left, CW)

Pixhawk peripherals
    ├── GPS1     → M8N GPS + Compass
    ├── TELEM1   → SiK Telemetry Radio (optional GCS link)
    ├── TELEM2   → Raspberry Pi UART (MAVLink)
    └── RC IN    → RC Receiver (manual override)
```

### Compute stack

```
Raspberry Pi 4B
    ├── GPIO 14/15 (UART TX/RX) → Pixhawk TELEM2
    ├── CSI Port → Pi Camera v2 (optional)
    ├── USB      → USB Camera / Pixhawk (SITL/debug)
    └── WiFi     → Ground station SSH / telemetry forwarding
```

### Sensor inputs

| Sensor | Interface | Data | Used by |
|---|---|---|---|
| IMU (MPU-6000) | SPI | Accel, Gyro @ 1kHz | Pixhawk |
| Barometer (MS5611) | I2C | Altitude | Pixhawk |
| Compass (HMC5883) | I2C | Heading | Pixhawk |
| GPS (M8N, u-blox) | UART | Lat/Lon/Alt @ 10Hz | Pixhawk → RPi via MAVLink |
| Camera | USB/CSI | RGB frames @ 30fps | Raspberry Pi |

---

## 4. Software Stack

### Pixhawk — ArduCopter

ArduPilot runs a cascaded PID control loop:

```
Desired position/velocity (from GCS or RPi MAVLink commands)
        │
        ▼
  Position Controller  (outer loop, ~50Hz)
        │
        ▼
  Velocity Controller
        │
        ▼
  Attitude Controller  (mid loop, ~100Hz)
        │
        ▼
  Rate Controller      (inner loop, ~400Hz)
        │
        ▼
  Motor Mixing → ESC PWM outputs
```

Flight modes used in StellaRay:

| Mode | Trigger | Description |
|---|---|---|
| `STABILIZE` | RC manual | Manual with attitude stabilization |
| `ALTHOLD` | RC manual | Altitude hold, manual lateral |
| `LOITER` | RC / MAVLink | GPS position + altitude hold |
| `GUIDED` | MAVLink only | Full RPi control via position/velocity commands |
| `AUTO` | MAVLink / GCS | Pre-uploaded waypoint mission |
| `RTL` | Failsafe / code | Return to launch and land |
| `LAND` | Code | Descend and disarm at current position |

### Raspberry Pi — Python stack

```
mission_control.py          ← top-level orchestrator
    │
    ├── mavlink_interface.py    ← MAVLink comms (pymavlink)
    │       └── Pixhawk UART/USB
    │
    ├── vision_pipeline.py      ← OpenCV camera pipeline (threaded)
    │       └── CameraStream, MotionDetector, FrameSaver, Annotator
    │
    └── object_detection.py     ← TFLite inference + NMS + CSV logging
            └── models/detect.tflite
```

**Key Python dependencies**

| Package | Purpose |
|---|---|
| `pymavlink` | MAVLink message encode/decode |
| `dronekit` | Higher-level MAVLink abstraction (optional) |
| `opencv-python` | Camera capture, frame processing, display |
| `tflite-runtime` | Lightweight TFLite inference on RPi |
| `numpy` | Array ops, NMS math |
| `threading` | Concurrent camera + mission execution |

---

## 5. Communication Architecture

### MAVLink (RPi ↔ Pixhawk)

MAVLink is the protocol used to send commands from the RPi to the Pixhawk and receive telemetry back.

```
Raspberry Pi                          Pixhawk
    │                                     │
    │  SET_MODE (GUIDED)                  │
    │ ──────────────────────────────────► │
    │                                     │
    │  MAV_CMD_NAV_TAKEOFF                │
    │ ──────────────────────────────────► │
    │                                     │
    │  SET_POSITION_TARGET_GLOBAL_INT     │
    │  (waypoint coordinates)             │
    │ ──────────────────────────────────► │
    │                                     │
    │       HEARTBEAT (1Hz)               │
    │ ◄────────────────────────────────── │
    │                                     │
    │       GPS_RAW_INT (10Hz)            │
    │ ◄────────────────────────────────── │
    │                                     │
    │       ATTITUDE (50Hz)               │
    │ ◄────────────────────────────────── │
    │                                     │
    │       SYS_STATUS (battery)          │
    │ ◄────────────────────────────────── │
```

- **Physical layer:** UART at 57600 baud (RPi GPIO 14/15 → Pixhawk TELEM2)
- **Protocol:** MAVLink v2
- **Library:** `pymavlink` on the RPi side

### Ground Control Station (optional)

Mission Planner connects to Pixhawk either via:
- **SiK telemetry radio** (TELEM1 on Pixhawk, USB on laptop)
- **MAVProxy UDP bridge** from RPi over WiFi (no radio needed)

```
Pixhawk TELEM1 → SiK Air Module
                        │ 915MHz RF
                 SiK Ground Module → USB → Laptop (Mission Planner)

OR

Pixhawk TELEM2 → RPi UART → mavproxy → UDP → WiFi → Mission Planner
```

---

## 6. Vision & Detection Pipeline

The vision stack runs in a **dedicated background thread** inside `VisionPipeline`. This ensures camera I/O and inference never block the mission control loop.

```
CameraStream (thread)
    │  30fps BGR frames
    ▼
preprocess()
    │  resize → denoise
    ▼
MotionDetector
    │  frame diff → contour filter
    ▼
ObjectDetector.detect()         ← TFLite inference
    │  resize to model input (320x320 or 512x512)
    │  invoke interpreter
    │  parse output tensors
    │  NMS per class
    ▼
annotate_frame()                ← HUD overlay
    │  timestamp, FPS, GPS, detection labels
    ▼
FrameSaver                      ← save on detection event
    │  GPS-tagged filename
    ▼
DetectionLogger                 ← CSV log
    │  timestamp, label, confidence, bbox, lat, lon, alt
    ▼
detection_fn callback           ← result passed to MissionController
```

### Inference performance (Raspberry Pi 4, 4 threads)

| Model | Input Size | Quantization | Approx. Latency |
|---|---|---|---|
| MobileNet SSD v2 | 300×300 | INT8 | ~90–120ms |
| EfficientDet-Lite0 | 320×320 | INT8 | ~180–220ms |
| EfficientDet-Lite2 | 448×448 | INT8 | ~380–420ms |

> All latencies measured on RPi4 using `tflite-runtime` with `num_threads=4`.

---

## 7. Mission Execution Flow

```
START
  │
  ▼
PreflightChecker
  ├── GPS fix type ≥ 3?
  ├── Battery ≥ 20%?
  ├── Heartbeat alive?
  └── Drone disarmed?
        │
    PASS│         FAIL──► ABORT
        ▼
  set_mode(GUIDED)
  arm()
  takeoff(alt)
  wait(8s)
        │
        ▼
  Mission Mode?
  ┌─────┼──────┐
SURVEY  SEARCH  PATROL
  │       │       │
  └───────┴───────┘
          │
    _goto_waypoint()
      │
      ├── Send SET_POSITION_TARGET_GLOBAL_INT
      ├── Poll GPS (Haversine distance)
      └── Reached? → hold_time → next waypoint
          │
          ▼
    Check detections (from vision thread)
    Log if found
          │
    Abort flag? ──► RTL
          │
    Mission complete?
          │
          ▼
      land() / RTL
          │
          ▼
        IDLE
```

---

## 8. Data Flow Diagram

```
┌──────────────┐     frames      ┌──────────────────┐
│   Camera     │────────────────►│  VisionPipeline  │
└──────────────┘                 │  (background     │
                                 │   thread)        │
┌──────────────┐   MAVLink msgs  │                  │
│   Pixhawk    │◄───────────────►│  MAVLinkInterface│
│  (ArduPilot) │                 │                  │
└──────────────┘                 │  ObjectDetector  │
                                 └────────┬─────────┘
                                          │detections
                                 ┌────────▼─────────┐
                                 │ MissionController │
                                 │                  │
                                 │  State Machine   │
                                 │  Waypoint Nav    │
                                 │  Abort Handling  │
                                 └────────┬─────────┘
                                          │
                        ┌─────────────────┼──────────────────┐
                        ▼                 ▼                  ▼
                   logs/*.csv      captures/*.jpg    terminal / SSH
               (detection log)   (GPS-tagged frames)  (status output)
```

---

## 9. Autonomy Design

### Levels of autonomy in StellaRay

| Level | Description | Implementation |
|---|---|---|
| 0 | Manual RC flight | Pixhawk Stabilize/AltHold |
| 1 | Position hold | Pixhawk Loiter mode |
| 2 | Waypoint navigation | RPi → MAVLink → Pixhawk AUTO/GUIDED |
| 3 | Reactive autonomy | RPi detection results influence mission path |
| 4 | Full autonomy (WIP) | Dynamic replanning based on detections |

Current StellaRay operates at **Level 2–3**. Level 4 (dynamic replanning) is on the roadmap.

### Failsafe hierarchy

```
1. RC signal loss      → RTL (ArduPilot FS_THR_ENABLE)
2. Battery critical    → Land immediately (FS_BATT_ENABLE)
3. GPS loss in AUTO    → AltHold / LAND depending on config
4. RPi abort flag      → RTL via MAVLink command
5. KeyboardInterrupt   → RTL via MAVLink command
6. Unhandled exception → RTL via MAVLink command
```

All failsafes are layered — Pixhawk-level hardware failsafes operate independently of RPi software. Even if the RPi crashes, the Pixhawk will trigger RTL on RC loss or battery fault.

---

## 10. Design Decisions & Trade-offs

### MAVLink over UART vs USB

UART (GPIO 14/15) was chosen over USB for the RPi↔Pixhawk link. USB introduces OS-level latency from the USB stack; UART is direct and lower latency. The trade-off is a fixed 57600 baud rate ceiling, which is sufficient for MAVLink at typical telemetry rates.

### tflite-runtime vs full TensorFlow

Full TensorFlow on RPi4 is ~800MB installed and has a long import time. `tflite-runtime` is ~25MB, imports in under a second, and supports the same INT8 quantized models. For edge inference on RPi4, there is no practical reason to use full TF.

### Threaded vision vs blocking

Running the vision pipeline in a background thread means the mission control loop never stalls on camera I/O or inference. The trade-off is that detections are always one frame behind — acceptable for the latency requirements of waypoint navigation.

### Python vs C++ on RPi

Python was chosen for rapid development and integration with pymavlink and OpenCV. The latency trade-off is acceptable at this stage. If inference latency becomes a bottleneck, the detection module can be rewritten in C++ using the TFLite C API without changing the rest of the stack.

---

*StellaRay — Built by Inderpal Singh, Manipal University Jaipur*
