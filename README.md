# StellaRay 🛸

> Autonomous quadcopter platform for surveillance, geo-mapping, and search & rescue operations.

![Status](https://img.shields.io/badge/Status-Active%20Development-brightgreen?style=flat-square)
![Platform](https://img.shields.io/badge/Flight%20Controller-Pixhawk-blue?style=flat-square)
![Firmware](https://img.shields.io/badge/Firmware-ArduPilot-orange?style=flat-square)
![Compute](https://img.shields.io/badge/Onboard%20Compute-Raspberry%20Pi%204-red?style=flat-square)
![Protocol](https://img.shields.io/badge/Telemetry-MAVLink-purple?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-lightgrey?style=flat-square)

---

<!-- Add a banner image or demo GIF here once available -->
<!-- ![StellaRay Demo](assets/banner.gif) -->

## Overview

StellaRay is a solo-developed autonomous quadcopter built for real-world UAV applications including aerial surveillance, geospatial mapping, and rescue operations. The system uses a dual-computer architecture — a **Pixhawk** handles low-level stabilization and flight control via ArduPilot, while a **Raspberry Pi 4** runs high-level autonomy: vision processing, object detection, path planning, and data transmission.

Development started in August 2024. The project targets both defense and commercial UAV use cases and is designed around a modular, extensible architecture.

---

## Hardware Architecture

```
┌─────────────────────────────────────────────────────┐
│                    StellaRay Frame                   │
│                                                      │
│   ┌───────────────┐        ┌─────────────────────┐  │
│   │  Raspberry Pi │◄──────►│ Pixhawk FC          │  │
│   │  4B (4GB)     │ UART/  │ (ArduPilot)         │  │
│   │               │ MAVLink│                     │  │
│   │  - Vision     │        │ - Attitude Control  │  │
│   │  - Planning   │        │ - Motor Mixing      │  │
│   │  - Detection  │        │ - Sensor Fusion     │  │
│   │  - Telemetry  │        │ - GPS Hold / RTL    │  │
│   └───────┬───────┘        └──────────┬──────────┘  │
│           │                           │              │
│       Camera                    ESC + Motors         │
│       (Pi Cam / USB)            (4x Brushless)       │
└─────────────────────────────────────────────────────┘
```

**Component List**

| Component | Details |
|---|---|
| Frame | F450 / custom quadcopter frame |
| Flight Controller | Pixhawk (ArduPilot firmware) |
| Onboard Computer | Raspberry Pi 4B (4GB) |
| Motors | Brushless DC, 920–1000KV |
| ESC | 30A ESCs (PWM) |
| GPS | M8N GPS module |
| Camera | Raspberry Pi Camera / USB camera |
| Communication | MAVLink over UART (RPi ↔ Pixhawk) |
| Ground Station | Mission Planner |
| Power | 3S/4S LiPo + power distribution board |

---

## Software Stack

```
┌──────────────────────────────────────┐
│         Raspberry Pi 4 (High-Level)  │
│                                      │
│  ┌────────────┐   ┌───────────────┐  │
│  │  Vision    │   │  Path Planner │  │
│  │  Pipeline  │   │  (Autonomy)   │  │
│  └─────┬──────┘   └──────┬────────┘  │
│        │                 │           │
│  ┌─────▼─────────────────▼────────┐  │
│  │         MAVLink Layer          │  │
│  │     (pymavlink / dronekit)     │  │
│  └────────────────┬───────────────┘  │
└───────────────────┼──────────────────┘
                    │ UART
┌───────────────────▼──────────────────┐
│        Pixhawk (Low-Level)           │
│        ArduCopter Firmware           │
│  - Rate/Attitude/Position loops      │
│  - Failsafe, RTL, Auto modes         │
└──────────────────────────────────────┘
```

**Software dependencies**

- `pymavlink` / `dronekit-python` — MAVLink communication between RPi and Pixhawk
- `OpenCV` — camera feed processing, frame capture
- `Mission Planner` — ground station for configuration, telemetry, and mission upload
- `ArduCopter` — flight firmware on Pixhawk
- Python 3.x on Raspberry Pi OS (64-bit)

---

## Computer Vision & Object Detection

The Raspberry Pi 4 handles real-time vision tasks onboard:

- **Live camera feed** captured via OpenCV
- **Object detection** using lightweight inference models suitable for edge compute (work in progress)
- Detected targets are geotagged using GPS coordinates from the Pixhawk
- Detection results fed back into the mission logic for autonomous response (hover, mark, alert)

> Vision pipeline is actively being developed. Detection model integration and geolocation tagging are current WIP items.

---

## Mission Planning & Autonomy

StellaRay supports both manual and autonomous flight modes:

- **Waypoint navigation** — missions uploaded via Mission Planner or scripted through MAVLink commands from the RPi
- **GUIDED mode control** — RPi sends dynamic position/velocity targets to Pixhawk over MAVLink for reactive autonomy
- **Loiter / Hold** — stable hover at a fixed position using GPS
- **Return to Launch (RTL)** — automatic failsafe return on signal loss or low battery
- **Auto mode** — pre-programmed mission execution (survey grids, perimeter scan)

Path planning and dynamic re-routing logic are in active development.

---

## Repository Structure

```
StellaRay/
├── hardware/
│   ├── wiring_diagram.png
│   ├── component_list.md
│   └── frame_assembly_notes.md
├── firmware/
│   └── ardupilot_params.param       # Pixhawk parameter file
├── rpi/
│   ├── mavlink_interface.py         # MAVLink comms (RPi ↔ Pixhawk)
│   ├── vision_pipeline.py           # OpenCV capture + processing
│   ├── object_detection.py          # Detection inference (WIP)
│   └── mission_control.py           # Autonomy logic
├── docs/
│   ├── architecture.md
│   ├── setup_guide.md
│   └── flight_logs/
├── assets/
│   └── (images, diagrams, demo media)
└── README.md
```

---

## Getting Started

### Prerequisites

- Raspberry Pi 4 running Raspberry Pi OS (64-bit)
- Pixhawk with ArduCopter firmware flashed
- Python 3.9+ on the RPi
- Mission Planner (Windows) for initial configuration

### Setup

```bash
# On Raspberry Pi
git clone https://github.com/<your-username>/StellaRay.git
cd StellaRay/rpi

pip install pymavlink dronekit opencv-python

# Connect Pixhawk to RPi via UART (or USB for testing)
# Run MAVLink interface
python mavlink_interface.py
```

> Full setup guide in [`docs/setup_guide.md`](docs/setup_guide.md)

---

## Roadmap

- [x] Dual-computer architecture (Pixhawk + RPi)
- [x] MAVLink communication over UART
- [x] Stable manual + assisted flight modes
- [x] Mission Planner integration
- [ ] Real-time object detection (edge-optimized model)
- [ ] GPS-based target geolocation
- [ ] Dynamic path replanning
- [ ] Onboard data logging and telemetry dashboard
- [ ] FPV / live video downlink

---

## About

Built by **Inderpal Singh** — ECE undergrad at Manipal University Jaipur, Senior Coordinator (Electronics) at IEEE RAS MUJ.

Solo project. Started August 2024. Targeting defense and commercial UAV applications.

---

## License

MIT License. See [`LICENSE`](LICENSE) for details.
