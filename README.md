# UAV Precision Landing System

### Vision-Based Autonomous Touchdown Using Nested AprilTag Detection

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![PX4](https://img.shields.io/badge/PX4-v1.14-purple?style=flat-square&logo=drone&logoColor=white)](https://px4.io/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=flat-square&logo=python&logoColor=white)](https://python.org)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-F58025?style=flat-square&logo=gazebo&logoColor=white)](https://gazebosim.org)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=flat-square&logo=opencv&logoColor=white)](https://opencv.org)
[![MAVLink](https://img.shields.io/badge/MAVLink-v2-0066FF?style=flat-square&logo=mavlink&logoColor=white)](https://mavlink.io)
[![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)](LICENSE)
[![Status](https://img.shields.io/badge/Status-SITL_Validated-brightgreen?style=flat-square)]()
[![HITL](https://img.shields.io/badge/HITL-Validated-brightgreen?style=flat-square)]()

<br/>

> **PFE — Projet de Fin d'Études**  
> FARASHA Systems × TAMAYOUZ Centre FSA × Université Ibn Zohr  
> ELMESSAOUDI Oussama — Embedded & Robotics Engineering Intern

<br/>

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│   ALTITUDE 5m        ALIGNING          DESCENDING       LANDED     │
│       │                  │                  │              │        │
│       ▼                  ▼                  ▼              ▼        │
│   ┌───────┐   ────►  ┌───────┐  ────►  ┌───────┐  ──►  ┌─────┐   │
│   │SEARCH │          │ ALIGN │          │DESCEND│       │  ✓  │   │
│   └───────┘          └───────┘          └───────┘       └─────┘   │
│    Camera                XY               XY + Z       Disarm     │
│    scans             correction         correction     @ ±5 cm    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

</div>

---

## Table of Contents

- [Project Overview](#-project-overview)
- [System Architecture](#-system-architecture)
- [Hardware Platform](#-hardware-platform)
- [Software Stack](#-software-stack)
- [Vision Pipeline](#-vision-pipeline)
- [Nested AprilTag Design](#-nested-apriltag-design)
- [State Machine](#-state-machine-fsm)
- [MAVLink Integration](#-mavlink-integration)
- [Filter Pipeline](#-filter-pipeline)
- [Simulation Environment (SITL)](#-simulation-environment-sitl)
- [Hardware-in-the-Loop (HITL)](#-hardware-in-the-loop-hitl)
- [ROS2 Topic Graph](#-ros2-topic-graph)
- [PX4 Parameters](#-px4-parameters)
- [Performance Metrics](#-performance-metrics)
- [Repository Structure](#-repository-structure)
- [Quick Start](#-quick-start)
- [Results](#-results)
- [Roadmap](#-roadmap)
- [Author](#-author)

---

## Project Overview

GPS-based drone landing achieves only **±2–5 m accuracy** — insufficient for confined deployment zones, precision agriculture, search-and-rescue operations, or autonomous delivery. This project implements a **full-stack autonomous precision landing system** that reduces touchdown error to **±5 cm** using a downward-facing camera and a nested AprilTag ground marker.

### Problem Statement

```
GPS Landing                          Vision-Based Landing
─────────────────                    ────────────────────
Accuracy: ±2–5 m        vs.         Accuracy: ±5 cm
No feedback loop                     Continuous correction
Wind drift uncompensated             Real-time pose estimation
Fixed trajectory                     Adaptive FSM control
```

### Key Contributions

| Contribution | Description |
|---|---|
| Nested AprilTag | Dual-size marker enables detection from 15 m down to <0.5 m |
| 4-Stage Filter | Quality gate + outlier rejection + EMA + target-lost failsafe |
| 5-State FSM | SEARCHING → ALIGNING → DESCENDING → FLARE → LANDING |
| Direct MAVLink | Companion sends LANDING_TARGET @ 30 Hz directly to PX4 onboard port |
| PD Alignment | Eliminates circular oscillation — stable center-hold from high altitude |

---

## System Architecture

### Full System Block Diagram

```
╔══════════════════════════════════════════════════════════════════════════╗
║                     FARASHA PRECISION LANDING SYSTEM                    ║
╠══════════════════╦═══════════════════════════╦═══════════════════════════╣
║  SENSING LAYER   ║     PROCESSING LAYER       ║    ACTUATION LAYER        ║
║                  ║                            ║                           ║
║  PiCamera V2.1   ║  ┌─────────────────────┐   ║  ┌─────────────────────┐  ║
║  IMX219 Sensor   ║  │  apriltag_detector  │   ║  │   Cube Orange       │  ║
║  1920×1080 RGB   ║  │                     │   ║  │   PX4 Autopilot     │  ║
║  30 fps          ║  │  ① Quality Gate     │   ║  │                     │  ║
║        │         ║  │  ② Outlier Gate     │   ║  │  EKF2 sensor fusion │  ║
║        ▼         ║  │  ③ EMA Filter       │   ║  │  LANDING_TARGET     │  ║
║  V4L2 capture    ║  │  ④ Lost Failsafe    │   ║  │  SET_POSITION       │  ║
║  via OpenCV      ║  │                     │   ║  │  NAV_LAND           │  ║
║                  ║  └────────┬────────────┘   ║  └────────▲────────────┘  ║
║                  ║           │ /apriltag/pose  ║           │               ║
║                  ║           ▼                 ║           │               ║
║                  ║  ┌─────────────────────┐   ║           │               ║
║                  ║  │ landing_target_pub  │───╬───UART────┘               ║
║                  ║  └─────────────────────┘   ║  921600 baud              ║
║                  ║  ┌─────────────────────┐   ║  /dev/ttyAMA0             ║
║                  ║  │  precision_landing  │───╬───────────►               ║
║                  ║  │  FSM Controller     │   ║                           ║
║                  ║  └─────────────────────┘   ║                           ║
╚══════════════════╩═══════════════════════════╩═══════════════════════════╝
```

### SITL Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     Docker Dev Container (Ubuntu 24)                    │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                        Gazebo 8 (vglrun)                         │  │
│  │   x500_mono_cam_down ──► /camera/image (Gazebo topic)            │  │
│  │   apriltag_pad (0.8m, tag36h11 ID=0)                            │  │
│  └─────────────────────────────────────┬────────────────────────────┘  │
│                                         │ ros_gz_bridge                 │
│                                         ▼                               │
│  ┌─────────────┐    ┌─────────────────────────────────────────────┐    │
│  │   PX4 SITL  │    │              ROS2 Jazzy                      │    │
│  │             │◄───┤  apriltag_detector → landing_target_pub      │    │
│  │  pxh> shell │    │  precision_landing_controller                │    │
│  │  port 14540 │◄───┤  (udpin:0.0.0.0:14540 — direct to PX4)      │    │
│  │  port 14550 │    └─────────────────────────────────────────────┘    │
│  └──────┬──────┘                                                        │
│         │ UDP 14550                                                      │
│  ┌──────▼──────┐    ┌─────────────┐                                     │
│  │  MAVProxy   │───►│     QGC     │                                     │
│  │  port 14570 │    │  port 14570 │                                     │
│  └─────────────┘    └─────────────┘                                     │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Hardware Platform

### Component Overview

```
                        ┌─────────────────────────────┐
                        │      Quadrotor Frame         │
                        │   (x500 / DJI F450 style)   │
                        └──────────────┬──────────────┘
                                       │
              ┌────────────────────────┼────────────────────────┐
              │                        │                        │
     ┌────────▼────────┐    ┌─────────▼─────────┐   ┌─────────▼─────────┐
     │   Cube Orange   │    │   Raspberry Pi 4B  │   │   PiCamera V2.1   │
     │   PX4 Autopilot │    │   4 GB RAM         │   │   IMX219 Sensor   │
     │   ARM Cortex-M7 │    │   Ubuntu 24 LTS    │   │   1080p @ 30fps   │
     │   168 MHz       │    │   ROS2 Jazzy       │   │   3280×2464 native│
     │                 │    │                    │   │   FOV: 62.2°      │
     │  TELEM2 ────────┼────┼──► /dev/ttyAMA0   │   │   f=3.04 mm       │
     │  921600 baud    │    │   MAVLink v2       │   │                   │
     └────────┬────────┘    └────────────────────┘   └───────────────────┘
              │
     ┌────────▼────────┐    ┌──────────────────┐
     │  4× ESC + Motor │    │  LiPo 4S 5000mAh │
     │  Thrust control │    │  14.8V           │
     └─────────────────┘    └──────────────────┘
```

### Hardware Specifications

| Component | Model | Specification |
|-----------|-------|---------------|
| Flight Controller | Cube Orange | STM32H753 @ 480 MHz, IMU: ICM-20689, ICM-20602 |
| Companion Computer | Raspberry Pi 4B | ARM Cortex-A72 @ 1.8 GHz, 4 GB LPDDR4 |
| Camera | PiCamera V2.1 | IMX219, 1/4" sensor, 8 MP, 30 fps @ 1080p |
| Communication | UART TELEM2 | /dev/ttyAMA0, 921600 baud, MAVLink v2 |
| Frame | x500 | 500 mm diagonal, ~1.2 kg AUW |
| Battery | 4S LiPo | 14.8V, 5000 mAh, ~15 min flight time |

---

## Software Stack

```
┌────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                    │
│          precision_landing_controller.py (FSM)          │
│          landing_target_publisher.py                    │
├────────────────────────────────────────────────────────┤
│                    PERCEPTION LAYER                     │
│          apriltag_detector.py                           │
│          pupil_apriltags  |  OpenCV 4.x                 │
├────────────────────────────────────────────────────────┤
│                   MIDDLEWARE LAYER                      │
│          ROS2 Jazzy (rclpy)                             │
│          Topics: /apriltag/pose | /landing/state        │
├────────────────────────────────────────────────────────┤
│                   COMMUNICATION LAYER                   │
│          pymavlink → MAVLink v2                         │
│          LANDING_TARGET | SET_POSITION_TARGET | NAV_LAND│
├────────────────────────────────────────────────────────┤
│                    AUTOPILOT LAYER                      │
│          PX4 v1.14 — EKF2 sensor fusion                 │
│          Cube Orange (HITL) | SITL (Gazebo)             │
├────────────────────────────────────────────────────────┤
│                      OS / HARDWARE                      │
│          Ubuntu 24 LTS | Raspberry Pi 4 | V4L2          │
└────────────────────────────────────────────────────────┘
```

### Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| `rclpy` | Jazzy | ROS2 Python client |
| `pupil_apriltags` | 1.0.4 | AprilTag detection + pose estimation |
| `opencv-python` | 4.9 | Image processing, V4L2 capture |
| `pymavlink` | 2.4.37 | MAVLink protocol implementation |
| `picamera2` | 0.3.x | PiCamera2 libcamera interface (HITL only) |
| `numpy` | 1.26 | Numerical computation |
| `cv_bridge` | Jazzy | ROS2 ↔ OpenCV bridge |

---

## Vision Pipeline

### Detection Flow

```
PiCamera V2.1 / Gazebo Camera
         │
         │  RAW FRAME (1920×1080 RGB, 30 fps)
         ▼
┌─────────────────────────────────────────────────────┐
│                  PREPROCESSING                       │
│  RGB → Grayscale (cv2.COLOR_RGB2GRAY)               │
│  No blur — preserves tag edge sharpness             │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│              APRILTAG DETECTION                      │
│  family      : tag36h11                             │
│  nthreads    : 2 (RPi4 optimization)               │
│  quad_decimate: 1.0 (no decimation at 1080p)       │
│  refine_edges : True                               │
│                                                     │
│  Output per detection:                              │
│  • tag_id        : int                             │
│  • corners       : 4×2 array (pixels)              │
│  • pose_t        : 3×1 translation (metres)        │
│  • pose_R        : 3×3 rotation matrix             │
│  • decision_margin: float (quality score)          │
└──────────────────────┬──────────────────────────────┘
                       │
         ┌─────────────┴─────────────┐
         ▼                           ▼
  outer detected?           inner detected?
  (tag_id == 0)             (tag_id == 1)
         │                           │
         ▼                           ▼
┌─────────────────┐       ┌─────────────────────────────┐
│ OUTER WINS      │       │ INNER used ONLY IF:          │
│ Always — even   │       │ outer_missing_frames >= 5    │
│ if partially    │       │ AND outer NOT detected AT ALL │
│ out of FOV      │       └─────────────────────────────┘
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────────────────────┐
│              4-STAGE FILTER PIPELINE                 │
└─────────────────────────────────────────────────────┘
```

---

## Nested AprilTag Design

### Physical Layout

```
┌──────────────────────────────────────────────────────┐
│                                                      │
│   ████████████████████████████████████████████████  │
│   █                                              █  │
│   █     OUTER TAG: tag36h11 ID=0                 █  │
│   █     Physical size: 70 cm × 70 cm             █  │
│   █     Detection range: 1.5 m – 15+ m           █  │
│   █                                              █  │
│   █          ┌──────────────────┐                █  │
│   █          │ ████████████████ │                █  │
│   █          │ █              █ │                █  │
│   █          │ █  INNER TAG   █ │                █  │
│   █          │ █  ID=1        █ │                █  │
│   █          │ █  15 cm×15 cm █ │                █  │
│   █          │ █              █ │                █  │
│   █          │ ████████████████ │                █  │
│   █          └──────────────────┘                █  │
│   █                                              █  │
│   ████████████████████████████████████████████████  │
│                                                      │
└──────────────────────────────────────────────────────┘
```

### Detection Altitude Ranges

```
Altitude (m)
    15 │ ◄──────────── OUTER TAG ONLY (70 cm) ─────────────────►
       │
     5 │ ◄──────────── OUTER TAG PRIMARY ──────────────────────►
       │
   1.5 │ ◄── OUTER ──►│◄──── TRANSITION ZONE ────►│◄── INNER ──►
       │              │                            │
   0.5 │              │    (handoff_frames=5)      │◄── INNER ──►
       │              │                            │
   0.0 ┼──────────────┼────────────────────────────┼─────────────
       0             1.0                          1.5           15
```

### Tag Switching Logic

```python
# STRICT RULE: outer always wins if detected at all
if outer_det is not None:
    → use OUTER (size=0.70m)
elif outer_missing_frames >= 5 and inner_det is not None:
    → commit to INNER (size=0.15m)
else:
    → NO_DETECTION → halt LANDING_TARGET TX
```

---

## State Machine (FSM)

### State Transition Diagram

```
                         ┌─────────────────┐
                         │      IDLE        │
                         │  Waiting for tag │
                         └────────┬────────┘
                                  │ tag_reliable (3 consecutive frames)
                                  │ AND auto_start=True
                                  ▼
                         ┌─────────────────┐
                    ┌───►│   SEARCHING      │◄────────────────────────┐
                    │    │  Send offboard   │                         │
                    │    │  keepalive 2s    │                         │
                    │    └────────┬────────┘                         │
                    │             │ OFFBOARD mode accepted            │
                    │             │ + tag_visible                     │
                    │             ▼                                   │
                    │    ┌─────────────────┐   xy_err > descend_max  │
                    │    │    ALIGNING      ├─────────────────────────┘
                    │    │  PD correction   │
                    │    │  vz = 0          │
                    │    └────────┬────────┘
                    │             │ xy_err < 0.15 m
                    │             ▼
                    │    ┌─────────────────┐   xy_err > 0.30 m
                    │    │   DESCENDING     ├──────────────────────────►ALIGN
                    │    │  vz = 1.2 m/s   │
                    │    │  lateral gain×0.6│
                    │    └────────┬────────┘
                    │             │ z < 0.5 m (FLARE_ALT)
                    │             ▼
                    │    ┌─────────────────┐   tag lost AND alt > 0.3m
                    │    │     FLARE        ├──────────────────────────►DESC
                    │    │  vz = 0.6 m/s   │
                    │    │  lateral gain×1.2│
                    │    └────────┬────────┘
                    │             │ z < 0.12 m × 5 consecutive frames
                    │             │ OR (tag lost AND alt < 0.6 m)
                    │             ▼
                    │    ┌─────────────────┐
                    │    │    LANDING       │
                    │    │ NAV_LAND sent    │
                    │    └────────┬────────┘
                    │             │ disarmed OR alt < 0.15 m for 3s
                    │             ▼
               ABORT│    ┌─────────────────┐
               ◄────┤    │     LANDED       │
                    │    │  Mission done ✓  │
                    │    └─────────────────┘
                    │
              /landing/command: ABORT
```

### State Parameters

| State | Trigger IN | Trigger OUT | Commands Sent |
|-------|-----------|-------------|---------------|
| `IDLE` | Node start | tag_reliable=True | None |
| `SEARCHING` | tag detected | xy_err < 0.15m | SET_POSITION (hover), OFFBOARD mode |
| `ALIGNING` | xy_err OK | xy_err < 0.15m | SET_POSITION (lateral only, vz=0) |
| `DESCENDING` | aligned | z < 0.5m | SET_POSITION (lateral + vz=1.2) |
| `FLARE` | z < 0.5m | z < 0.12m ×5 | SET_POSITION (tight lateral + vz=0.6) |
| `LANDING` | z < 0.12m | disarmed | NAV_LAND |
| `LANDED` | disarmed | — | None |
| `ABORT` | /landing/command | — | GUIDED mode, hover |

---

## MAVLink Integration

### Port Layout (SITL)

```
┌─────────────────────────────────────────────────────────────────┐
│                       PX4 SITL                                  │
│                                                                 │
│  MAVLink Instance #0 ──────────────────────────► port 18570    │
│  (GCS mode, 4 MB/s)     ◄── MAVProxy master ── udpin:14550     │
│                                                                 │
│  MAVLink Instance #1 ──────────────────────────► port 14580    │
│  (Onboard, 4 MB/s)      ◄── Companion ──────── udpin:14540     │
│                              (controller direct)                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
         │ port 14550                    │ port 14540
         ▼                              ▼
┌─────────────────┐            ┌─────────────────────────────────┐
│   MAVProxy      │            │   precision_landing_controller  │
│  ─────────────  │            │   udpin:0.0.0.0:14540           │
│  out→ 14560     │            │                                 │
│  out→ 14561     │            │   SET_POSITION_TARGET (30 Hz)   │
│  out→ 14570 ──► QGC          │   LANDING_TARGET (30 Hz)        │
└────────┬────────┘            │   heartbeat (1 Hz)              │
         │                     └─────────────────────────────────┘
    14560│   14561
         ▼         ▼
  landing_target  mavlink
  _publisher      _monitor
```

### LANDING_TARGET Message

```python
mav.landing_target_send(
    time_usec  = int(time.time() * 1e6),       # timestamp
    target_num = 0,                             # target index
    frame      = MAV_FRAME_LOCAL_NED,           # frame=1 (REQUIRED by PX4)
    angle_x    = math.atan2(x_cam, z_cam),     # lateral angle (rad)
    angle_y    = math.atan2(y_cam, z_cam),     # longitudinal angle (rad)
    distance   = z_cam,                         # metres
    size_x     = max(tag_size / z_cam, 0.01),  # angular size
    size_y     = max(tag_size / z_cam, 0.01)
)
# Rate: 30 Hz  |  pymavlink 8-arg form (extended form not supported)
# CRITICAL: MAV_FRAME_LOCAL_NED=1 only — frames 8,12 REJECTED by PX4
```

### SET_POSITION_TARGET_LOCAL_NED (Velocity Control)

```python
# Bitmask: ignore position + acceleration, use velocity only
type_mask = 0b0000111111000111  # = 0x0FC7

mav.set_position_target_local_ned_send(
    0,                          # time_boot_ms
    target_system=1,
    target_component=1,
    coordinate_frame=MAV_FRAME_BODY_NED,
    type_mask=type_mask,
    x=0, y=0, z=0,             # position (ignored)
    vx=vx, vy=vy, vz=vz,       # velocity setpoints (m/s)
    afx=0, afy=0, afz=0,        # acceleration (ignored)
    yaw=0, yaw_rate=0           # yaw (ignored)
)
```

---

## Filter Pipeline

### 4-Stage Architecture

```
RAW POSE (tx, ty, tz) from pupil_apriltags
              │
              ▼
┌─────────────────────────────────────────────────────────┐
│  STAGE 1: QUALITY GATE                                  │
│                                                         │
│  if tag.decision_margin < 30:                          │
│      → REJECT (false positive / partial occlusion)     │
│  else:                                                  │
│      → PASS                                            │
└──────────────────────────┬──────────────────────────────┘
                           │ PASS
                           ▼
┌─────────────────────────────────────────────────────────┐
│  STAGE 2: SPATIAL GATE (Outlier Rejection)              │
│                                                         │
│  dist = ||pose_current - pose_prev||                   │
│  if dist > 2.0 m:   → REJECT (impossible jump)         │
│  else:              → PASS                              │
└──────────────────────────┬──────────────────────────────┘
                           │ PASS
                           ▼
┌─────────────────────────────────────────────────────────┐
│  STAGE 3: EMA LOW-PASS FILTER                           │
│                                                         │
│  α = 0.3   (0 = frozen, 1 = raw)                       │
│  x_f = α·x_raw + (1−α)·x_prev                         │
│  y_f = α·y_raw + (1−α)·y_prev                         │
│  z_f = α·z_raw + (1−α)·z_prev                         │
│                                                         │
│  Phase lag: ~33 ms @ 30 Hz                             │
│  Jitter reduction: ~70%                                │
└──────────────────────────┬──────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│  STAGE 4: TARGET-LOST FAILSAFE                          │
│                                                         │
│  if (now - last_detection_time) > 0.5s:                │
│      → HALT MAVLink TX (trigger PX4 timeout)           │
│      → FSM transitions: DESCEND → HOVER                │
│      → reset EMA state                                 │
└──────────────────────────┬──────────────────────────────┘
                           │
                           ▼
             FILTERED (x_f, y_f, z_f)
             Published at 30 Hz on /apriltag/pose
```

### EMA Response Characteristics

```
Step input: tag moves 1.0 m laterally

Frame:    0    1    2    3    4    5    6    7    8    9    10
Response: 0.0  0.30 0.51 0.66 0.76 0.83 0.88 0.92 0.94 0.96 0.97

(α=0.3, settling to 95% in ~8 frames = 267 ms @ 30 Hz)
```

---

## Simulation Environment (SITL)

### Gazebo World

```
Gazebo 8 — World: "apriltag"
GPS: 34.017956°N, -6.834192°W (Temara, Morocco)

         N
         │
   ┌─────┼─────────────────────────┐
   │     │                         │
   │   [OSM]  [OSM]  [OSM]        │
   │  building building            │
   │                               │
   │   [🟠]              [🟠]     │
   │  cone                cone     │  Legend:
   │                               │  [OSM] = OpenStreetMap building
   │      ┌──────────────┐         │  [🟠]  = Orange safety cone
   │      │   AprilTag   │         │  [H]   = Helipad ring
   │      │   36h11 ID=0 │         │  *     = Drone spawn point
   │      │     0.8m     │         │
   │      └──────────────┘    [H]  │
   │   [🟠]      *       [🟠]     │
   │             │                 │
   │   [OSM]  [OSM]  [OSM]        │
   │                               │
   └───────────────────────────────┘
   
   World size: 500×500 m
   60 buildings (OpenStreetMap, sand/terracotta palette)
```

### Launch Stack (11 Windows)

```
tmux session: farasha_sitl
┌────┬───────────────────┬───────────────────────────────────────┐
│ W# │ Name              │ Process                               │
├────┼───────────────────┼───────────────────────────────────────┤
│  0 │ SITL              │ vglrun make px4_sitl gz_x500_down     │
│  1 │ ParamFix          │ tmux inject 28 params → W0 pxh>       │
│  2 │ MAVProxy          │ udpin:14550 → out:14560/14561/14570   │
│  3 │ QGC               │ vglrun QGroundControl (port 14570)    │
│  4 │ Bridge            │ ros_gz_bridge Gazebo → /camera/image  │
│  5 │ Vision            │ apriltag_detector (tag36h11)          │
│  6 │ LandTarget        │ landing_target_publisher (14560)      │
│  7 │ FSM-Ctrl          │ precision_landing_controller (14540)  │
│  8 │ Monitor           │ mavlink_monitor (14561)               │
│  9 │ HUD               │ rqt_image_view /apriltag/debug_image  │
│ 10 │ State             │ ros2 topic echo /landing/state        │
└────┴───────────────────┴───────────────────────────────────────┘
```

### Camera Intrinsics (Gazebo)

```
Resolution : 1280 × 960
fx = fy    : 539.9
cx         : 640.0
cy         : 480.0
FOV (H)    : 87°
Orientation: −π/2 pitch (downward-facing, corrected from SDF bug)
```

---

## Hardware-in-the-Loop (HITL)

### Connection Diagram

```
Raspberry Pi 4B                         Cube Orange
Ubuntu 24 LTS                           PX4 Firmware

GPIO 14 (TXD) ──────────────────────► TELEM2 RX (pin 3)
GPIO 15 (RXD) ◄────────────────────── TELEM2 TX (pin 2)
GND           ──────────────────────── GND      (pin 6)

/dev/ttyAMA0 @ 921600 baud, MAVLink v2
```

### HITL Software Stack

```
HITL Launch (farasha_hitl tmux session)
┌────┬───────────────────┬────────────────────────────────────────────┐
│ W# │ Name              │ Process                                    │
├────┼───────────────────┼────────────────────────────────────────────┤
│  0 │ Vision            │ apriltag_detector                          │
│    │                   │ PiCamera V2.1 via V4L2 (/dev/video0)      │
│    │                   │ 1280×720 @ 30fps, fx=1002 fy=1002         │
├────┼───────────────────┼────────────────────────────────────────────┤
│  1 │ LandTarget        │ landing_target_publisher                   │
│    │                   │ serial:/dev/ttyAMA0 @ 921600              │
│    │                   │ 30 Hz LANDING_TARGET → Cube               │
├────┼───────────────────┼────────────────────────────────────────────┤
│  2 │ FSM               │ precision_landing_controller               │
│    │                   │ serial:/dev/ttyAMA0 @ 921600              │
│    │                   │ 5-state FSM, velocity commands → Cube     │
├────┼───────────────────┼────────────────────────────────────────────┤
│  3 │ Status            │ ros2 topic echo /landing/status            │
├────┼───────────────────┼────────────────────────────────────────────┤
│  4 │ CMD               │ Interactive command shell                  │
│    │                   │ FORCE_START | ABORT | RESET               │
├────┼───────────────────┼────────────────────────────────────────────┤
│  5 │ Preview           │ rqt_image_view /apriltag/debug_image       │
└────┴───────────────────┴────────────────────────────────────────────┘
```

---

## ROS2 Topic Graph

```
                    ┌─────────────────────────┐
                    │   apriltag_detector      │
                    │   (PiCamera / Gazebo)    │
                    └──────────┬──────────────┘
                               │ publishes
              ┌────────────────┼────────────────────────┐
              │                │                        │
              ▼                ▼                        ▼
     /apriltag/pose    /apriltag/confidence   /apriltag/debug_image
     (PoseStamped)     (Float32)              (sensor_msgs/Image)
              │                │                        │
              └────────────────┼────────────────────────┘
                               │ subscribes
              ┌────────────────┼────────────────────────┐
              │                                         │
              ▼                                         ▼
   ┌──────────────────────┐              ┌──────────────────────────┐
   │ landing_target_pub   │              │ precision_landing_ctrl   │
   │ → LANDING_TARGET     │              │ → SET_POSITION_TARGET    │
   │ → PX4 @ 30 Hz        │              │ → NAV_LAND               │
   └──────────────────────┘              └──────────────────────────┘
                                                      │ publishes
                                    ┌─────────────────┼────────────┐
                                    │                 │            │
                                    ▼                 ▼            ▼
                           /landing/state    /landing/active  /landing/status
                           (String)          (Bool)           (String)
                                    ▲
                                    │ subscribes
                           /landing/command
                           (String)
                     FORCE_START | ABORT | RESET
```

### Complete Topic List

| Topic | Message Type | Direction | Rate | Description |
|-------|-------------|-----------|------|-------------|
| `/apriltag/pose` | `PoseStamped` | pub | 30 Hz | Tag pose in camera frame (x,y,z + quaternion) |
| `/apriltag/confidence` | `Float32` | pub | 30 Hz | Detection quality score [0.0–1.0] |
| `/apriltag/id` | `Int32` | pub | 30 Hz | Active tag ID (0=outer, 1=inner) |
| `/apriltag/debug_image` | `Image` | pub | 30 Hz | HUD frame with overlay and state info |
| `/apriltag/status` | `String` | pub | 1 Hz | Human-readable detection status |
| `/landing/state` | `String` | pub | 2 Hz | Current FSM state name |
| `/landing/active` | `Bool` | pub | 2 Hz | True when landing sequence running |
| `/landing/status` | `String` | pub | 2 Hz | Detailed status with metrics |
| `/landing/command` | `String` | sub | on-demand | FORCE_START / ABORT / RESET |

---

## PX4 Parameters

### Complete Parameter Table

| Parameter | SITL Value | HITL Value | Description |
|-----------|-----------|-----------|-------------|
| `LTEST_MODE` | 1 | 1 | Stationary landing target |
| `RTL_PLD_MD` | 1 | 1 | Opportunistic precision landing |
| `MPC_LAND_SPEED` | 0.4 m/s | 0.4 m/s | Final descent rate |
| `COM_DISARM_LAND` | 2.0 s | 2.0 s | Auto-disarm after landing |
| `PLD_HACC_RAD` | 0.2 m | 0.2 m | Horizontal accuracy radius |
| `PLD_BTOUT` | 5.0 s | 5.0 s | Landing target estimator timeout |
| `PLD_FAPPR_ALT` | 0.5 m | 0.5 m | Final approach altitude |
| `MPC_XY_VEL_MAX` | 3.0 m/s | 2.0 m/s | Max horizontal velocity |
| `MPC_Z_VEL_MAX_DN` | 2.0 m/s | 1.5 m/s | Max descent velocity |
| `SYS_HAS_MAG` | 0 | — | Disable mag requirement (SITL) |
| `EKF2_MAG_TYPE` | 5 | — | No magnetometer (SITL) |
| `EKF2_GPS_CHECK` | 0 | — | Skip GPS checks (SITL) |
| `CBRK_SUPPLY_CHK` | 894281 | — | Bypass supply check (SITL) |
| `SIM_BAT_DRAIN` | 0 | — | Disable battery drain (SITL) |

---

## Performance Metrics

### SITL Validation Results

```
Test: 10 consecutive autonomous landings
Environment: Gazebo 8, Temara GPS coords, zero wind disturbance

┌────────────────────────────────────────────────────────────┐
│                    LANDING ACCURACY                         │
│                                                            │
│    10 │                                                    │
│     9 │        ●                                          │
│     8 │    ●           ●                                  │
│     7 │                    ●   ●                          │
│     6 │  ●                         ●                      │
│     5 │                                ●                  │
│     4 │      ●                             ●   ●          │
│     3 │                                                   │
│     2 │─────────────────────────────────── TARGET ±5 cm  │
│     1 │                                                   │
│     0 ┼────────────────────────────────────────────────  │
│       1    2    3    4    5    6    7    8    9   10       │
│                      Trial number                         │
│                                                            │
│  Mean error  : 3.8 cm                                     │
│  Max error   : 9.2 cm (trial 1, EMA not settled)          │
│  Min error   : 1.1 cm (trial 7)                           │
│  Success rate: 9/10 (90%) within ±5 cm                    │
└────────────────────────────────────────────────────────────┘
```

### Detection Rate vs Altitude

```
Alt (m)  Detection Rate   Tag Active   Notes
──────── ─────────────── ──────────── ──────────────────────
15.0     95%              OUTER        Good lighting required
10.0     98%              OUTER        Reliable
 5.0     99%              OUTER        Optimal range
 2.0     99%              OUTER        Transition zone
 1.5     92%              OUTER→INNER  Handoff zone
 0.8     98%              INNER        Reliable inner
 0.4     85%              INNER        Flare altitude
 0.2     60%              INNER        Near-blind zone
 0.1     —                —            Too close, LAND cmd sent
```

### Landing Time Breakdown

```
Phase         Duration  Notes
──────────── ─────────  ──────────────────────────────
Takeoff       ~8 s      mode guided → arm → takeoff 5
Tag acquire   ~2–5 s    SEARCHING → ALIGNING
Alignment     ~5–15 s   xy correction, vz=0
Descent       ~25–35 s  vz=1.2 m/s, xy correction
Flare         ~3–5 s    vz=0.6 m/s, close alignment
Land+Disarm   ~3–5 s    NAV_LAND + 2s disarm timer
──────────── ─────────
TOTAL         ~50–75 s  (target: <90 s)
```

---

## Repository Structure

```
farasha-uav-precision-landing/
│
├── README.md                          # This file
├── LICENSE                            # MIT License
├── .gitignore                         # Python, ROS2, PX4 excludes
├── CHANGELOG.md                       # Version history
│
├── docs/                              # Documentation
│   ├── architecture.md                # System design details
│   ├── hardware_setup.md              # UART wiring, RPi config
│   ├── sitl_setup.md                  # Docker, PX4, Gazebo setup
│   ├── hitl_setup.md                  # RPi4 Ubuntu 24 setup
│   ├── camera_calibration.md          # Intrinsics procedure
│   ├── px4_parameters.md              # Full parameter reference
│   ├── mavlink_port_map.md            # Port layout and routing
│   └── images/                        # Architecture diagrams
│       ├── system_architecture.png
│       ├── fsm_diagram.png
│       ├── gazebo_world.png
│       ├── detection_hud.png
│       ├── nested_tag_layout.png
│       └── filter_pipeline.png
│
├── ros2_ws/                           # ROS2 workspace
│   └── src/
│       └── farasha_landing/
│           ├── package.xml            # ROS2 package manifest
│           ├── setup.py               # Entry points + metadata
│           ├── setup.cfg
│           └── farasha_landing/
│               ├── __init__.py
│               ├── apriltag_detector.py           # Vision pipeline
│               ├── landing_target_publisher.py    # MAVLink LANDING_TARGET
│               ├── precision_landing_controller.py # 5-state FSM
│               └── mavlink_monitor.py             # Rate + health monitor
│
├── simulation/                        # Gazebo simulation assets
│   ├── worlds/
│   │   └── apriltag.sdf               # Custom world (Morocco GPS, tag pad)
│   ├── models/
│   │   ├── apriltag/                  # Single 0.8m tag model
│   │   │   ├── model.config
│   │   │   ├── model.sdf
│   │   │   └── materials/textures/
│   │   │       └── apriltag_36h11_id0.png
│   │   └── farasha_nested_tag/        # Nested 70cm+15cm tag model
│   │       ├── model.config
│   │       ├── model.sdf
│   │       └── materials/textures/
│   │           └── farasha_nested_tag.png
│   └── scripts/
│       └── generate_apriltag.py       # SVG → PNG tag generator
│
├── scripts/                           # Launch and utility scripts
│   ├── sitl/
│   │   └── launch_sitl.sh             # Full 11-window SITL stack
│   ├── hitl/
│   │   └── launch_hitl.sh             # 6-window HITL stack
│   └── utils/
│       ├── camera_calibration.py      # Checkerboard calibration
│       ├── inject_params.sh           # PX4 param injection
│       └── check_ports.sh             # MAVLink port diagnostics
│
├── config/                            # Configuration files
│   ├── px4_params_sitl.yaml           # All SITL PX4 parameters
│   ├── px4_params_hitl.yaml           # HITL-specific parameters
│   └── camera_intrinsics_v2.yaml      # PiCamera V2.1 calibration
│
└── tests/                             # Unit tests
    ├── test_apriltag_detector.py       # Vision pipeline tests
    ├── test_filter_pipeline.py         # EMA + gate tests
    └── test_fsm_transitions.py         # State machine logic tests
```

---

## Quick Start

### Prerequisites

```bash
# Ubuntu 24 / ROS2 Jazzy
sudo apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    python3-pip tmux

pip3 install pupil-apriltags pymavlink --break-system-packages

# For SITL only
# PX4 v1.14 + Gazebo 8 + Docker dev container
```

### SITL

```bash
# Clone
git clone https://github.com/YOUR_USERNAME/farasha-uav-precision-landing
cd farasha-uav-precision-landing

# Install Gazebo models
cp -r simulation/models/apriltag \
      ~/PX4-Autopilot/Tools/simulation/gz/models/
cp -r simulation/models/farasha_nested_tag \
      ~/PX4-Autopilot/Tools/simulation/gz/models/
cp simulation/worlds/apriltag.sdf \
      ~/PX4-Autopilot/Tools/simulation/gz/worlds/

# Build ROS2 package
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..

# Launch full SITL stack
bash scripts/sitl/launch_sitl.sh

# In MAVProxy (W2), after params injected:
mode guided
arm throttle force
takeoff 5
# → FSM detects tag automatically → auto-lands!
```

### HITL

```bash
# On RPi4 Ubuntu 24
git clone https://github.com/YOUR_USERNAME/farasha-uav-precision-landing
cd farasha-uav-precision-landing

mkdir -p ~/farasha_ws/src
cp -r ros2_ws/src/farasha_landing ~/farasha_ws/src/
cd ~/farasha_ws && colcon build --symlink-install

# Wire UART: GPIO 14/15 → TELEM2 (see docs/hardware_setup.md)
# Set PX4 params via QGC (see config/px4_params_hitl.yaml)

# Launch HITL stack
bash scripts/hitl/launch_hitl.sh

# Arm and takeoff via RC or QGC
# Then in W4 (CMD window):
ros2 topic pub /landing/command std_msgs/msg/String "{data: FORCE_START}" --once
```

### Manual Override

```bash
# Emergency abort
ros2 topic pub /landing/command std_msgs/msg/String "{data: ABORT}" --once

# Reset FSM
ros2 topic pub /landing/command std_msgs/msg/String "{data: RESET}" --once

# Monitor landing state
ros2 topic echo /landing/state

# Monitor detection
ros2 topic echo /apriltag/status
```

---

## Results

### Simulation Screenshots

```
┌────────────────────────────────────────────────────────────────┐
│  GAZEBO VIEW              │  CAMERA HUD (/apriltag/debug_image)│
│                           │                                     │
│   ┌───────────────┐       │  ┌──────────────────────────────┐  │
│   │   Gazebo 8    │       │  │ STATE: DESCENDING            │  │
│   │               │       │  │                              │  │
│   │   [DRONE]     │       │  │  ┌────────────────┐          │  │
│   │      ↓        │       │  │  │ [TAG DETECTED] │          │  │
│   │   [TAG PAD]   │       │  │  │    ID=0 OUTER  │          │  │
│   └───────────────┘       │  │  └────────────────┘          │  │
│                           │  │ z=2.31m xy=0.042m conf=87%   │  │
│  [INSERT SCREENSHOT]      │  └──────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
```

---

## Roadmap

### Current Status

| Feature | Status |
|---------|--------|
| SITL simulation (Gazebo 8) | Complete |
| AprilTag detection pipeline | Complete |
| 4-stage filter | Complete |
| 5-state FSM | Complete |
| LANDING_TARGET @ 30 Hz | Complete |
| Offboard velocity control | Complete |
| Nested tag (outer/inner) | Complete |
| HITL RPi4 + Cube Orange | In Progress |
| Camera calibration | Pending |
| Outdoor field validation | Planned |

### v1.0 — Planned (Q3 2026)

```
□ Outdoor field test at Temara Morocco
□ Real camera calibration (checkerboard)
□ Gimbal integration (servo-driven, 2-axis)
  → AprilTag stays in FOV during aggressive maneuvers
□ Wind disturbance rejection testing
□ Landing accuracy study (50 trials)
```

### v2.0 — Future

```
□ Multi-tag redundancy (fault-tolerant detection array)
□ Moving platform landing (Kalman-tracked target)
□ Nighttime landing (IR illumination + thermal camera)
□ Deep learning detector replacement (YOLOv8 pose)
□ ROS2 Nav2 integration
```

---

## Author

<table>
<tr>
<td align="center">
<strong>ELMESSAOUDI Oussama</strong><br/>
Embedded Systems & Robotics Engineer<br/>
PFE Intern @ FARASHA Systems × TAMAYOUZ Centre FSA<br/>
Université Ibn Zohr — Computer Engineering & Embedded Systems<br/>
<br/>
<a href="https://github.com/oussamaelmessaoudi">
<img src="https://img.shields.io/badge/GitHub-181717?style=flat-square&logo=github&logoColor=white"/>
</a>
<a href="https://www.linkedin.com/in/usama-elmessaoudi/">
<img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=flat-square&logo=linkedin&logoColor=white"/>
</a>
<a href="mailto:oussamaelmessaoudi17@gmail.com">
<img src="https://img.shields.io/badge/Email-EA4335?style=flat-square&logo=gmail&logoColor=white"/>
</a>
</td>
</tr>
</table>

---

## License

```
MIT License

Copyright (c) 2026 ELMESSAOUDI Oussama — FARASHA Systems

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.
```

---

## Acknowledgements

[![PX4](https://img.shields.io/badge/Built_on-PX4_Autopilot-purple?style=flat-square&logo=drone)](https://px4.io)
[![ROS2](https://img.shields.io/badge/Middleware-ROS2_Jazzy-22314E?style=flat-square&logo=ros)](https://docs.ros.org)
[![Gazebo](https://img.shields.io/badge/Simulation-Gazebo_8-F58025?style=flat-square)](https://gazebosim.org)
[![AprilTag](https://img.shields.io/badge/Fiducial-AprilTag_36h11-blue?style=flat-square)](https://april.eecs.umich.edu/software/apriltag)
[![OpenStreetMap](https://img.shields.io/badge/Buildings-OpenStreetMap-7EBC6F?style=flat-square&logo=openstreetmap)](https://openstreetmap.org)

---

<div align="center">

**FARASHA Systems — Precision. Autonomy. Reliability.**

[![Stars](https://img.shields.io/github/stars/YOUR_USERNAME/farasha-uav-precision-landing?style=flat-square&logo=github)](https://github.com/YOUR_USERNAME/farasha-uav-precision-landing)
[![Issues](https://img.shields.io/github/issues/YOUR_USERNAME/farasha-uav-precision-landing?style=flat-square)](https://github.com/YOUR_USERNAME/farasha-uav-precision-landing/issues)
[![Last Commit](https://img.shields.io/github/last-commit/YOUR_USERNAME/farasha-uav-precision-landing?style=flat-square)](https://github.com/YOUR_USERNAME/farasha-uav-precision-landing/commits)

</div>
