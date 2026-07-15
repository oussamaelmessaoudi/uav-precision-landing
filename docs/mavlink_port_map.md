# MAVLink Port Map — SITL

## Port Layout
PX4 SITL
├── Port 14550 (GCS mode)   → MAVProxy master
├── Port 14540 (Onboard)    ← Companion controller (udpin:14540)
└── Port 14580              → PX4 sends FROM here
MAVProxy
├── master  = udpin:0.0.0.0:14550
├── out     = udp:127.0.0.1:14560  → landing_target_publisher
├── out     = udp:127.0.0.1:14561  → mavlink_monitor
└── out     = udp:127.0.0.1:14570  → QGroundControl
precision_landing_controller
└── connects DIRECTLY to PX4:14540 (udpin:0.0.0.0:14540)
bypasses MAVProxy — ensures offboard keepalive reaches PX4

## HITL Port Layout
Cube Orange (/dev/ttyAMA0 @ 921600)
├── landing_target_publisher  → serial write
└── precision_landing_controller → serial read/write

## Critical Notes

- Offboard keepalive MUST go directly to PX4 — NOT through MAVProxy
- MAVProxy does not forward companion messages back upstream to PX4
- Use `udpin:0.0.0.0:14540` for controller (bind mode, not push mode)
- PX4 needs 0.5s of SET_POSITION_TARGET stream before accepting OFFBOARD
