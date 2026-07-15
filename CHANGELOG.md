# Changelog

## [0.3.0] — 2026-04-22 (HITL)
- Migrated detector to PiCamera V2.1 via V4L2 (bypasses libcamera crash)
- Added serial MAVLink connection for Cube Orange via /dev/ttyAMA0
- Nested tag handoff: outer 70cm → inner 15cm with strict FOV rule
- HITL launch script with tmux session management

## [0.2.0] — 2026-04-10 (SITL)
- Full autonomous FSM: SEARCHING → ALIGNING → DESCENDING → FLARE → LANDING
- Fixed LANDING_TARGET to 8-arg pymavlink call (MAV_FRAME_LOCAL_NED)
- Fixed offboard keepalive: udpin:14540 direct to PX4 (bypasses MAVProxy)
- Added offboard pre-stream: 2s keepalive before mode switch
- PD control in ALIGNING state — eliminates circular orbit
- Speed: descent 1.2 m/s, flare 0.6 m/s — landing < 1 min

## [0.1.0] — 2026-03-16 (Vision Pipeline)
- ROS2 Jazzy package: farasha_landing_vision
- AprilTag detector with 4-stage filter pipeline
- Camera orientation bug fixed (−π/2 pitch)
- Gazebo world: Temara Morocco GPS, 60 OSM buildings, AprilTag pad
- MAVProxy port layout: 14550 → 14560/14561/14570
