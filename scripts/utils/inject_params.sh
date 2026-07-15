#!/bin/bash
# Inject all FARASHA params into running PX4 SITL
# Usage: bash inject_params.sh <tmux_session> <window_id>

SESSION=${1:-"farasha_sitl"}
WIN=${2:-"0"}

send() {
  tmux send-keys -t ${SESSION}:${WIN} "$1" C-m
  sleep 0.35
}

echo "Injecting params into ${SESSION}:${WIN}..."

# Sensors
send "param set SYS_HAS_MAG 0"
send "param set SYS_HAS_BARO 0"

# EKF2
send "param set EKF2_EN 1"
send "param set EKF2_MAG_TYPE 5"
send "param set EKF2_GPS_CHECK 0"

# Arming
send "param set COM_ARM_WO_GPS 1"
send "param set COM_PREARM_MODE 0"
send "param set COM_RCL_EXCEPT 4"
send "param set NAV_RCL_ACT 0"
send "param set CBRK_SUPPLY_CHK 894281"
send "param set CBRK_IO_SAFETY 22027"
send "param set CBRK_USB_CHK 197848"

# Battery
send "param set SIM_BAT_DRAIN 0"
send "param set BAT_CRIT_THR 0.0"
send "param set BAT_EMERGEN_THR 0.0"
send "param set COM_LOW_BAT_ACT 0"
send "param set GF_ACTION 0"

# MAVLink
send "param set MAV_0_BROADCAST 1"
send "param set MAV_1_BROADCAST 1"

# Precision landing
send "param set LTEST_MODE 1"
send "param set RTL_PLD_MD 1"
send "param set MPC_LAND_SPEED 0.4"
send "param set COM_DISARM_LAND 2.0"
send "param set PLD_HACC_RAD 0.2"
send "param set PLD_BTOUT 5.0"
send "param set PLD_FAPPR_ALT 0.5"
send "param set MPC_XY_VEL_MAX 3.0"
send "param set MPC_Z_VEL_MAX_DN 2.0"

send "param save"
echo "✓ Done. Go to MAVProxy: mode guided → arm throttle force → takeoff 5"
