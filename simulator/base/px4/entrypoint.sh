#!/bin/bash

# If PX4_SIM_HOST is set, lookup IP for simulator
if [ -n "$PX4_SIM_HOST" ]; then
    echo "Looking up sim host"
    export PX4_SIM_IP=$(getent ahostsv4 $PX4_SIM_HOST | head -1 | awk '{print $1}')
else
    echo "PX4_SIM_HOST not set, assuming loopback"
fi
echo "Setting PX4_SIM_IP to $PX4_SIM_IP"

# If PX4_OFFBOARD_HOST is set, lookup IP for offboard
if [ -n "$PX4_OFFBOARD_HOST" ]; then
    echo "Looking up offboard host"
    export PX4_OFFBOARD_IP=$(getent ahostsv4 $PX4_OFFBOARD_HOST | head -1 | awk '{print $1}')
else
    echo "PX4_OFFBOARD_HOST not set, assuming loopback"
fi
echo "Setting PX4_OFFBOARD_IP to $PX4_OFFBOARD_IP"

# If PX4_INSTANCE is "ordinal", then set instance id to take PX4_INSTANCE_BASE + ORDINAL from StatefulSet hostname
# Therefore SYSID is set to PX4_INSTANCE_BASE + ORDINAL + 1
# Hostname is of the form '<stateful set name>-<ordinal>'
if [ "$PX4_INSTANCE" == "ordinal" ]; then
    ORDINAL="${HOSTNAME##*-}"
    export PX4_INSTANCE=$((PX4_INSTANCE_BASE + ORDINAL));
    echo "PX4_INSTANCE was 'ordinal' therefore set to $PX4_INSTANCE (from base: $PX4_INSTANCE_BASE and hostname: $HOSTNAME)"
elif (($PX4_INSTANCE >= 0 && $PX4_INSTANCE <= 254 )); then
    echo "PX4_INSTANCE setting as specified: $PX4_INSTANCE"
else
    echo "PX4_INSTANCE (set to $PX4_INSTANCE) is invalid, setting to 0. Must either be set to 'ordinal' or number between 0 and 254"
    export PX4_INSTANCE=0;
fi
export PX4_SYSID=$((PX4_INSTANCE + 1))
echo "PX4 SYSID is set to $PX4_SYSID"

# Enable simulated external vision system in SITL
# To be used with the gazebo ros state (libgazebo_ros_state) plugin to emulate VICON
# See: https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system
if [[ $ENABLE_EXTERNAL_VISION ]]; then
    # Check if params have already been set
    RCS_FILE="/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS"
    if ! grep -q EKF2_AID_MASK "$RCS_FILE"; then
        echo "Enabled External Vision in SITL, set EKF2_HGT_MODE to 3 and EKF2_AID_MASK to 24 (EV_POS+EV_YAW)"
        sed -i '/param set IMU_INTEG_RATE 250/a param set EKF2_HGT_MODE 3' "$RCS_FILE"
        sed -i '/param set IMU_INTEG_RATE 250/a param set EKF2_AID_MASK 24' "$RCS_FILE"
        sed -i '/param set IMU_INTEG_RATE 250/a param set EKF2_EV_DELAY 10' "$RCS_FILE"
    fi
fi

if [[ $ENABLE_ROVER ]]; then
    RCS_FILE="/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS"
    echo "Enable Rover, EKF2_RANGE_AID set to 0"
    sed -i '/param set IMU_INTEG_RATE 250/a param set EKF2_RNG_AID 0' "$RCS_FILE"
    sed -i '/param set IMU_INTEG_RATE 250/a param set EKF2_GND_EFF_DZ 0.0' "$RCS_FILE"
    sed -i '/param set IMU_INTEG_RATE 250/a param set EKF2_GND_MAX_HGT 5.0' "$RCS_FILE"
fi

/ros_entrypoint.sh "$@"
