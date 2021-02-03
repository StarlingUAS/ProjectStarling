source mavros_setup.sh;

echo "Launching MAVROS with:"
echo "fcu_url=${MAVROS_FCU_URL}"
echo "sysid=${MAVROS_TGT_SYSTEM}"
echo "firmware=${MAVROS_TGT_FIRMWARE}"

ros2 launch launch/mavros_bridge.launch.xml fcu_url:=${MAVROS_FCU_URL} target_system:=${MAVROS_TGT_SYSTEM} firmware:=${MAVROS_TGT_FIRMWARE}
