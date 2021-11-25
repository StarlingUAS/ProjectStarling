echo "---- xacro_launch.sh START ------------"
# Generate sim_model sdf
src_path="/src/PX4-Autopilot"

echo "Generating ${PX4_SIM_MODEL} instance ${PX4_INSTANCE} sysid ${PX4_SYSID}"
python3 ${src_path}/Tools/sitl_gazebo/scripts/jinja_gen.py \
    ${src_path}/Tools/sitl_gazebo/models/${PX4_SIM_MODEL}/${PX4_SIM_MODEL}.sdf.jinja ${src_path}/Tools/sitl_gazebo \
    --mavlink_tcp_port ${PX4_SITL_PORT} \
    --mavlink_udp_port $((${PX4_OFFBOARD_PORT_BASE}+${PX4_INSTANCE})) \
    --mavlink_id ${PX4_SYSID} \
    # --gst_udp_port $((5600+${PX4_INSTANCE}))  \
    # --video_uri $((5600+${PX4_INSTANCE}))  \
    # --mavlink_cam_udp_port $((14530+${PX4_INSTANCE})) \
    --output-file /tmp/${PX4_SIM_MODEL}_${PX4_SYSID}.sdf

echo "Model TCP Port set to: ${PX4_SITL_PORT}"
echo "Specific SDF saved to /tmp/${PX4_SIM_MODEL}_${PX4_SYSID}.sdf"

# Modifying drone start location based on PX4 INSTANCE
# Drones will start in a spiral starting from (PX4_SIM_INIT_LOC_X, PX4_SIM_INIT_LOC_Y)
re_number='^[0-9]+$'
ORDINAL="${HOSTNAME##*-}"
if [[ $ORDINAL =~ $re_number ]] && ! ${PX4_SIM_FORCE_USE_SET_POSITION} ; then
    echo "Starting location set by spiral"
    separation_distance=${PX4_SIM_SPAWN_SEP_DISTANCE:-1}
    x=0
    y=0
    dx=0
    dy=-1
    SPIRAL_POSITION=${ORDINAL:-${PX4_INSTANCE}}
    echo "Calculating position $SPIRAL_POSITION on starting spiral"
    for (( i=0; i<$SPIRAL_POSITION; i++))
    do  
        if (( x == y || (x < 0 && x == -y) || (x > 0 && x == 1-y ) )); then
            tmp=$dx
            dx=$((-dy))
            dy=$tmp
        fi
        echo "Step $i: $x, $y"
        x=$((x+dx))
        y=$((y+dy))
    done

    export PX4_SIM_INIT_LOC_X=$(( PX4_SIM_INIT_LOC_X + x*separation_distance ))
    export PX4_SIM_INIT_LOC_Y=$(( PX4_SIM_INIT_LOC_Y + y*separation_distance ))
else
    echo "Using user specified starting location"
fi
echo "Starting location set to ($PX4_SIM_INIT_LOC_X, $PX4_SIM_INIT_LOC_Y, $PX4_SIM_INIT_LOC_Z)"

echo "---- xacro_launch.sh END ------------"
