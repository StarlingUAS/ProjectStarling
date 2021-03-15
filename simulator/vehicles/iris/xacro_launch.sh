# Generate sim_model sdf
src_path="/src/PX4-Autopilot"

echo "Generating ${PX4_SIM_MODEL} instance ${PX4_INSTANCE} sysid ${PX4_SYSID}"
python3 ${src_path}/Tools/sitl_gazebo/scripts/xacro.py ${src_path}/Tools/sitl_gazebo/models/rotors_description/urdf/${PX4_SIM_MODEL}_base.xacro \
    rotors_description_dir:=${src_path}/Tools/sitl_gazebo/models/rotors_description \
    mavlink_tcp_port:=${PX4_SITL_PORT}  \
    -o /tmp/${PX4_SIM_MODEL}_${PX4_SYSID}.urdf

gz sdf -p  /tmp/${PX4_SIM_MODEL}_${PX4_SYSID}.urdf > /tmp/${PX4_SIM_MODEL}_${PX4_SYSID}.sdf

echo "Model TCP Port set to: ${PX4_SITL_PORT}"
echo "Specific SDF saved to /tmp/${PX4_SIM_MODEL}_${PX4_SYSID}.sdf"