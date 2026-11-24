#!/usr/bin/env bash
# Starts official UR driver with joint motion controllers inactive, no dashboard client.
set -eo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# != 1 ]]; then
    echo "Usage: $0 ROBOT_IPV4 (observation only; no program start or brake release)" >&2
    exit 2
fi
if [[ ! -f "$root_dir/calibration/ur5e_calibration.yaml" ]]; then
    echo 'Missing local robot calibration; see docs/REAL_ROBOT_COMMISSIONING.md.' >&2
    exit 2
fi
python3 - "$1" <<'PY'
import ipaddress,sys
ip=ipaddress.IPv4Address(sys.argv[1])
if ip.is_unspecified or ip.is_multicast or ip.is_loopback:raise SystemExit('Provide the robot LAN IPv4 address')
PY
source /opt/ros/humble/setup.bash
if [[ ! -f "$root_dir/install/ur5e_clock_fixed_controller/share/ur5e_clock_fixed_controller/local_setup.bash" ]]; then
    bash "$root_dir/scripts/build_ur_clock_controller.sh"
fi
source "$root_dir/install/ur5e_clock_fixed_controller/share/ur5e_clock_fixed_controller/local_setup.bash"
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$root_dir/diagnostics/real-robot/ros-logs"
mkdir -p "$ROS_LOG_DIR"
controller_config="$root_dir/diagnostics/real-robot/controllers-blended.yaml"
python3 "$root_dir/scripts/real_robot/controller_config.py" "$controller_config"
exec ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:="$1" \
    controllers_file:="$controller_config" \
    kinematics_params_file:="$root_dir/calibration/ur5e_calibration.yaml" \
    activate_joint_controller:=false headless_mode:=false launch_dashboard_client:=false \
    launch_rviz:=false use_fake_hardware:=false verify_robot_model:=true
