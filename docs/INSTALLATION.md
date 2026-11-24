# Installation

## Platform and external dependencies

Use Ubuntu 22.04, ROS 2 Humble and Python 3.10. Source `/opt/ros/humble/setup.bash` before development commands. Required components:

- ROS packages: `ur_robot_driver`, `ur_description`, `ur_moveit_config`, `ur_calibration`, `xacro`, their ROS control/message dependencies.
- Simulation: installed `mujoco_ros2_control`, `mujoco_vendor` and supporting plugins. Local simulation plugins build against these installed packages, not an archived source checkout.
- Python: NumPy, SciPy, PyYAML and pytest (Ubuntu packages are suitable).
- Build tools: GCC/G++, CMake, make, pkg-config and Git.
- Touch X: separately obtain/install the vendor Touch Device Driver and OpenHaptics SDK. Vendor binaries, licenses and device profiles are not redistributed here. Follow [the device guide](TOUCH_X_UBUNTU_SETUP.md).

The local clock correction vendors the scaled controller source from UR ROS 2 driver **2.16.0** and builds against the installed ROS libraries. That implementation was tested with `joint_trajectory_controller` 2.54.1; an upstream/ABI change requires rebuilding and rechecking the C++ harness. Do not silently assume compatibility with another ROS distribution.

## Build

From the repository root:

```bash
./scripts/build_sim_feedback.sh
bash scripts/build_ur_clock_controller.sh
```

The Touch launcher builds its reader and USB shim on demand. Simulation and physical entry points source their own domain settings; do not bridge domains 42 and 43.

## Local configuration

Simulation does not need a real robot calibration file. Physical use needs:

- `calibration/ur5e_calibration.yaml`, extracted from the intended robot.
- `config/real_robot_commissioning.json`, reviewed for that robot and tool.

`config/examples/real_robot_commissioning.example.json` is a schema/example with checks deliberately false. Copying it does not commission a robot. The physical adapter currently has bench-specific identity/TCP requirements; see the commissioning guide before adapting it.

Runtime directories under `diagnostics/` and generated `build/`/`install/` directories are created as needed and are not distributed in Git. Never copy another Touch's device profile as a substitute for configuring your own device.
