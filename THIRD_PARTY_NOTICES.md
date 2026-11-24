# Third-party sources and licensing

This repository includes multiple components with different provenance. No blanket project license has been selected; public availability alone does not grant a license to unlicensed portions.

| Component | Origin / terms |
|---|---|
| UR5e model meshes and base model | MuJoCo Menagerie / Universal Robots model; preserve `models/mujoco_menagerie/universal_robots_ur5e/LICENSE` (BSD-3-Clause, ROS Industrial Consortium). Local scene/tool changes are separate. |
| Clock-corrected UR controller | UniversalRobots/Universal_Robots_ROS2_Driver, tag 2.16.0, `ur_controllers`; BSD notices retained in source/header. See `ros2/ur5e_clock_fixed_controller/UPSTREAM.md` for the narrow changes. |
| Kinematic-chain adaptation | `scripts/real_robot/bench_motion.py` credits the user-supplied MuhammadIeyan/UR5e_Teleop archive. The supplied archive had no project-wide LICENSE; no broader license is asserted here. |
| Grasper STL | User-supplied `bowel_grasper_v1.stl`; no separate asset license was supplied. Do not assume the robot-mesh license applies to it. |
| OpenHaptics and Touch drivers | External proprietary dependencies; obtain separately under the vendor's terms. Not redistributed. |
| MuJoCo ROS 2 control | External installed dependency; its old source snapshot is retained locally outside Git and is not needed by the launch path. |

The source/asset notices above remain applicable independently of any license later chosen for original project code. See upstream projects for their complete notices: [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), [UR ROS 2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver), [referenced teleoperation archive](https://github.com/MuhammadIeyan/UR5e_Teleop).
