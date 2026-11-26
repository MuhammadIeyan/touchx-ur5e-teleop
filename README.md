# Touch X / UR5e Teleoperation

ROS 2 Humble teleoperation using a Touch X haptic device, with separate MuJoCo simulation and physical UR5e backends. Developed on Ubuntu 22.04.

**Experimental bench software, not a validated surgical system.** Physical RCM and force feedback are disabled. Robot-specific commissioning is required before physical use.

## Start here

- [Installation and dependencies](docs/INSTALLATION.md)
- [Architecture and data flow](docs/ARCHITECTURE.md)
- [Touch X Linux setup and recovery](docs/TOUCH_X_UBUNTU_SETUP.md)
- [Simulation and force feedback](docs/SIMULATION.md)
- [Physical setup](docs/REAL_ROBOT_COMMISSIONING.md)
- [Continuous physical control](docs/SMOOTH_PHYSICAL_CONTROL.md)
- [Development and tests](docs/DEVELOPMENT.md)
- [Third-party sources and licensing](THIRD_PARTY_NOTICES.md)

## Simulation — ROS domain 42

After installing the dependencies and configuring your Touch X:

```bash
./scripts/build_sim_feedback.sh
./scripts/run_touch_x_rviz.sh                        # RViz + headless physics, simulated motion
./scripts/run_touch_x_sim.sh                         # tracking only
./scripts/run_touch_x_sim.sh execute:=true           # simulated motion
./scripts/run_touch_x_sim.sh execute:=true force_feedback:=true
```

The last command enables Touch motor output from simulated contact forces. Close the previous launch before starting another. The responsive simulation preset is documented in the simulation guide.

## Physical robot — ROS domain 43

First complete [commissioning](docs/REAL_ROBOT_COMMISSIONING.md). Calibration and the commissioning record are local files deliberately excluded from Git. The example configuration is uncommissioned and cannot authorize motion.

In separate terminals:

```bash
./scripts/run_ur5e_observe_driver.sh ROBOT_IPV4
./scripts/run_touch_x_real_readonly.sh
```

With the workspace clear and pendant Stop available, start the intended External Control-only program, keep the Touch button released, then launch:

```bash
./scripts/run_ur5e_haptic.sh --budget-mm 100 --scale 0.75
```

Release once, hold the single button to engage, move gently, and release to cancel/reposition. The workspace stays centered on the starting pose; it does not reset on each press. The older `run_ur5e_touch_50mm.sh` entry point retains a 50 mm radius and 0.5 scale.

Both launchers accept `--speed-mm-s 15` to adjust trajectory timing and the nominal Cartesian speed ceiling (default 12; range 1–50 mm/s). This is separate from displacement `--scale`. Adjustable speed has offline test coverage only; see [physical control limits](docs/SMOOTH_PHYSICAL_CONTROL.md).

The driver starts motion controllers inactive. The command launcher verifies calibrated hardware, payload, current state and the local clock-corrected trajectory plugin before activation. No launcher presses pendant Play, powers the robot or releases brakes.

## Project layout

```text
calibration/              Local robot calibration (ignored), instructions
config/                  Simulation configuration and uncommissioned examples
description/            Robot/tool description files
drivers/touch_x/        C++ SDK reader, USB timing shim, force safeguards
launch/                 ROS 2 simulation launch files
models/                 MuJoCo UR5e and tool assets, upstream licenses
ros2/
   sim_contact_feedback/ MuJoCo contact-feedback plugin
   ur5e_clock_fixed_controller/ Project-local UR controller clock correction
scripts/                Launch/build commands and shared/simulation modules
   real_robot/           Planning, continuous control, watchdog, observation
tests/                  Hardware-free regression tests and replay tools
docs/                   Setup, architecture and operating guides
diagnostics/            Local recordings/history (ignored)
build/, install/        Generated build products (ignored)
```

The simulation and physical stacks use separate ROS domains (42 and 43), with localhost-only DDS. They are not bridged. MuJoCo is not in the physical command path.

## Current validation

Smooth physical free-space motion was reported during bench testing. The later process-isolated planner and larger-motion preset have offline regression/replay coverage; they still require a supervised physical check. Software tests and a smooth response do not establish surgical accuracy or safety.
