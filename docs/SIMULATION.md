# Simulation and force feedback

Simulation uses ROS domain 42; physical operation stays separate on domain 43.

## Run

Close Touch Setup/Diagnostic. Connect Touch X USB and power. Build the local contact plugin after changing its C++ source:

```bash
./scripts/build_sim_feedback.sh
./scripts/run_touch_x_sim.sh
```

Also rebuild after updating the MuJoCo/ROS packages, with the simulation stopped. An outdated plugin binary can appear loaded while `/simulation/tool_contact` has no publisher, leaving teleop at `touch_graph=False`. The build script performs a clean rebuild to avoid stale objects after package updates. Before pressing the button, wait for `READY: Simulation teleop active`.

The default starts the MuJoCo viewer and read-only Touch tracking. Enable simulated motion explicitly:

```bash
./scripts/run_touch_x_sim.sh execute:=true
```

Release the single stylus button before engaging. Hold it to move; release to hold the simulated arm and reposition the Touch. Each press captures a new relative position/orientation reference. Default translation scale is 0.2; rotation scale is 1.0. Touch commands are limited to 15 mm/s Cartesian speed and 0.1 rad/s joint speed. RCM is experimental and off by default (`rcm:=true` opts in).

After a coordinated low-force check, opt into Touch motor output:

```bash
./scripts/run_touch_x_sim.sh execute:=true force_feedback:=true
```

For a stronger/faster **simulation-only** preset, stop the previous launch first and run:

```bash
./scripts/run_touch_x_sim_responsive.sh
```

| Setting | Conservative default | Responsive preset |
|---|---:|---:|
| Translation scale | 0.20 | 0.35 |
| Additional force gain | 1.0 | 2.0 |
| Touch force cap | 0.5 N | 0.8 N |
| Cartesian speed limit | 15 mm/s | 30 mm/s |
| Joint speed limit | 0.1 rad/s | 0.2 rad/s |

These are command limits, not guaranteed measured motion speeds. The original launch retains its conservative defaults. For explicit tuning, the launch arguments are `scale`, `force_gain`, `force_limit` (N), `max_linear_speed` (m/s), and `max_joint_speed` (rad/s). The servo rejects force limits above **1.0 N**. The responsive preset automatically opts into simulation motion and Touch forces; it still uses domain 42 only.

Force gain 1 uses the power-conjugate mapping. Gain 2 intentionally exaggerates feedback for perception; it is not a calibrated representation of tissue force. The 2 N/s ramp and 5 mJ energy budget are unchanged, so actual commanded output can be below the cap. No contact still means no contact feedback. These settings are not approved surgical parameters; use finer motion for delicate tasks and evaluate precision, overshoot, collision forces and delay before choosing task-specific settings.

`headless:=true` runs without a viewer. Never run multiple stacks concurrently. Stop with Ctrl-C. A driver communication stall requires disconnecting USB and power before retrying. The supported standalone reader is `scripts/run_touch_x.sh`; it defaults to forces disabled.

## RViz view

Stop the previous simulation stack, then run:

```bash
./scripts/run_touch_x_rviz.sh
# With bounded Touch motor output from MuJoCo contacts:
./scripts/run_touch_x_rviz.sh force_feedback:=true
```

Choose one command. This launcher starts headless MuJoCo, RViz, Touch input and simulated motion on domain 42. It does not start a physical driver. Wait for READY, release the button once, then hold it to move. Forces are off by default. Ctrl+C in the launch terminal stops the stack; closing RViz alone only closes the view.

RViz displays `/robot_description` and TF for the robot, and `/simulation/scene` for the grasper, tip and environment. The read-only `sim_rviz_scene.py` computes marker poses from the same MuJoCo model and `/joint_states`; it has no command or force publisher. The simulation URDF world rotation matches Menagerie. Its rounded nominal link dimensions agree within 2 mm in the regression poses; this is a visualization, not physical TCP calibration. MuJoCo still computes dynamics and contacts; RViz is the viewer.

For the responsive preset with RViz instead of the MuJoCo window:

```bash
./scripts/run_touch_x_sim_responsive.sh rviz:=true headless:=true
```

This preset enables Touch force output. Do not run it alongside another stack.

## Feedback and interlocks

The local `sim_contact_feedback` plugin reads solved contact forces from the running MuJoCo snapshot at 100 Hz. A 4 mm collision sphere at `instrument_tip` interacts with the environment. The visual grasper mesh is not a validated collision model of the complete instrument; tissue is a rigid ellipsoid with compliant contacts, not a validated deformable tissue model.

Contact force is expressed in world coordinates and mapped using `F_touch = force_gain * scale * M.T * F_world`, with the same mapping and scale as translation. Touch renders XYZ forces only, not torques. The previous independent geometric tissue/barrier force approximation was removed.

The device servo independently enforces:

- Explicit force opt-in, valid calibration, recent USB reception, and the actual single button held.
- Release before initial engagement and after a fault/calibration change.
- Original command age below 100 ms, finite values, and advancing command sequence.
- Configured magnitude cap (default 0.5 N, responsive 0.8 N, absolute maximum 1.0 N) and 2 N/s force-change limit. Release or fault zeros immediately.
- A 5 mJ energy tank limiting net commanded work; not replenished by clutching. This conservative sampled-data safeguard is not a formal proof of stability for every driver/hand interaction.
- OpenHaptics force ramping, force, velocity, and impulse protections enabled.

Servo input uses a nonblocking handoff. ROS writes to the device pipe without blocking. Stamped force messages reject stale or unexpected publishers; paused simulation snapshots do not refresh contact timestamps. `/touch_x/applied_force` reports the **commanded** servo force, not a measured force-sensor reading.

The simulation controller checks the model, ROS graph, heartbeat, fresh joints and Touch state; the command router independently expires its deadman lease. Domain isolation is a deployment guard, not authentication against malicious publishers.

