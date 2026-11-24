# Physical commissioning

The repository separates simulation (ROS domain 42) from physical operation (43). Domain separation prevents accidental mixing in this deployment; it is not authentication against malicious ROS publishers.

## Required local records

The physical adapters deliberately retain constraints from the commissioned bench. Review code and measurements before adapting them to another robot:

- Extract and verify that robot's calibration; keep it in `calibration/ur5e_calibration.yaml` (ignored by Git).
- Record the robot/computer IPs, External Control host, mounting, measured payload/CoG, and available clearance in `config/real_robot_commissioning.json` (also ignored).
- The template in `config/examples/` starts uncommissioned. Never mark verification flags true without completing the corresponding check.
- `bench_motion.validate_commissioning` currently checks robot IP `192.168.1.103` and domain 43; the driver receives the actual IP explicitly.
- `bench_step.main` checks an active TCP offset of 197 mm along flange Z. This existing bench constraint is not a universal tool dimension.
- The geometry envelope assumes a user-measured minimum clearance of 660 mm for the complete tool and links. This is not inferred from photographs or guaranteed by the program.

The model tool tip and pendant TCP were not fully reconciled for RCM. Physical control therefore keeps orientation fixed and does not enable RCM or contact-force feedback. It must not be described as validated surgery control.

## Startup sequence

1. Verify the mounted assembly, payload, mounting, calibration and clear workspace. Remain near pendant Stop.
2. Start `./scripts/run_ur5e_observe_driver.sh ROBOT_IPV4`. Motion controllers load inactive; this does not press Play or release brakes.
3. Start `./scripts/run_touch_x_real_readonly.sh`. Close vendor Setup/Diagnostic to avoid competing device access.
4. Select the intended External Control-only pendant program with the computer's wired IP; avoid unexpected Move/Home nodes.
5. With Touch released, start the program and follow [continuous control](SMOOTH_PHYSICAL_CONTROL.md).

On another bench, begin with command-free observation and bounded commissioning tests before continuous motion. The scripts named `run_ur5e_bench_*` are explicit legacy commissioning tools, not normal daily startup commands.
