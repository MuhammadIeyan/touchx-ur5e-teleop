# Continuous physical control

## Run on a commissioned bench

Keep the UR driver and read-only Touch publisher running on domain 43. The driver must use `run_ur5e_observe_driver.sh` so the local clock-corrected controller is loaded. At the pendant, use the intended External Control-only program with the configured computer IP.

With the workspace clear, pendant Stop available and the Touch button released:

```bash
./scripts/run_ur5e_haptic.sh --budget-mm 100 --scale 0.75
```

The general launcher defaults to these values. Ten millimetres of hand displacement requests 7.5 mm of robot displacement. The 100 mm limit is a fixed sphere radius around the starting pose, not travel per button press. Geometry/joint checks can stop motion before this radius is reached.

For a smaller profile:

```bash
./scripts/run_ur5e_touch_50mm.sh
# or
./scripts/run_ur5e_haptic.sh --budget-mm 10 --scale 0.5
```

Hold for 0.25 s to engage; continue holding to track; release to cancel and reposition. Robot orientation stays fixed. Ctrl+C requests shutdown and controller deactivation. No daily operator JSON or setup script is required by continuous control; legacy isolated fault tests retain their own session records.

## Limits and timing

- Workspace radius: `--budget-mm`, 10–200 mm; general default 100 mm.
- Translation ratio: `--scale`, 0.1–1.0; general default 0.75.
- Nominal speed cap remains 12 mm/s; joint speed 0.08 rad/s and acceleration 0.8 rad/s².
- Nominal outstanding motion bound remains 2 mm, with endpoint advance <=1.5 mm.
- Session duration defaults to ten minutes; `--timeout-min` accepts up to 60.
- Physical force feedback and RCM remain disabled.

These are software planning limits, not certified physical stopping distances. The actual trajectory starts from the controller's state at receipt, later than the planning snapshot.

The planner runs in a separate process. Late results are discarded, not sent. Release/re-engagement invalidates all prior-hold results. A completion notification wakes the control loop without waiting for its next 20 ms timer tick. Feedback, cancellation and watchdog servicing remain in the parent. Three expired plans or a >500 ms planning stall stop the session. The 100 ms input/reference freshness cutoff is unchanged.

## Troubleshooting

- **Waiting for calibrated hardware:** inspect the UR driver terminal and controller inventory.
- **Controller parameter timeout:** the controller may not have loaded; inspect parameter types and plugin installation.
- **Activation clock bug:** restart the driver with this project's launcher to select the local plugin.
- **Stale input/feedback:** the message identifies joint, TCP or Touch streams. A dead UR driver can remove robot feedback while Touch continues.
- **Planning deadlines missed:** inspect `planning_timing` and `plan_discarded` records; do not increase freshness limits to hide stalled planning.
- **Workspace boundary:** move the hand inward. Releasing does not relocate the boundary.
- **Proximity/geometry fault:** stop and inspect the posture and surrounding clearance. Do not disable the pair check to continue.

Local evidence is written under `diagnostics/real-robot/` and is not part of the public repository. Smooth physical response was reported for the interpolation/clock correction; the process-worker update and larger preset still require a supervised physical recheck.
