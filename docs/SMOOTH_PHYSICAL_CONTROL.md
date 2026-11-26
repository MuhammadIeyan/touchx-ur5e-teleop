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

Select trajectory speed independently of hand displacement gain:

```bash
./scripts/run_ur5e_touch_50mm.sh --speed-mm-s 15
```

`--speed-mm-s` defaults to 12 and accepts 1–50. Stop the current teleop launcher with Ctrl+C and verify controller deactivation before restarting with a new setting. No rebuild is required. `--scale` changes requested robot displacement per hand displacement; `--budget-mm` changes the workspace radius. Neither is a velocity setting.

The speed argument sets the nominal Cartesian speed ceiling and adjusts the trajectory horizon: 300 ms at 12 mm/s and 180 ms at 20 mm/s, with a 150 ms minimum from 24 through 50 mm/s. Raising the ceiling above 24 therefore does not further shorten the horizon or guarantee faster tracking; the 50 mm/s setting is an upper bound, not a commanded constant speed. If the nominal horizon violates joint speed/acceleration limits, the planner tries longer horizons up to 500 ms while preserving incoming velocity/acceleration and the endpoint. Every candidate must pass the unchanged motion bounds, and the chosen curve must pass geometry checks. If no valid curve exists, control still stops. Planned and transmitted durations match. Endpoint advance stays at most 1.5 mm, and outstanding planned motion stays at most 2 mm. Actual speed can be lower because of joint limits, controller speed scaling, planning latency and repeated endpoint blending. A 10× ceiling (120 mm/s) is rejected; it has not been commissioned.

- Workspace radius: `--budget-mm`; general default 100 mm. Local parser accepts 10–5000 mm, but geometric checks still apply; this range is not a validated physical workspace.
- Translation ratio: `--scale`; general launcher default 0.75. Local parser accepts 0.1–100; larger gains are not commissioned by the speed change.
- Nominal speed ceiling: `--speed-mm-s`, 1–50 mm/s, default 12; joint speed 0.08 rad/s and acceleration 0.8 rad/s².
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

Local evidence is written under `diagnostics/real-robot/` and is not part of the public repository. Smooth physical response was reported for the interpolation/clock correction; the process-worker update, larger preset and adjustable speed still require a supervised physical recheck. Offline tests cover speed rejection, curve timing/bounds, faster repeated tracking and release/re-engagement; they do not establish physical stopping performance.
