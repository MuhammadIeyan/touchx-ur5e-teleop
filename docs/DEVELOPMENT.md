# Development

## Structure and local data

Native Touch code is in `drivers/touch_x/`; Python entry points and shared/simulation modules remain under `scripts/`; physical modules are grouped under `scripts/real_robot/`. ROS plugins are independent CMake/ament packages under `ros2/`. Keep hardware and simulation domain selection explicit in launch wrappers.

Do not commit `diagnostics/`, device profiles, robot calibration, commissioning records, generated build/install trees, credentials or ROS bags. The uncommissioned example is suitable for Git; your working commissioning record is not. Historical source snapshots and session reports are retained locally in `diagnostics/organization-20261124/`.

## Hardware-free tests

```bash
source /opt/ros/humble/setup.bash
./scripts/test.sh
```

Tests use the nominal model from the installed `ur_description`, not the local robot calibration. Python tests mock ROS interfaces and do not activate hardware. The C++ force-safety test is independent of the vendor device SDK.

For the controller activation regression:

```bash
bash scripts/build_ur_clock_controller.sh
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  build/ur5e_clock_fixed_controller/clock_fixed_offline_check
```

That executable uses memory-backed interfaces, not a robot driver. The `vendor_offline_check` executable reproduces the installed UR 2.16 clock exception and is expected to fail on the affected version.

Optional local replay (requires an ignored recording and your local calibration/manifest):

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 tests/real_robot/replay_planner.py PATH_TO_RECORD.jsonl --output /tmp/replay.json
```

## Changes to control behavior

Preserve release-to-arm, finite endpoints, cancellation of late action acceptance, original-snapshot freshness checks, generation-based invalidation, and the independent guard. Never treat a passing unit test or a nominal model as proof of physical accuracy, collision freedom or surgical suitability.

## Optional GitHub Actions

`ci/github-actions-tests.yml.example` contains a hardware-free CI job. It is a template, not an enabled workflow. To enable it, add it as `.github/workflows/tests.yml` using GitHub permissions that allow workflow changes. No CI runner ever needs access to the robot LAN, Touch USB or local commissioning records.
