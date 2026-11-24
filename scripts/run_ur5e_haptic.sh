#!/usr/bin/env bash
# Continuous haptic teleoperation for the physical UR5e (domain 43, localhost only).
#
# Usage:
#   ./scripts/run_ur5e_haptic.sh [--budget-mm 100] [--scale 0.75] [--timeout-min 10]
#
# The guard supervises the rolling position controller. Release cancels motion
# and permits repositioning; Ctrl-C/child loss deactivates the controller.
# The workspace origin remains fixed throughout the session (no auto-rebasing).
set -eo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "$root_dir/scripts/run_ur5e_guard_test.sh" --segmented "$@"
