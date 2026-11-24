#!/usr/bin/env bash
# Opt-in stronger/faster simulation preset. Never starts a physical robot driver.
set -euo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "$root_dir/scripts/run_touch_x_sim.sh" execute:=true force_feedback:=true \
    scale:=0.35 force_gain:=2.0 force_limit:=0.8 \
    max_linear_speed:=0.03 max_joint_speed:=0.2 "$@"
