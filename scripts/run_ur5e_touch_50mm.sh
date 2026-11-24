#!/usr/bin/env bash
set -eo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "$root_dir/scripts/run_ur5e_guard_test.sh" --segmented --budget-mm 50 --scale 0.5 "$@"
