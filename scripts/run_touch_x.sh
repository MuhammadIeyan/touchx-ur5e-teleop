#!/usr/bin/env bash
# Touch X reader; force-disabled unless explicitly opted in on simulation domain 42.
set -euo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --help ]]; then
    echo "Usage: $0 [duration_seconds (default 300)]"
    echo "Builds and runs a 1000 Hz Touch X reader, forces off by default. No robot connection."
    echo "Close Touch Setup/Diagnostic first. Slowly sweep the arm in all three axes to calibrate."
    exit 0
fi
duration=${1:-300}
if [[ ! $duration =~ ^[0-9]+$ ]] || (( duration < 1 || duration > 3600 )); then
    echo 'Duration must be 1..3600 seconds.' >&2; exit 2
fi
if [[ -n ${LD_PRELOAD:-} ]]; then
    echo 'Unset LD_PRELOAD before running this isolated reader.' >&2; exit 2
fi
exec 9>"/tmp/touch-x-${UID}.lock"
if ! flock -n 9; then
    echo 'Another Touch X reader from this launcher is already running.' >&2; exit 2
fi
expected=2d207a24f59716d7335a539ca6e5c04d709c156874c45522cf2ae1602d928b42
actual=$(sha256sum /usr/lib/libPhantomIOLib42.so)
if [[ ${actual%% *} != "$expected" ]]; then
    echo 'This workaround was tested with Touch Driver 2023-11-15; installed library differs.' >&2
    exit 2
fi
sdk_hash=$(sha256sum /usr/lib/libHD.so.3.4.0)
if [[ ${sdk_hash%% *} != 82605a5df5284f81c1de4230b521b99a0663cd0fe388ab5889af4d0252f9f914 ]]; then
    echo 'Installed OpenHaptics SDK differs from the tested calibration-compatible version.' >&2; exit 2
fi
properties=$(udevadm info -q property -n /dev/ttyACM0)
if ! printf '%s\n' "$properties" | grep -qx ID_VENDOR_ID=2988 ||
   ! printf '%s\n' "$properties" | grep -qx ID_MODEL_ID=0303; then
    echo '/dev/ttyACM0 is not the tested 2988:0303 Touch X USB device.' >&2; exit 2
fi
if [[ ! -r /dev/ttyACM0 || ! -w /dev/ttyACM0 ]]; then
    echo 'Read/write access to /dev/ttyACM0 is required (normally via dialout).' >&2; exit 2
fi
build_dir="$root_dir/diagnostics/touch-x/build"
mkdir -p "$build_dir"
runtime_dir="$root_dir/diagnostics/touch-x/runtime"
mkdir -p "$runtime_dir/config"
# The 2023 driver reads calibration from config/config but writes to config.
# Keep the workaround and a copy of the selected profile inside this workspace.
for name in 'Default Device.config' 'Device Calibration (Default Device)'; do
    if [[ ! -e "$runtime_dir/config/$name" && -f "$HOME/.3dsystems/config/$name" ]]; then
        cp -- "$HOME/.3dsystems/config/$name" "$runtime_dir/config/$name"
    fi
done
if [[ ! -f "$runtime_dir/config/Default Device.config" ]]; then
    echo 'Configure Default Device with Touch Setup before running this reader.' >&2; exit 2
fi
# Temperature data, when present, must match this profile rather than a fixed device.
profile_serial=$(sed -n 's/^DeviceSerialNumber=//p' "$runtime_dir/config/Default Device.config" | tr -d '\r')
if [[ $profile_serial =~ ^[0-9]+$ ]]; then
    temperature_name="Device Temperature ($profile_serial)"
    if [[ ! -e "$runtime_dir/config/$temperature_name" && -f "$HOME/.3dsystems/config/$temperature_name" ]]; then
        cp -- "$HOME/.3dsystems/config/$temperature_name" "$runtime_dir/config/$temperature_name"
    fi
fi
if [[ ! -e "$runtime_dir/config/config" ]]; then
    ln -s . "$runtime_dir/config/config"
elif [[ ! -L "$runtime_dir/config/config" || $(readlink "$runtime_dir/config/config") != . ]]; then
    echo 'Unexpected calibration path; inspect diagnostics/touch-x/runtime/config/config.' >&2; exit 2
fi
gcc -shared -fPIC -O2 -Wall -Wextra "$root_dir/drivers/touch_x/pace_serial.c" \
    -ldl -pthread -o "$build_dir/pace_serial.so"
g++ -std=gnu++17 -O2 -Wall -Wextra -Wno-attributes -pthread \
    "$root_dir/drivers/touch_x/touch_probe.cpp" -lHD -o "$build_dir/touch_probe"
echo 'Using 100 us minimum spacing between Touch X USB protocol writes.'
# Pin the installed, validated driver even if a shell has an alternate search path.
export LD_LIBRARY_PATH=/usr/lib:/usr/lib/x86_64-linux-gnu
export LD_PRELOAD="$build_dir/pace_serial.so"
export TOUCH_WRITE_GAP_US=100
export TOUCH_CALIBRATION_COMPAT=1
export GTDD_HOME="$runtime_dir"
exec "$build_dir/touch_probe" 1000 "$duration"
