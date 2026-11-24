# Native Touch X interface

- `touch_probe.cpp`: OpenHaptics reader and opt-in simulation force servo.
- `force_safety.hpp`: force magnitude, slew, energy, button and freshness safeguards.
- `pace_serial.c`: scoped USB-write spacing and calibration compatibility shim.
- `test_calibration_compat.cpp`: optional SDK compatibility test.

Build/run through `scripts/run_touch_x.sh`. The launcher verifies the tested vendor library versions and device identity. Runtime files stay under ignored `diagnostics/touch-x/`. Vendor SDK/driver binaries are not included. See `docs/TOUCH_X_UBUNTU_SETUP.md`.
