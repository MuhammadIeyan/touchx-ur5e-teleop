# Local robot calibration

Place your robot-specific extracted calibration at `ur5e_calibration.yaml` in this directory. YAML files here are deliberately ignored by Git. Obtain calibration with the official `ur_calibration` tooling and record its hash in your local commissioning configuration. Do not substitute the nominal model for physical operation.

Tests use the nominal model distributed by `ur_description`, independently of this directory.
