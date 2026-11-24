#!/usr/bin/env python3
"""Package source-only recovery files; exclude vendor binaries and device configuration."""
import hashlib
import io
import tarfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
files = [
    'docs/TOUCH_X_UBUNTU_SETUP.md',
    'scripts/run_touch_x.sh',
    'scripts/check_touch_x_setup.py',
    'drivers/touch_x/touch_probe.cpp',
    'drivers/touch_x/force_safety.hpp',
    'drivers/touch_x/pace_serial.c',
    'drivers/touch_x/test_calibration_compat.cpp',
    'tests/touch_x/test_force_safety.cpp',
]
entries = {name: (root/name).read_bytes() for name in files}
entries['README.md'] = b'''# Touch X Ubuntu recovery kit

Start with docs/TOUCH_X_UBUNTU_SETUP.md.
Verify source integrity with: sha256sum -c SHA256SUMS

This kit contains source and instructions only. Obtain the vendor SDK and driver
separately. Configure/calibrate your own device. No device profiles, offsets,
vendor binaries, ROS stack or robot driver are included.
'''
entries['SHA256SUMS'] = ''.join(
    f'{hashlib.sha256(data).hexdigest()}  {name}\n'
    for name, data in sorted(entries.items())).encode()
out = root/'diagnostics/transfer/touch-x-recovery-kit.tar.gz'
out.parent.mkdir(parents=True, exist_ok=True)
with tarfile.open(out, 'w:gz') as archive:
    for name, data in entries.items():
        info = tarfile.TarInfo(name)
        info.size = len(data)
        info.mode = 0o755 if name.endswith(('.sh', '.py')) else 0o644
        archive.addfile(info, io.BytesIO(data))
out.with_name(out.name+'.sha256').write_text(f'{hashlib.sha256(out.read_bytes()).hexdigest()}  {out.name}\n')
print(out)
