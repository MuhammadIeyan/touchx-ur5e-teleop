#!/usr/bin/env python3
"""Read-only preflight. Does not open the serial port or initialize haptic motors."""
import hashlib
import os
import platform
import shutil
import stat
from pathlib import Path

failures = 0
def check(ok, description):
    global failures
    print(('PASS' if ok else 'CHECK') + ': ' + description)
    failures += not ok

check(platform.machine() == 'x86_64', 'tested architecture: x86_64')
print('INFO: host kernel ' + platform.release())
for name in ('gcc', 'g++', 'udevadm', 'flock', 'sha256sum'):
    check(shutil.which(name) is not None, 'command available: ' + name)
for filename, expected in {
    '/usr/lib/libPhantomIOLib42.so': '2d207a24f59716d7335a539ca6e5c04d709c156874c45522cf2ae1602d928b42',
    '/usr/lib/libHD.so.3.4.0': '82605a5df5284f81c1de4230b521b99a0663cd0fe388ab5889af4d0252f9f914',
}.items():
    p = Path(filename)
    digest = hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else 'missing'
    check(digest == expected, filename + ' SHA-256: ' + digest)
check(Path('/usr/include/HD/hd.h').is_file(), 'OpenHaptics HD headers installed')
check(Path('/usr/lib/libHD.so').is_file(), 'linker can find /usr/lib/libHD.so')
port = Path('/dev/ttyACM0')
if port.exists():
    s = port.stat()
    check(stat.S_ISCHR(s.st_mode) and os.major(s.st_rdev)==166 and os.minor(s.st_rdev)==0,
          'supported device node: ttyACM0, major/minor 166:0')
    check(os.access(port, os.R_OK | os.W_OK), 'current user can read/write ttyACM0')
    usb = Path('/sys/class/tty/ttyACM0/device').resolve()
    ids = next(((p/'idVendor', p/'idProduct') for p in (usb, *usb.parents)
                if (p/'idVendor').is_file() and (p/'idProduct').is_file()), None)
    check(ids is not None and tuple(p.read_text().strip() for p in ids)==('2988','0303'),
          'ttyACM0 belongs to USB 2988:0303 Touch X')
else:
    check(False, 'Touch X must be connected as /dev/ttyACM0; other ACM indices are not supported by this interposer')
check(not os.environ.get('LD_PRELOAD'), 'no inherited LD_PRELOAD')
profile = Path.home()/'.3dsystems/config/Default Device.config'
check(profile.is_file(), 'current user has a Touch Setup Default Device profile')
runtime = Path(__file__).resolve().parents[1]/'diagnostics/touch-x/runtime/config'
if (runtime/'Default Device.config').exists():
    print('INFO: local runtime profile already exists; verify it belongs to the connected device. It is not overwritten automatically.')
print('Preflight only; perform the force-disabled movement/calibration test next.')
raise SystemExit(1 if failures else 0)
