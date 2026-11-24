# Touch X on Ubuntu: diagnosis, reproducible setup and recovery

Prepared 16 November 2026. This guide reproduces the solution tested on this computer. It is a workaround for one tested USB Touch X / Linux driver / SDK combination, not a universal firmware fix. Start with tracking only on every new computer or haptic unit.

## 1. Why it originally failed

There were **three distinct problems**. Installing another driver alone did not address all three.

| Problem | Evidence from this machine | Solution |
|---|---|---|
| Timing-sensitive USB serial communication | The original reader hung after about 4.4 seconds. Another continuous reader reported `HD_COMM_ERROR` 770, internal `-28`. A traced run lasted 30 seconds; the untraced moving run failed again. | Delay closely spaced Touch protocol writes so they have at least **100 microseconds** spacing. |
| Calibration file loaded from the wrong path | Driver logs showed reads from `config/config/Device Calibration (Default Device)` while writes went to `config/Device Calibration (Default Device)`. | Use an isolated runtime directory with `config/config -> .`, so both paths resolve to the same file. |
| Driver/SDK calibration return-code mismatch | Inspection of the installed libraries showed the driver returning `1` on a successful update while this SDK treated nonzero as an error. Calibration became OK but the application received error 258/internal 1. | A narrowly scoped compatibility handler accepts only that exact automatic-calibration result after independently verifying calibration OK with force output disabled. |

The first finding is a **timing diagnosis supported by controlled observations**, not proof of a specific firmware buffer overflow, USB-controller defect, or exact required timing threshold. Tracing perturbs timing; deliberately spacing writes reproduced the improvement without tracing. We did not establish why Windows avoided the problem internally. Your Windows test supports the conclusion that the device was functional, but does not identify the Linux defect by itself.

The isolated 2024 driver test opened the serial port but failed to obtain the firmware number and initialized with error 770/internal -28. Its deeper cause was not established. The successfully tested baseline was the 2023 driver plus the fixes above. Do not interpret this as proof that every 2024 driver or every Ubuntu host fails.

Things that were **not** established as causes: permissions (already correct here), autosuspend (already disabled here), or a leftover 2024 manager library (not loaded by the tested 2023 driver). A requested 500 Hz scheduler rate did not solve it: observed callbacks remained near 1024 Hz. A 100 Hz request was rejected. The solution was spacing protocol bursts, not simply lowering the requested servo rate.

## 2. What the solution stack does

```text
Touch X USB 2988:0303, ttyACM0
    ↕
2023 vendor driver + OpenHaptics 3.4.0
    ↕  process-local write pacing (pace_serial.c)
    ↕  isolated configuration + calibration compatibility handler
continuous C++ reader (touch_probe.cpp)
    ↓  position, orientation, button, calibration, timestamps
optional ROS 2 publisher → clutch controller → MuJoCo UR5e (domain 42)
```

The pacing library intercepts `write()` only for packets beginning `a5 5a` sent to character device **166:0** (`ttyACM0`). It preserves packet contents and driver error checks. It also timestamps successful serial reads so later consumers can reject stale transport data. Normal traffic spaced about 1 ms apart does not need the added delay.

`run_touch_x.sh` builds the source, checks the exact library hashes and USB identity, takes an exclusive reader lock, selects the tested driver search path, and sets `LD_PRELOAD` and `GTDD_HOME` only for the launched process. It does not patch a vendor library or globally configure preloading. Vendor Setup/Diagnostic applications do not automatically get this workaround.

The compatibility handler is **not a general “ignore calibration errors” switch**. It requires automatic calibration, `HD_INVALID_OPERATION` (258), internal code 1, disabled force output, and an immediate error-free `HD_CALIBRATION_OK` result. Other errors remain failures. Current code disables force output before a calibration update.

## 3. Exact tested baseline

| Component | Tested value |
|---|---|
| OS / architecture | Ubuntu 22.04.5, x86_64 |
| Kernel during investigation | `6.8.0-138-generic` |
| USB device | Touch X `2988:0303`, `/dev/ttyACM0` |
| Driver archive | `TouchDriver_2023_11_15.tgz` |
| Driver-reported version | `Dev_Oct 30 2023` |
| SDK archive | `openhaptics_3.4-0-developer-edition-amd64.tar.gz` |
| SDK runtime | `/usr/lib/libHD.so.3.4.0` |
| Requested servo rate / pacing | 1000 Hz / minimum 100 µs between matching writes |

Installed-library SHA-256 values, enforced by the launcher:

```text
2d207a24f59716d7335a539ca6e5c04d709c156874c45522cf2ae1602d928b42  /usr/lib/libPhantomIOLib42.so
82605a5df5284f81c1de4230b521b99a0663cd0fe388ab5889af4d0252f9f914  /usr/lib/libHD.so.3.4.0
```

SHA-256 values of the installer archives retained on this computer:

```text
5d51bd0f7140112ce3617798b1e754a5b31617763d4f40332b6ba7d2aeb2a239  TouchDriver_2023_11_15.tgz
9068e039a4a4d8c0a25ec672a1c41dfe4e791509d78b81b9a4bfe2fd6e72271f  openhaptics_3.4-0-developer-edition-amd64.tar.gz
```

These are reproducibility fingerprints of the tested files, not publisher signatures. Obtain vendor software from [3D Systems downloads](https://support.3dsystems.com/s/article/Downloads?language=en_US), following its license. The [official OpenHaptics page](https://www.3dsystems.com/haptics-devices/openhaptics) describes Linux support and host real-time performance requirements. A newer “supported” version is not automatically the exact combination tested here. Do not remove the hash checks just to make a different version run.

## 4. Guided setup on another Ubuntu computer

### Step A — Transfer the workaround source

Use `touch-x-recovery-kit.tar.gz` from the project's `diagnostics/transfer/` directory, or copy the source tree. The kit contains the guide, launcher, reader, pacing source, preflight and offline safety/calibration tests. It excludes vendor binaries, device profiles, calibration offsets, logs and ROS/robot software.

On the destination computer, choose a fresh directory:

```bash
mkdir -p "$HOME/touch-x-kit"
tar -xzf touch-x-recovery-kit.tar.gz -C "$HOME/touch-x-kit"
cd "$HOME/touch-x-kit"
sha256sum -c SHA256SUMS
```

The standalone kit can live anywhere. The complete UR5e project currently expects its model/configuration tree at `$HOME/ur5e_teleop`; copying this small kit does not install the robot simulation.

### Step B — Install the tested vendor stack

These are instructions for the **destination machine**, not commands to reinstall the currently working host. On an existing installation, inventory/back up the old version and follow its vendor removal instructions before replacing it.

Install compilation tools and the dependencies listed in the SDK's bundled `README_INSTALL`:

```bash
sudo apt update
sudo apt install build-essential python3 usbutils udev libncurses5-dev freeglut3
```

Extract the two vendor archives in a separate installer directory. The commands below match the inspected installers in the tested archives:

```bash
tar -xzf openhaptics_3.4-0-developer-edition-amd64.tar.gz
tar -xzf TouchDriver_2023_11_15.tgz
cd openhaptics_3.4-0-developer-edition-amd64
sudo ./install
cd ../TouchDriver_2023_11_15
sudo bash ./install_haptic_driver
sudo ldconfig
```

Read and accept the SDK's installer/license prompts yourself. Follow its reboot instruction. These vendor installers copy libraries directly; on the tested host these library files were not tracked by `dpkg`, so package listings alone are insufficient to identify the installed version. Verify the hashes above afterward.

The 2023 driver installer copies the library and udev rule; its setup tools remain in the extracted `bin/` directory. Use `bin/Touch_Setup` there. If a tool cannot start, inspect missing dependencies with `ldd bin/Touch_Setup`; do not substitute arbitrary library versions or symlinks.

### Step C — Verify USB and access

Connect device power and USB, then:

```bash
lsusb -d 2988:0303
ls -l /dev/ttyACM*
udevadm info -q property -n /dev/ttyACM0
id -nG
```

Expected: `ID_VENDOR_ID=2988`, `ID_MODEL_ID=0303`, and user read/write access to `/dev/ttyACM0`.

If the node belongs to `dialout` and your account lacks that group:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in before testing. Do not run the haptic application as root to work around permissions; that also creates configuration under the wrong home directory.

The inspected 2023 installer rule matches USB product `0304`, whereas this Touch X reports `0303`. This was not the original failure on our already-accessible host, but may matter on a fresh machine. If normal tty permissions are still insufficient, a narrowly scoped local rule is:

```bash
printf '%s\n' 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2988", ATTRS{idProduct}=="0303", GROUP="dialout", MODE="0660"' | sudo tee /etc/udev/rules.d/99-touch-x-local.rules
sudo udevadm control --reload-rules
```

Then unplug/reconnect USB and recheck permissions. Do not use broad `chmod 777` rules.

**Port limitation:** this version is deliberately restricted to `ttyACM0`/166:0. If the Touch is `ttyACM1`, stop here. Reconnect with other ACM devices disconnected, or have the device selection and interposer updated together. A symlink named `ttyACM0` does not change the underlying device minor number and will not make the pacing work.

### Step D — Create a profile for the connected device

Run the vendor `Touch_Setup` as your normal user. Configure this Touch X as **Default Device**. Confirm the profile exists:

```bash
test -f "$HOME/.3dsystems/config/Default Device.config"
```

Close Setup, Diagnostic and every other application using the device. If a vendor diagnostic freezes, disconnect **both USB and power**, reconnect, and leave vendor tools closed before using the workaround.

Do not copy calibration offsets from a different Touch X. Even when moving the same physical unit to another computer, configuring/calibrating locally is the reproducible baseline. The launcher copies this account's profile on its first run and selects an optional temperature file using that profile's serial number; the original unit's serial is no longer hardcoded into that selection.

### Step E — Preflight and first force-disabled run

From the kit/project root, in a normal host terminal:

```bash
python3 scripts/check_touch_x_setup.py
env -u LD_PRELOAD TOUCH_ENABLE_FORCE=0 scripts/run_touch_x.sh 60
```

The preflight only reads files/device metadata; it does not initialize the device or turn on forces. It returns a nonzero status for checks needing attention. Device nodes may be hidden inside a sandbox/container; run this check on the actual host.

The launcher creates `diagnostics/touch-x/runtime/config/`, copies the selected profile and existing matching calibration, and creates:

```text
runtime/config/config -> .
```

Slowly move the arm through normal left/right, up/down and in/out travel. Touch X uses automatic calibration, not the Touch/Omni inkwell procedure. Do not force mechanical stops. Move XYZ and press/release its **single** stylus button.

Expected: frame count keeps increasing, XYZ values change when moved, buttons change between 0 and 1, `calibration=20480` once calibrated, and `Closed; result=0` on normal completion. `20481` means calibration needs updating; `20482` means manual movement is required. Successful communication alone does not prove calibrated measurement.

### Step F — Prove stability before adding ROS or forces

```bash
set -o pipefail
env -u LD_PRELOAD TOUCH_ENABLE_FORCE=0 scripts/run_touch_x.sh 300 |& tee touch-tracking-300s.log
```

Pass criteria: five minutes without a communication error, stall or watchdog exit; changing positions and button state; calibration OK after the sweep. A frame counter alone is not enough—verify actual movement. Restart for another short run and confirm calibration loads correctly.

Optional offline checks, requiring the SDK but no device initialization:

```bash
g++ -std=c++17 tests/touch_x/test_force_safety.cpp -o /tmp/test_force_safety
/tmp/test_force_safety
g++ -std=gnu++17 -Wno-attributes -pthread drivers/touch_x/test_calibration_compat.cpp -lHD -o /tmp/test_calibration
/tmp/test_calibration
```

### Step G — Add simulation only after tracking passes

Use the **full project**, with ROS 2 Humble, its Python/MuJoCo dependencies, and the installed `mujoco_ros2_control` stack. Follow the project README to build the local feedback plugin. The small recovery kit does not include those dependencies.

```bash
cd "$HOME/ur5e_teleop"
./scripts/build_sim_feedback.sh
./scripts/run_touch_x_sim.sh
# Stop the first launch before starting another:
./scripts/run_touch_x_sim.sh execute:=true
```

The first launch is read-only tracking. The second enables simulated clutch motion while Touch motor forces remain disabled. Simulation stays on **ROS_DOMAIN_ID=42**; the physical UR5e domain **43 remains inactive**. Only after coordinated low-force commissioning should you opt into `force_feedback:=true`. The default force cap is 0.5 N; the full project also offers an optional responsive preset at 0.8 N, bounded by an absolute 1.0 N servo limit. See the full project README for its settings. Do not put `LD_PRELOAD` in `.bashrc` or inject the timing shim into unrelated robot applications.

## 5. Troubleshooting and clean recovery

| Symptom | Next action |
|---|---|
| No `2988:0303` device | Check power, USB cable/port and enumeration before investigating ROS. |
| Access denied | Check node ownership, current groups and udev rule; log in again after group changes. |
| Hash mismatch | Stop. Identify the actual installed libraries; restore the tested combination or revalidate a new one. |
| Another reader already running | Close the other launcher/vendor application. Do not bypass the lock. |
| Error 770/internal -28, stream stops or watchdog fires | Stop applications; disconnect USB **and power**; reconnect; rerun force-disabled. Keep the log. If it repeats with the matching stack, treat the new host as unvalidated rather than increasing gaps or weakening checks blindly. |
| Calibration never becomes OK | Verify the right Default Device profile, sweep the device normally, inspect the runtime path and symlink. |
| Error 258/internal 1 persists | The narrow handler's conditions were not satisfied. Inspect calibration/error logs; do not suppress all HD errors. |
| Good vendor setup, but launcher uses old profile | The runtime copy is intentionally not overwritten automatically. Reset it as below. |
| Vendor Diagnostic still freezes but launcher works | Expected possibility: vendor tools do not inherit this process-local workaround. |
| Tracking works, ROS does not | Diagnose the ROS graph separately; do not reinstall the device driver immediately. |

To switch haptic units or refresh a changed profile, first stop all readers. Preserve the old runtime rather than mixing profiles:

```bash
if [ -d diagnostics/touch-x/runtime ]; then
  mv diagnostics/touch-x/runtime "diagnostics/touch-x/runtime.saved-$(date +%Y%m%d-%H%M%S)"
fi
```

Configure the correct unit in Touch Setup under your normal account, close Setup, and rerun the force-disabled launcher. It will make a fresh runtime copy. Never use another unit's saved offsets merely because the model name matches.

To undo the workaround, stop its process and stop using the launcher. Its preload and configuration environment are process-local. This does not uninstall the vendor SDK/driver; manage those separately using their installation documentation.

## 6. Evidence and what remains unproven

Original evidence lives in `diagnostics/touch-x/`: `1000hz.log`, `500hz.log`, `2024-isolated.log`, `traced-30s.log`, `untraced-moving-failure.log`, `paced-120s.log`, `paced-300s.log`, `paced-calibration-transition.log`, `calibration-path-test.log`, and the driver/SDK return-code inspection files.

With pacing, a five-minute run completed **306,525 frames** without communication failure. A calibrated moving run completed **122,323 frames over two minutes**. Later live calibration transitions exercised the compatibility handler successfully. On 16 November, the bounded motor test ran for about 74 seconds of streamed data: maximum commanded force **0.15 N**, zero during recorded button releases/invalid calibration, and normal exit. You confirmed the light force disappeared on release. See `diagnostics/review-2026-11-16/README.md` for the separate simulation/force tests.

A new host still needs its own acceptance run. These results do not establish a universal fix, independent hardware force cutoff during a blocked vendor call, or physical-UR5e readiness. The known MuJoCo ROS shutdown crash is a separate simulation-runtime issue, not the original Touch USB failure.
