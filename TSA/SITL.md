# SITL test of stick_alt.lua

Not yet run. This is the procedure; record results here once it has been.

## 1. Install (once)

PowerShell as administrator, then reboot and create the Ubuntu user:

```powershell
wsl --install -d Ubuntu
```

Inside Ubuntu. Clone inside WSL (not under `/mnt/c`, which is slow and breaks file modes):

```sh
git clone --recurse-submodules https://github.com/nswgsafety/TSA-Flight-Software ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

## 2. Launch

```sh
cd ~/ardupilot/ArduCopter
mkdir -p scripts
cp ../TSA/scripts/stick_alt.lua scripts/
sim_vehicle.py -v ArduCopter --console --map
```

SITL loads Lua from `scripts/` in the directory it is started from.

## 3. Parameters (MAVProxy prompt)

```
param load ../TSA/params/tsa_base.param
param load ../TSA/params/sitl_overrides.parm
reboot
param load ../TSA/params/tsa_base.param
param load ../TSA/params/sitl_overrides.parm
reboot
```

Loaded twice because `SCR_ENABLE`, `RNGFND1_TYPE` and `FLOW_TYPE` only expose their
sub-parameters after a reboot. `sitl_overrides.parm` swaps the MSP sensors for the simulated
rangefinder and optical flow and turns the simulated GPS off. `BRD_ALT_CONFIG` does not exist
in SITL, so one "unable to find parameter" message is expected.

Confirm before flying:

```
param show RC3_MIN
param show RC3_MAX
param show FS_OPTIONS
```

Console must show `stick_alt: loaded, 0.5-3.0 ft`.

## 4. Fly

```
mode ALT_HOLD
rc 3 1000
arm throttle
rc 3 1700
```

When the rangefinder reads about 0.5 m:

```
rc 3 1500
mode LOITER
mode GUIDED
```

Console must show `stick_alt: engaged at ... m`. If Loiter or Guided is refused with
"Need Position Estimate", right-click the map > Set Origin, and retry.

## 5. Checks that prove throttle maps to altitude

Read height with `status RANGEFINDER` (field `distance`, metres) or the console rangefinder.
Expected values assume `RC3_MIN=1000`, `RC3_MAX=2000`: height = 0.1524 + (pwm - 1000) / 1000 x 0.762 m.

| # | Command | Pass condition |
|---|---|---|
| 1 | `rc 3 1000` | settles at 0.15 m (0.5 ft) +/- 0.03 |
| 2 | `rc 3 1500` | settles at 0.53 m (1.75 ft) +/- 0.03 |
| 3 | `rc 3 2000` | settles at 0.91 m (3.0 ft) +/- 0.03 |
| 4 | `rc 3 1700`, wait 30 s | holds 0.69 m and does not keep climbing (absolute, not climb rate) |
| 5 | `rc 3 1000` then `rc 3 2000` | takes about 2.5 s to arrive (0.3 m/s slew), no overshoot above 1.0 m |
| 6 | `rc 2 1300` for 3 s, then `rc 2 1500` | moves forward, stops, height unchanged |
| 7 | `rc 4 1700` for 3 s, then `rc 4 1500` | yaws right, stops, height and position unchanged |
| 8 | `mode LOITER` with `rc 3 1500` | console shows `stick_alt: released`, height holds |
| 9 | back in GUIDED: `param set SIM_RC_FAIL 1` | mode changes to LAND within a few seconds and it lands. `param set SIM_RC_FAIL 0` after |

Log check (logs are in `ArduCopter/logs/`, open in Mission Planner or `MAVExplorer.py`):

```
graph SALT.Tgt SALT.Alt SALT.Rng
```

`SALT.Tgt` must step between 0.152 and 0.914 with the throttle, with ramps of 0.3 m/s, and
`SALT.Alt` and `SALT.Rng` must follow it.

## Limits of this test

SITL does not model ground effect, the ToF blind zone, floor texture or battery sag, so it
proves the script logic and the failsafe, not the +/- 0.5 in hold.
