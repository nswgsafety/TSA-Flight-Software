# TSA drone bring-up checklist

ArduCopter 4.7.2, Matek H743-Mini V3, MicoAir MTF-01, ELRS. Props OFF until step 6.

Files: `TSA/params/tsa_base.param`, `TSA/scripts/stick_alt.lua`.

## 1. Firmware

No source changes are needed (the feature is a Lua script), so stock firmware works:
`https://firmware.ardupilot.org/Copter/stable-4.7.2/MatekH743/`

To build from this repo instead (WSL2/Ubuntu):

```sh
Tools/environment_install/install-prereqs-ubuntu.sh -y && . ~/.profile
git submodule update --init --recursive
./waf configure --board MatekH743 && ./waf copter
# outputs: build/MatekH743/bin/arducopter_with_bl.hex and arducopter.apj
```

- First flash: hold BOOT while plugging in USB, flash `arducopter_with_bl.hex`
  with STM32CubeProgrammer (USB/DFU).
- Later updates: Mission Planner > Setup > Install Firmware > Load custom firmware > `arducopter.apj`.

## 2. Wiring

| Device | Pads | ArduPilot port |
|---|---|---|
| ELRS receiver | TX6 / RX6 | SERIAL7 |
| MTF-01 | TX4 / RX4 | SERIAL6 |
| 4-in-1 ESC | VBAT, GND, CURR, S1-S4 | motors 1-4 |
| M10Q-5883 compass | SDA / SCL (+ 5V, GND) | I2C, compass only |

- The H743-Mini has no onboard compass. Wire only the M10Q-5883's SDA/SCL for now and keep
  `GPS1_TYPE=0` indoors.
- Outdoor use later: wire the module's GPS TX/RX to a free UART, set `GPS1_TYPE=1` and that
  port's `SERIALx_PROTOCOL=5`. The EKF sources (`EK3_SRC1_*`) would also need changing to use it.

- Check the ESC harness pin order against the H743-Mini pinout before powering. A swapped
  VBAT/GND kills the board.
- MTF-01: output MSP @ 115200 (MicoAir tool), flat, lens down, near centre, clear of props.

## 3. Parameters

1. SD card (FAT32) in the FC.
2. Load `tsa_base.param`, write, reboot, load and write again, reboot.
3. `RNGFND1_GNDCLR` is a placeholder (0.03): **MEASURE sensor lens to floor, quad on landing
   gear, in metres**, and set it.
4. If the MTF-01 is not mounted with its arrow forward, set `FLOW_ORIENT_YAW` (centidegrees).
5. RC failsafe must land, not freeze:
   - Set the ELRS receiver failsafe to "No Pulses".
   - `FS_THR_ENABLE=3` (always Land), set by the param file.
   - `FS_OPTIONS=16`. It must not include bit 2 (value 4, "Continue if in Guided on RC
     failsafe"). With that bit set the quad would sit in Guided on the frozen setpoint.
   - On RC loss the script freezes its setpoints only as a bridge until the failsafe
     triggers. Land mode then takes over and the script releases.
   - Test props-off: arm, switch to Guided, turn the transmitter off. The mode must change to LAND.

## 4. Calibrate (Mission Planner > Setup > Mandatory Hardware)

1. Accelerometer, then level.
2. Compass (M10Q-5883, `COMPASS_ENABLE=1`, `COMPASS_USE=1`, `COMPASS_EXTERNAL=1`,
   `EK3_SRC1_YAW=1`):
   - After reboot, Setup > Compass must list one external compass. If not, check SDA/SCL.
   - Onboard calibration: Setup > Compass > Start, rotate the quad through all orientations
     until the bar completes, reboot. Do it away from steel, speakers and the bench supply.
     It also sets `COMPASS_ORIENT` automatically.
   - Large Vehicle MagCal (same page) is the alternative when you cannot rotate the quad:
     point it at a known heading and enter that heading. It needs the vehicle's location.
     With no GPS, set it first: right-click the map > Set EKF Origin Here.
   - Venue check before flying: in the Status tab, heading should be steady and match a
     phone compass at the take-off spot. Steel floors and stage rigging will bend it.
3. Radio: all bars move with the sticks except Pitch, which moves opposite. Sets `RC3_MIN/MAX`,
   which the script uses for the altitude range.
4. Battery monitor: verify voltage against a meter. Current scale for the F55A needs checking.

## 5. Sensor checks (props off)

- Status tab: `rangefinder1` reads the true height when you lift the quad (cm level).
- `opt_m_x/y` change when you slide the quad, `opt_qua` > 100 over a textured floor.
- Messages tab shows `stick_alt: loaded, 0.5-3.0 ft` after boot.
- If arming reports no position / needs origin: right-click the map > Set EKF Origin Here.

## 6. Motor test (props off)

Setup > Optional Hardware > Motor Test. A, B, C, D must spin front-right, rear-right,
rear-left, front-left (clockwise from front-right), with the correct rotation.
- Wrong order: `FRAME_TYPE=12` assumes a Betaflight-order harness; use 1 for ArduPilot order.
- Wrong direction: reverse in the ESC configurator (or `SERVO_BLH_RVMASK`).

## 7. First hover: Stabilize

Props on, open space. Short hover, land. Check for oscillation and hot motors.
Fly 30 s so `MOT_THST_HOVER` is learned.

## 8. AltHold

Height should hold on the rangefinder. Log check: `RFND.Dist` clean, no dropouts.

## 9. Tune

1. Setup > Initial Tune Parameters (prop size, cell count) and accept.
2. AutoTune (mode 15) per axis, in a large calm space.
3. Altitude loop, from logs (`SALT.Tgt` vs `SALT.Alt` vs `SALT.Rng`, or `PSCD`):
   `PSC_D_POS_P`, `PSC_D_VEL_P`, `PSC_D_ACC_P`, `PSC_D_ACC_I`.
   (4.7 names. The older `PSC_POSZ_P` / `PSC_ACCZ_*` no longer exist.)

## 10. Loiter

Optical flow hold. In-flight flow calibration if it drifts or toilet-bowls
(`FLOW_FXSCALER`, `FLOW_FYSCALER`). Position should hold within a few cm over texture.

## 11. Guided + script

1. Take off in Loiter, hover at about 1.5 ft. If Loiter refuses with "Need Position
   Estimate" on the ground (flow needs height to work), take off in AltHold and switch to
   Loiter once airborne.
2. Put the throttle stick where you want the height (mid = 1.75 ft), switch to Guided.
   GCS shows `stick_alt: engaged`. Height slews to the stick position at 0.3 m/s.
3. Throttle = absolute height 0.5-3 ft. Roll/pitch move the hold point at up to 0.5 m/s.
   Yaw stick = yaw rate.
4. **Leaving Guided: centre the throttle first.** In Loiter the stick is climb rate again,
   so a low stick means an immediate descent.
5. Land in Loiter or Land mode. Stabilize is the manual takeover.

Script constants (top of `stick_alt.lua`): `MIN_FT`, `MAX_FT`, `ALT_SLEW`, `XY_SPEED`,
`XY_LEAD_MAX`, `YAW_RATE`.

## Known limits

- Below about 8 in, ground effect and the sensor blind zone make hold unstable.
- Glossy or dark floors and strong lighting degrade ToF and flow.
- Height is the sensor height, not the gear height.
- `EK3_SRC1_POSZ=2` assumes a flat floor: flying over a box or a step moves the quad up with it.
