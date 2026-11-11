# TSA Flight Software: objectives

Fork of ArduPilot (ArduCopter 4.7.2, GPLv3). The TSA-specific work lives in `TSA/`.

## Goal

Turn an FPV quad into a stable, "DJI-like" platform that can be flown live at
competitions and demos in unknown venues.

## Primary objective: precise low-altitude hold

- Hold height between 0 and 3 ft above the floor.
- Accuracy target +/- 0.5 in, realistic +/- 1 in.
- The throttle stick commands an **absolute altitude**, not a climb rate:
  stick low = 0.5 ft, stick high = 3 ft.

## Supporting objectives

1. **Horizontal position hold** from optical flow, so the quad stays put hands-off.
   Roll/pitch sticks move the hold point; the yaw stick turns the quad.
2. **Onboard sensing only.** No motion capture, no companion computer, no GPS indoors.
3. **Safe behaviour on faults.**
   - RC loss lands the quad (it must never freeze in place).
   - A Stabilize mode on the switch is always available as a manual takeover.
   - The pilot is warned if the height sensor drops out.
4. **Repeatable setup.** One parameter file, one script and one checklist take a bare
   flight controller to a flying vehicle.
5. **Tested before flown.** The script logic and failsafe are proven in SITL before the
   first hardware flight, and tuned from flight logs afterwards.

## Constraints

- Hardware: Matek H743-Mini V3, T-Motor F55A PRO II ESC, MicoAir MTF-01 (optical flow + ToF),
  Matek M10Q-5883 (compass only indoors), ELRS receiver.
- Stock ArduCopter firmware: the feature is a Lua script on the SD card, with no changes to
  the ArduPilot source.
- Keep `COPYING.txt` (GPLv3).

## Known limits

- Below about 8 in, ground effect and the sensor blind zone make hold unstable.
- Glossy or dark floors and strong lighting degrade the ToF and flow readings.
- Height hold assumes a flat floor.

## Status

| Item | State |
|---|---|
| `scripts/stick_alt.lua` | written, checked against 4.7.2 source, logic-tested offline (33 checks pass) |
| `tests/test_stick_alt.lua` | offline logic test against a mocked flight controller, passing |
| `params/tsa_base.param` | written, names checked against 4.7.2 source |
| `BRINGUP.md` checklist | written |
| SITL test (`SITL.md`) | procedure written, not yet run |
| Hardware build and first flight | not started |
| Altitude tuning to +/- 1 in | not started |

## Possible later work

- Outdoor flight with GPS (wire the M10Q GPS UART, change the EKF sources).
- Script constants as parameters, adjustable from the ground station without editing the file.
