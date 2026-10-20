# EQ-3 GoTo firmware

Arduino firmware that turns a motorised **Bresser EQ-3** equatorial mount into a GoTo mount that KStars/Ekos (or any LX200-capable software) can control: GoTo, sync, sidereal tracking, manual moves and autoguiding.

The full build story is on my blog, Sky Workbench:

- [Part 1: motors, belts and the first spin](https://skyworkbench.com/posts/eq3-goto-part-1/)
- [Part 2: the firmware, sidereal time and first light](https://skyworkbench.com/posts/eq3-goto-part-2/)

## Hardware

| Part | Notes |
|---|---|
| Arduino UNO R4 WiFi | Runs this sketch, USB serial to the laptop |
| Arduino CNC Shield V3 | Shared enable on pin 8 |
| 2 × TMC2209 drivers | STEP/DIR mode (UART not used) |
| 2 × NEMA 17, 0.9°/step | One per axis |
| GT2 belts, 20T → 60T pulleys | 3:1 reduction onto each slow-motion shaft |

Pin mapping (CNC Shield V3):

| Axis | Shield slot | STEP | DIR |
|---|---|---|---|
| Declination | Y | 3 | 6 |
| Right ascension | Z | 4 | 7 |

## How it works

- **Protocol:** a subset of the Meade **LX200** protocol over USB serial at **9600 baud**. In Ekos, use the **LX200 GPS** driver.
- **Position model:** the RA motor position stores the **hour angle**. Reported RA = local sidereal time − hour angle, so a tracked target keeps a constant RA.
- **Time:** there is no clock chip. Ekos sends date, local time, UTC offset and longitude on connect (`:SC#`, `:SL#`, `:SG#`, `:Sg#`); the sketch keeps time with `millis()` and computes sidereal time from it. LX200 longitude is **west-positive**.
- **Tracking:** RA runs at the sidereal rate derived from the calibration (about 31.2 steps/s with the values in the sketch).
- **GoTo:** `:Sr#` / `:Sd#` set the target, `:MS#` slews with acceleration (AccelStepper), then tracking resumes. `:D#` reports when the slew is done.
- **Sync:** `:CM#` sets the current position to the last target, for example after plate solving in the Ekos Align module.
- **Moves and guiding:** `:Mn#` `:Ms#` `:Me#` `:Mw#` at the rate chosen with `:RG#` (guide, 10 steps/s) `:RC#` `:RM#` `:RS#`.

## Setup

1. Install the **AccelStepper** library in the Arduino IDE.
2. Open `lx200_mount_lst_latest/lx200_mount_lst_latest.ino` and upload it to the UNO R4 WiFi.
3. Adjust the calibration constants at the top of the sketch for your mount:
   - `STEPS_PER_DEG_RA`, `STEPS_PER_DEG_DEC`: motor steps per degree on each axis. Mine were tuned in the field with real GoTos, starting from an estimate.
   - `RA_DIR`, `DEC_DIR`: flip to `-1` if an axis moves the wrong way.
4. In KStars/Ekos, create a profile with the **LX200 GPS** mount driver on the Arduino's serial port (on Linux usually `/dev/ttyACM0`) at 9600 baud.
5. At the start of each session, plate-solve and sync (Ekos Align), then GoTo.

## Limitations

- No meridian flip / pier-side handling.
- Position is not remembered between power cycles: sync at the start of each session.
- Top slew speed is about 0.5°/s.
- TMC2209 UART configuration and the R4's Wi-Fi are not used yet.

## License

[MIT](LICENSE)
