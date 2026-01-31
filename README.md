# UTRA Hacks 2026 Autonomous Robot

Arduino Uno + PlatformIO project for an autonomous robot that navigates the Winter-Olympics style course, drops boxes, follows paths, aligns to target center via color, shoots the ball, and returns.

## Overview
- Motors: 2x DC with L298N driver
- Servos: claw (pick/drop), shooter (launch)
- Sensors: 2x IR line sensors, HC-SR04 ultrasonic, TCS3200 color sensor
- Control: modular drivers and a simple state machine in `src/main.cpp`

## Pin Assignments (see `include/pins.hpp`)
- Motor (L298N):
  - `ENA` 5, `IN1` 7, `IN2` 8 (Left)
  - `ENB` 6, `IN3` 11, `IN4` 12 (Right)
- Servos: `Claw` 9, `Shooter` 10
- IR sensors: `Left` A0, `Right` A1
- Ultrasonic: `TRIG` A2, `ECHO` A3
- TCS3200: `S0` 2, `S1` 3, `S2` 4, `S3` 13, `OUT` A4
- Cal button (optional): A5 (INPUT_PULLUP)

## Build & Upload
```bash
cd "/Users/ericx/Documents/PlatformIO/Projects/UTRA Hacks 2026"
platformio run
platformio run -t upload
```
- Open Serial Monitor at 115200 baud.

## Serial Commands
- `h`: show help
- `ir`: print IR analog values (Left/Right)
- `d`: print ultrasonic distance (cm)
- `m f` / `m s` / `m l`: motors forward/stop/slow
- `c o` / `c c`: claw open/close
- `shoot`: fire shooter
- `rgb`: raw TCS3200 pulses (lower = brighter)
- `rgbn`: normalized 0..255 RGB and classification
- Tunables at runtime:
  - `set bs <int>`: base speed
  - `set tg <int>`: turn gain
  - `set ib <int>`: IR black level
  - `set iw <int>`: IR white level
  - `set rmin|rmax|gmin|gmax|bmin|bmax <int>`: color calibration ranges

## Color Calibration (TCS3200)
This project aligns to the calibration scheme using pulse widths:
- Raw pulses: `rgb` prints R, G, B pulse widths (lower = brighter)
- Normalized values: `rgbn` maps pulse ranges to 0..255 (255 = bright) via `Config::R_MIN/R_MAX`, `G_MIN/G_MAX`, `B_MIN/B_MAX`.
- Update ranges live using `set rmin/rmax/...` commands.
- Initial ranges set in `include/config.hpp`:
  - R: 44–72, G: 51–121, B: 67–71

## Tuning (see `include/config.hpp`)
- Speeds: `BASE_SPEED`, `SLOW_SPEED`, `TURN_GAIN`
- IR thresholds: `IR_BLACK_LEVEL`, `IR_WHITE_LEVEL`
- Ultrasonic stop distance: `OBSTACLE_STOP_CM`
- Shooter/claw positions: `SHOOT_*`, `CLAW_*`

## Behavior Summary
- Search for box (line follow + ultrasonic stop)
- Pick box (claw close) → drive to blue zone → drop box
- Choose green path → detect rings → sweep to black center
- Shoot ball → back off → return along line

## File Structure
- `platformio.ini`: PlatformIO environment
- `include/pins.hpp`: pin assignments
- `include/config.hpp`: tunable parameters + color calibration ranges
- `src/main.cpp`: drivers, state machine, Serial interface
- `lib/`: optional libraries
- `test/`: PlatformIO tests (empty by default)

## Safety & Notes
- Secure all wiring; avoid shorts and strain on servo cables.
- Verify motor polarity and servo angles before full-speed tests.
- Ramp approach: prefer `SLOW_SPEED` for alignment; increase only after stable.
