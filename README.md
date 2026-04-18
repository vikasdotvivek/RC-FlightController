# RC-FlightController

Fixed-wing flight controller firmware for the Adafruit Feather ESP32 V2, built with Arduino-on-ESP32 and scheduled with FreeRTOS.

This project combines:

- sensor tasks for IMU, barometer, and GPS
- PWM output for ESC and control surfaces
- multiple autopilot / assist modes
- waypoint navigation with mission progress tracking
- LoRa telemetry for a ground station

The current codebase is functional as a firmware skeleton with working sensor ingestion, navigation math, telemetry transmission, and mode scheduling. A few subsystems are still placeholders, especially the Spektrum RX backend, airspeed sensing, and failsafe logic.

## At A Glance

| Area | Status | Notes |
| --- | --- | --- |
| Board support | Implemented | `adafruit_feather_esp32_v2` via PlatformIO |
| IMU | Implemented | ICM-20948 over shared I2C, complementary filter for roll/pitch, gyro and level calibration helpers |
| Barometer | Implemented | BMP3XX over I2C |
| GPS | Implemented | UART2 parser for GGA and RMC sentences |
| Waypoint navigation | Implemented | Haversine distance, bearing, leg and mission progress |
| Flight scheduler | Implemented | FreeRTOS tasks plus mode dispatcher |
| LoRa telemetry TX | Implemented | Raw binary telemetry packet sent periodically |
| LoRa telemetry RX | Reference receiver sketch available | Arduino IDE receiver sketch provided for ESP32-WROOM DevKit under `test/arduino_lora_receiver` |
| Manual / Stabilize / AltHold / Glide / Waypoint modes | Implemented | Runtime-selectable by RC mode PWM mapping |
| Spektrum receiver backend | Placeholder | Mode selection logic exists, but `rx_read()` is still stubbed |
| Airspeed sensor | Placeholder | Source file exists but not implemented |
| Failsafe | Placeholder | Scaffolding exists but logic is incomplete |

## Hardware Target

- Board: Adafruit Feather ESP32 V2
- Framework: Arduino
- Build system: PlatformIO
- Serial monitor speed: `115200`

PlatformIO environment:

```ini
[env:adafruit_feather_esp32_v2]
platform = espressif32
board = adafruit_feather_esp32_v2
framework = arduino
monitor_speed = 115200
```

## Wiring and Pin Map

The source of truth for project pin assignments is [`include/config.h`](include/config.h).

### Sensor and Comms Pins

| Subsystem | Interface | Pins | Notes |
| --- | --- | --- | --- |
| IMU (ICM-20948) | I2C | `SDA=22`, `SCL=20` | Shared sensor bus, explicit `Wire.begin(22, 20)` and `400 kHz` bus clock |
| Barometer (BMP3XX) | I2C | `SDA=22`, `SCL=20` | Shares the same mutex-protected sensor bus as the IMU |
| GPS | UART2 | `TX=8`, `RX=7` | Configured for `115200` baud |
| LoRa radio | SPI | `SCK=5`, `MOSI=19`, `MISO=21` | External SX127x-style radio expected |
| LoRa control | GPIO | `CS=26=A0`, `RST=4=A5`, `IRQ=39=A3` | IRQ pin is wired, RX callback mode is not yet used in firmware |

The shared I2C layer in [`src/hal/sensors/sensor_bus.cpp`](src/hal/sensors/sensor_bus.cpp):

- initializes `Wire` on `SDA=22`, `SCL=20`
- sets the bus clock to `400000 Hz`
- guards IMU and barometer access with a FreeRTOS mutex
- lets both sensor drivers retry initialization if a device is temporarily missing

### PWM Output Pins

| Output | GPIO | PWM channel | Purpose |
| --- | --- | --- | --- |
| ESC | `13` | `0` | Throttle output |
| Aileron servo | `12` | `1` | Roll control |
| Elevator servo | `27` | `2` | Pitch control |
| Rudder servo | `14` | `3` | Yaw / rudder control |

### PWM Operating Mode

All actuator outputs are generated with ESP32 LEDC PWM:

- Frequency: `50 Hz`
- Resolution: `16-bit`
- Output range: `1000 us` to `2000 us`

The motor mixer uses these conventions:

- throttle command is treated internally as `0..1000`, then shifted to `1000..2000 us`
- aileron / elevator / rudder are centered at `1500 us`
- control outputs are added as offsets around center

## Firmware Architecture

The main runtime lives in [`src/main.cpp`](src/main.cpp). The default Arduino `loop()` is only used for optional debug printing. Actual flight work happens in FreeRTOS tasks.

### FreeRTOS Tasks

| Task | Period | Purpose |
| --- | --- | --- |
| `TaskIMURead` | `10 ms` | Reads ICM-20948 and updates filtered roll/pitch |
| `TaskBarometerRead` | `50 ms` | Reads barometer pressure and altitude |
| `TaskGPSRead` | `50 ms` | Parses incoming GPS NMEA stream and updates navigation |
| `TaskFlightControl` | `100 ms` | Reads RC input, selects flight mode, runs active mode |
| `TaskTelemetryTx` | `500 ms` | Sends telemetry snapshot over LoRa |

### Main Data Flow

1. Sensors update shared state structures.
2. GPS fixes update the waypoint navigator.
3. The flight-control task reads the RC mode channel and selects a flight mode.
4. The active flight mode computes control outputs.
5. The mixer converts those commands into PWM outputs.
6. A telemetry task packages the current state and sends it over LoRa.

## Sensor Behavior

### IMU

Current IMU driver:

- Sensor: ICM-20948
- Bus: shared I2C via `sensor_bus`
- Accelerometer range: `8G`
- Gyro range: `500 deg/s`
- Magnetometer data rate: `50 Hz`
- Body-frame transform signs configurable in `include/config.h`

Roll and pitch are computed with a complementary filter:

- short-term motion from gyro
- long-term gravity reference from accelerometer

The IMU path also now includes:

- magnetometer reads stored in `IMUData_raw`
- gyro bias calibration via `IMU_Calibrate_Gyro()`
- level calibration helper via `IMU_Run_Level_Calibration(...)`
- startup calibration flags in `include/config.h`, disabled by default

Current yaw is still not coming from tilt-compensated magnetometer fusion or an AHRS yaw estimator. The IMU currently provides roll and pitch only. The filtered `imu_data.yaw` is updated from `gps_data.heading` when a valid GPS lock exists and the aircraft is moving faster than `WAYPOINT_MIN_GROUND_SPEED_MPS`. If that GPS condition is not met, yaw is not recomputed from the IMU and simply retains its previous value.

### Barometer

Current barometer driver:

- Sensor family: BMP3XX
- Bus: I2C
- Output altitude from measured pressure using `SEALEVELPRESSURE_HPA`

The sea-level reference pressure now lives in [`include/config.h`](include/config.h), so altitude accuracy depends on adjusting `SEALEVELPRESSURE_HPA` to local conditions.

### GPS

Current GPS driver:

- Interface: UART2
- Baud: `115200`
- Supported NMEA sentences: `GGA`, `RMC`

What the firmware extracts:

- decimal latitude / longitude
- altitude
- fix quality
- satellite count
- local time with configurable UTC offset
- ground speed
- track heading

The driver prints readable debug output when `GPS_DEBUG_OUTPUT_ENABLED` is set in `config.h`.

## Flight Modes

The mode selector is defined in [`src/main.cpp`](src/main.cpp) and mapped from the RC flight-mode PWM channel.

### Flight-Mode PWM Mapping

| Flight-mode PWM | Selected mode |
| --- | --- |
| `<= 1200` | `Manual` |
| `1200..1400` | `Stabilize` |
| `1400..1600` | `AltHold` |
| `1600..1800` | `Glide` |
| `> 1800` | `Waypoint` |

If the PWM value is invalid, the firmware falls back to:

- `DEFAULT_FLIGHT_MODE = FlightMode::Manual`

### Mode Summary

| Mode | What it does |
| --- | --- |
| `Manual` | Maps RC roll/pitch stick commands directly to servo output scaling through the mixer |
| `Stabilize` | Uses roll and pitch PIDs to track RC-commanded attitude |
| `AltHold` | Holds current barometric altitude by generating a desired pitch command from the altitude PID |
| `Glide` | Cuts throttle and holds pitch near zero while preserving roll command authority |
| `Waypoint` | Uses GPS navigation to steer toward the current waypoint and uses altitude hold toward waypoint altitude |

## Waypoint Navigation

Waypoint configuration lives in [`include/config.h`](include/config.h):

- `missionwaypoints[]`
- `num_waypoints`
- `WAYPOINT_ACCEPTANCE_RADIUS_METERS`
- `WAYPOINT_HEADING_TO_ROLL_KP`
- `WAYPOINT_MIN_GROUND_SPEED_MPS`

Each waypoint is:

```cpp
struct waypoint {
    double lat;
    double lon;
    float alt;
};
```

### Navigation Logic

The navigation backend in [`src/nav/waypoint.cpp`](src/nav/waypoint.cpp):

- computes distance to the active waypoint with the haversine formula
- computes initial bearing to the active waypoint
- tracks per-leg progress
- tracks overall mission progress
- advances to the next waypoint when the aircraft enters the acceptance radius

### Waypoint Autopilot Behavior

In `Waypoint` mode:

- heading error becomes desired roll
- altitude error becomes desired pitch
- pilot throttle is passed through
- rudder is currently held neutral

This matches normal fixed-wing turning behavior: the aircraft turns primarily by banking, not by commanding pure yaw.

## LoRa Telemetry

The LoRa driver in [`src/hal/comms/lora.cpp`](src/hal/comms/lora.cpp) initializes the radio and exposes thread-safe send / receive calls guarded by a mutex.

Current radio configuration:

- Frequency: `915000000 Hz`
- Sync word: `0xF3`
- Spreading factor: `7`

`115200` is only the USB serial monitor speed. It is not the LoRa over-the-air data rate.

### Telemetry Transport

Telemetry is sent every `500 ms` from the telemetry task.

The current telemetry implementation:

- sends a raw in-memory `telemetrydata` struct
- uses a single LoRa packet per snapshot
- currently sends `128` bytes per packet on ESP32
- does not add framing, versioning, checksums, or serialization beyond the native C++ layout

That means the ground station must decode the exact same field order and 32-bit `float` / `int` layout to interpret packets correctly.

### Telemetry Contents

The telemetry packet currently includes:

- attitude: roll, pitch, yaw
- GPS position, altitude, speed, heading, satellites, fix quality, lock state
- barometric altitude
- current flight mode
- waypoint target and mission status
- active waypoint index
- total waypoint count
- distance to waypoint
- desired heading
- target lat / lon / altitude
- leg progress percent
- mission progress percent
- mission complete flag

Not every field in `telemetrydata` is populated yet. In the current sender, the key populated values are attitude, altitude, GPS state, flight mode, and waypoint status. Fields such as desired control commands, airspeed, and failsafe status are still effectively placeholders.

## Receiver (ESP32-WROOM DevKit)

A ready-to-flash Arduino IDE receiver sketch is included at [`test/arduino_lora_receiver/arduino_lora_receiver.ino`](test/arduino_lora_receiver/arduino_lora_receiver.ino).

Default ESP32-WROOM DevKit receiver wiring in that sketch:

| Receiver signal | ESP32-WROOM DevKit pin |
| --- | --- |
| `SCK` | `18` |
| `MISO` | `19` |
| `MOSI` | `23` |
| `CS / NSS` | `5` |
| `RST` | `14` |
| `IRQ / DIO0` | `2` |

The receiver sketch expects:

- `915 MHz`
- sync word `0xF3`
- spreading factor `7`
- payload size `128 bytes`
- serial monitor speed `115200`

What it does:

- receives the Feather's raw binary telemetry packet
- verifies packet length before decoding
- prints decoded attitude, altitude, GPS, and waypoint values
- prints a hex dump if the packet size does not match the expected layout

If your LoRa module is wired differently on the ESP32-WROOM DevKit, change the pin constants at the top of the sketch before flashing.

## Receiver and RC Input

This section is about flight-control RC input, not the LoRa telemetry receiver described above.

The firmware already has the following logic:

- RC roll / pitch / throttle conversion helpers
- flight-mode PWM threshold selection

However, the actual receiver backend is still placeholder code in [`src/hal/comms/rx_spektrum.cpp`](src/hal/comms/rx_spektrum.cpp):

- `rx_init()` is not implemented
- `rx_read()` is not implemented
- raw PWM values are currently hard-coded test values

So the control architecture is wired, but real RC input depends on finishing the Spektrum interface.

## Project Layout

```text
include/
  config.h              Project-wide pins, gains, timing, waypoints
  datatypes.h           Shared state and telemetry structs
  flight/               Flight-mode and mixer interfaces
  hal/                  Sensors, comms, and actuators
  math/                 PID and math helpers
  nav/                  Waypoint navigation interfaces

src/
  main.cpp              Task creation, scheduler, flight-mode dispatch
  flight/               Manual, stabilize, alt-hold, glide, waypoint
  hal/sensors/          IMU, barometer, GPS
  hal/comms/            LoRa and receiver interface
  hal/actuators/        PWM outputs
  nav/waypoint.cpp      Mission navigation math

test/
  arduino_lora_receiver/ Arduino IDE LoRa receiver for ESP32-WROOM DevKit
```

## Build, Upload, and Monitor

Build:

```powershell
pio run
```

Upload:

```powershell
pio run -t upload
```

Open serial monitor:

```powershell
pio device monitor
```

## Important Configuration Knobs

Most tuning happens in [`include/config.h`](include/config.h).

Key groups:

- pin assignments
- I2C bus pins and frequency
- task periods and stack sizes
- PID gains and limits
- default flight mode
- RC PWM thresholds for flight-mode switching
- waypoint mission list
- GPS timezone offset
- LoRa settings
- IMU body-frame signs, level offsets, and calibration flags

## Current Limitations

This README tries to reflect the code as it exists now, not an idealized finished system.

Known limitations:

- Spektrum receiver backend is placeholder-only
- airspeed driver is not implemented yet
- failsafe logic is incomplete
- telemetry RX command protocol is not implemented yet
- telemetry uses raw binary struct layout instead of a stable serialized protocol
- magnetometer data is read but not yet fused into yaw estimation
- yaw control is not actively used in waypoint steering
- no persistent configuration or parameter storage yet


## Quick Start Checklist

1. Wire the Feather ESP32 V2, ICM-20948, BMP3XX, GPS, LoRa radio, ESC, and servos according to the tables above.
2. If you want a simple ground receiver, flash [`test/arduino_lora_receiver/arduino_lora_receiver.ino`](test/arduino_lora_receiver/arduino_lora_receiver.ino) to the ESP32-WROOM DevKit and wire its LoRa module to `SCK=18`, `MISO=19`, `MOSI=23`, `CS=5`, `RST=14`, `IRQ=2`.
3. Update `missionwaypoints[]` in `include/config.h`.
4. Adjust `SEALEVELPRESSURE_HPA`, IMU signs, and any IMU calibration offsets in `include/config.h` for your airframe and location.
5. Build with `pio run`.
6. Upload and monitor at `115200`.
7. Verify ICM-20948 startup, barometer readings, GPS lock, and LoRa telemetry before attempting closed-loop flight.

## [next steps/TODO]
- finish the Spektrum RX backend and validate live RC mode switching
- add yaw coordination or rudder mixing for cleaner turns
- seperate imu_yaw and gps_heading
- sd card


