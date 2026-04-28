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
| IMU | Implemented | ICM-20948 over shared I2C, complementary filter for roll/pitch/yaw (tilt-compensated magnetometer fusion), gyro/level/magnetometer calibration helpers |
| Barometer | Implemented | BMP3XX over I2C |
| GPS | Implemented | UART2 parser for GGA and RMC sentences |
| Waypoint navigation | Implemented | Haversine distance, bearing, leg and mission progress |
| Flight scheduler | Implemented | FreeRTOS tasks plus mode dispatcher |
| NeoPixel status LED | Implemented | Built-in Feather NeoPixel cycles through active subsystem faults |
| LoRa telemetry | Implemented | Bidirectional telemetry and OTA PID tuning |
| Manual / Stabilize / AltHold / Glide / Waypoint modes | Implemented | Runtime-selectable by RC mode PWM mapping |
| PWM RC Receiver | Implemented | Hardware timer-based PWM capture via ESP32 MCPWM |
| Airspeed sensor | Implemented | MS4525DO over shared I2C |
| Failsafe | Placeholder | Scaffolding exists but logic is incomplete |

## Over-the-Air (OTA) PID Tuning

The system supports live, over-the-air tuning of the flight controller's PID gains from the Ground Control Station via a robust Request-Acknowledge LoRa protocol.

### 1. Loading PID Values on Startup
When the Flight Controller boots, the PID values go through a two-stage initialization process:
1. **Hardcoded Defaults (Fallback):** First, the firmware applies the hardcoded defaults defined in `include/config.h` (e.g., `roll_kp = 20.0f`).
2. **SD Card Override:** Next, it attempts to load `pid_config.json` from the SD card. If parsing is successful, it overrides the hardcoded defaults with the saved values.

### 2. Thread-Safe Tuning Locks
Because this is a multi-core FreeRTOS system, PID values are read and written by multiple tasks concurrently (Flight Control, Telemetry, and SD Logging). To prevent data tearing, the active PID configurations are guarded by a FreeRTOS spinlock (`g_tuning_lock`).

### 3. Receiving Live Updates
Because LoRa is a half-duplex radio, the protocol synchronizes with the telemetry stream to avoid mid-air collisions:
1. **GCS Queueing:** The Ground Station receives a command (`pid roll 1.5 0.1 0.05`) and queues it.
2. **Ping-Pong Transmission:** The GCS waits for the Flight Controller to transmit a telemetry packet, then instantly fires the command back while the FC is in receive mode.
3. **FC Application:** The Flight Controller validates the packet, updates the PID controllers safely via the spinlock, and immediately saves the new configuration back to `pid_config.json` on the SD card.
4. **Acknowledgement:** The FC transmits an ACK back to the GCS to confirm the new tuning is active.

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
| LoRa control | GPIO | `CS = 26 = A0`, `RST =4 = A5`, `IRQ = 39 = A3` | IRQ pin is wired, RX callback mode is not yet used in firmware |
| SD Card | SPI | `CS = 25 = A1` | SPI wiring same as LoRa |
| Built-in status LED | NeoPixel | `PIN_NEOPIXEL` | Used for subsystem health / fault indication |
| Airspeed (MS4525DO) | I2C | `SDA=22`, `SCL=20` | `0x28`, uses shared sensor bus |

The shared I2C layer in [`src/hal/sensors/sensor_bus.cpp`](src/hal/sensors/sensor_bus.cpp):

- initializes `Wire` on `SDA=22`, `SCL=20`
- sets the bus clock to `400000 Hz`
- guards IMU and barometer access with a FreeRTOS mutex
- lets both sensor drivers retry initialization if a device is temporarily missing

The shared SPI layer in [`src/hal/comms/spi_bus.cpp`](src/hal/comms/spi_bus.cpp):

- initializes the SPI bus on `SCK=5`, `MOSI=19`, `MISO=21`
- sets a unified bus clock to `10000000 Hz` (10 MHz) for stability across devices
- guards LoRa radio and SD card logger access with a FreeRTOS mutex
- enforces safe peripheral deselection before driver accesses to prevent bus collisions

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
| `TaskIMURead` | `10 ms` | Reads ICM-20948 and updates filtered roll/pitch/yaw |
| `TaskBarometerRead` | `50 ms` | Reads barometer pressure and altitude |
| `TaskGPSRead` | `50 ms` | Parses incoming GPS NMEA stream and updates navigation |
| `TaskFlightControl` | `100 ms` | Reads RC input, selects flight mode, runs active mode |
| `TaskTelemetryTx` | `500 ms` | Sends telemetry snapshot over LoRa |
| `TaskSDLog` | `50 ms` | Logs telemetry snapshots to the SD card when enabled |
| `TaskStatusLED` | `100 ms` | Updates the built-in NeoPixel based on current subsystem health |

### Main Data Flow

1. Sensors update shared state structures.
2. GPS fixes update the waypoint navigator.
3. The flight-control task reads the RC mode channel and selects a flight mode.
4. The active flight mode computes control outputs.
5. The mixer converts those commands into PWM outputs.
6. A telemetry task packages the current state and sends it over LoRa.
7. A status LED task cycles through active subsystem faults on the built-in NeoPixel.

## Status LED

The Feather ESP32 V2 built-in NeoPixel is used as a quick health indicator for major subsystems. The implementation lives in [`src/status/status_led.cpp`](src/status/status_led.cpp) and is driven by `TaskStatusLED` from [`src/main.cpp`](src/main.cpp).

The LED cycles through every currently active fault instead of showing only one. If no tracked faults are active, it shows solid green.

### Fault Color Map

| Condition | LED behavior |
| --- | --- |
| GPS not healthy or no lock acquired | Solid red |
| SD card not ready or no log file open | Blinking white |
| LoRa not initialized | Solid yellow/orange |
| Airspeed not healthy | Blinking cyan |
| IMU not healthy | Solid purple |
| Barometer not healthy | Solid blue |
| No active faults | Solid green |

### Status LED Timing

The timing and brightness are configurable in [`include/config.h`](include/config.h):

- `STATUS_LED_TASK_PERIOD_MS`
- `STATUS_LED_CYCLE_PERIOD_MS`
- `STATUS_LED_BLINK_PERIOD_MS`
- `STATUS_LED_BRIGHTNESS`

Current behavior:

- the LED task updates every `100 ms`
- each active fault is shown for `1200 ms` before rotating to the next one
- blinking faults toggle at `500 ms`

### Notes

- GPS fault indication intentionally treats "connected but not locked yet" as a fault, so the LED stays red until a usable fix is available.
- SD status currently means the card mounted successfully and a log file is open. It does not yet detect every possible runtime write failure after startup.
- LoRa and SD are only considered faults when their corresponding `LORA_LOGGING_ENABLED` or `SD_LOGGING_ENABLED` config flags are enabled.

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
- magnetometer hard/soft iron calibration helper via `IMU_Run_Mag_Calibration(...)`
- startup calibration flags in `include/config.h`, disabled by default

Yaw is estimated in the IMU path using gyro integration plus tilt-compensated magnetometer correction. This means `imu_data.yaw` is available without requiring GPS lock. GPS heading remains available as a separate navigation signal (`gps_data.heading`) and is still used by waypoint logic where appropriate.

Magnetometer calibration parameters now live in `include/config.h`:

- `IMU_MAG_OFFSET_X`, `IMU_MAG_OFFSET_Y`, `IMU_MAG_OFFSET_Z`
- `IMU_MAG_SCALE_X`, `IMU_MAG_SCALE_Y`, `IMU_MAG_SCALE_Z`
- `IMU_RUN_MAG_CALIBRATION` startup flag

Important calibration note:

- `IMU_RUN_MAG_CALIBRATION` should not be `true` at the same time as `IMU_RUN_STARTUP_GYRO_CALIBRATION` or `IMU_RUN_STARTUP_LEVEL_CALIBRATION` at startup.

### Barometer

Current barometer driver:

- Sensor family: BMP3XX
- Sensor: BMP388
- Bus: I2C
- Output altitude from measured pressure using `SEALEVELPRESSURE_HPA`

The sea-level reference pressure now lives in [`include/config.h`](include/config.h), so altitude accuracy depends on adjusting `SEALEVELPRESSURE_HPA` to local conditions.

### GPS

Current GPS driver:

- Interface: UART2
- Baud: `115200`
- Supported NMEA sentences: `GGA`, `RMC`
- Sensor: HGLRC M100-5883

What the firmware extracts:

- decimal latitude / longitude
- altitude
- fix quality
- satellite count
- local time with configurable UTC offset
- ground speed
- track heading
- GPS Heading/Compass will only be available over I2C for HGLRC M100-5883

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

#### Navigation Logic

The navigation backend in [`src/nav/waypoint.cpp`](src/nav/waypoint.cpp):

- computes distance to the active waypoint with the haversine formula
- computes initial bearing to the active waypoint
- tracks per-leg progress

#### Waypoint Autopilot Behavior

In `Waypoint` mode:

- heading error becomes desired roll
- altitude error becomes desired pitch
- pilot throttle is passed through
- rudder is currently held neutral

This matches normal fixed-wing turning behavior: the aircraft turns primarily by banking, not by commanding pure yaw.

## Telemetry & Ground Station

### LoRa Telemetry

The LoRa driver in [`src/hal/comms/lora.cpp`](src/hal/comms/lora.cpp) initializes the radio and exposes thread-safe send / receive calls guarded by a mutex.

Current radio configuration:

- Frequency: `915000000 Hz`
- Sync word: `0xF3`
- Spreading factor: `7`

`115200` is only the USB serial monitor speed. It is not the LoRa over-the-air data rate.

#### Telemetry Transport

Telemetry is sent every `500 ms` from the telemetry task.

The current telemetry implementation:

- sends a raw in-memory `telemetrydata` struct
- uses a single LoRa packet per snapshot
- currently sends `228` bytes per packet on ESP32
- does not add framing, versioning, checksums, or serialization beyond the native C++ layout

That means the ground station must decode the exact same field order and 32-bit `float` / `int` layout to interpret packets correctly.

#### Telemetry Contents

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

The telemetry packet includes attitude, altitude, GPS state, flight mode, waypoint status, airspeed, RC inputs, and the active PID controller gains for live GCS monitoring.

### LoRa Ground Receiver

A ready-to-flash Arduino IDE receiver sketch is included at [`test/arduino_lora_receiver/arduino_lora_receiver.ino`](test/arduino_lora_receiver/arduino_lora_receiver.ino).

Any microcontroller (like an ESP32-WROOM, Feather, or standard Arduino) can be used as long as the pins are wired correctly. Below wiring is for ESP32-Feather v2. Default wiring in that sketch (ESP32):

| Receiver signal | Microcontroller pin |
| --- | --- |
| `SCK` | `5` |
| `MISO` | `21` |
| `MOSI` | `19` |
| `CS / NSS` | `26` |
| `RST` | `4` |
| `IRQ / DIO0` | `39` |

The receiver sketch expects:

- `915 MHz`
- sync word `0xF3`
- spreading factor `7`
- signal bandwidth `250 kHz`
- payload size `228 bytes`
- serial monitor speed `921600`

What it does:

- receives the Feather's raw binary telemetry packet
- verifies packet length before decoding
- prints decoded attitude, altitude, GPS, and waypoint values
- prints a hex dump if the packet size does not match the expected layout

If your LoRa module is wired differently or you are using another microcontroller, change the pin constants at the top of the sketch before flashing.

### GCS Bridge and Dashboard

The Ground Control Station tools live under [`test/gcs`](test/gcs):

- [`test/gcs/gcs_bridge.py`](test/gcs/gcs_bridge.py): reads receiver serial telemetry and rebroadcasts packets as JSON over WebSocket

#### What the Bridge Does

`gcs_bridge.py`:

- reads the ESP32 LoRa receiver serial output line-by-line
- groups telemetry fields into complete packet JSON objects
- broadcasts each packet to connected WebSocket clients
- forwards command JSON from WebSocket clients back to the receiver over serial

#### Install

From the repo root:

```powershell
cd test/gcs
python -m venv .venv
```

Activate the environment:

Windows PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
```

Linux/macOS:

```bash
source .venv/bin/activate
```

Install dependencies:

```powershell
pip install pyserial websockets
```

### Setup

1. Connect the ESP32 receiver board to your computer over USB.
2. Identify the serial port:

```powershell
python gcs_bridge.py --list-ports
```

3. Pick the correct port for your OS:

- Windows example: `COM5`
- Linux example: `/dev/ttyUSB0`
- macOS example: `/dev/tty.usbserial-xxxx`

### Usage

Start the bridge from `test/gcs`:

Linux example:

```bash
python gcs_bridge.py --port /dev/ttyUSB0
```

Windows example:

```powershell
python gcs_bridge.py --port COM5 --baud 921600 --ws-port 8765
```

With defaults, the WebSocket server listens on `ws://localhost:8765`.

Open [`test/gcs/gcs_gui.html`](test/gcs/gcs_gui.html) in a browser and connect it to:

- `ws://localhost:8765` when the bridge runs locally
- `ws://<bridge-host-ip>:8765` if the bridge is running on another machine

### CLI Reference

```text
python gcs_bridge.py --port /dev/ttyUSB0
python gcs_bridge.py --port COM5 --baud 921600 --ws-port 8765
python gcs_bridge.py --list-ports
```

## Flight Log Viewer

A self-contained browser tool for analysing SD card logs from the flight controller lives at [`test/rc_log_viewer.html`](test/rc_log_viewer.html).

Open the file directly in Chrome, Firefox, or Edge — no server, no install, no internet connection required.

### How to Use

1. Pull the SD card from the Feather and copy the CSV log file to your computer.
2. Open `test/rc_log_viewer.html` in a browser.
3. Drag and drop the CSV onto the upload zone, or click to browse for the file.
4. Set the **Sample rate** in the top-right to match the rate you configured in firmware (default `10 Hz`).

### Tabs

| Tab | What it shows |
| --- | --- |
| **Dashboard** | Stat cards (duration, airspeed, altitude, distance, GPS sats), primary attitude chart (roll/pitch/yaw), altitude chart, GPS flight path map, desired vs actual flight path, airspeed vs GPS speed, sensor health summary, flight mode timeline |
| **Presets** | Grouped one-click plots: Attitude (roll/pitch/yaw, desired vs actual), Altitude, Speed, PID outputs, RC inputs, GPS, Navigation |
| **Custom Plot** | Pick any combination of log fields; Multi-Axis / Single Axis / Normalize modes; drag a Y-axis label to pan it, scroll wheel to zoom, double-click to reset |
| **Sensor Health** | Sensor status gauges (IMU, baro, GPS, RX), GPS quality metrics, altitude source comparison, mission progress, PID output charts, RC stick PWM charts |

### Expected Log Fields

The viewer auto-detects whatever columns are present in the CSV. It works best with these field names (the names the SD logger uses):

```
roll  pitch  yaw  des_roll  des_pitch  des_yaw  des_throttle
altitude  des_altitude  baro_altitude
gps_lat  gps_long  gps_alt  gps_speed  gps_heading
gps_sats  gps_fix_quality  gps_lock_acquired
airspeed  flightmode  armed
waypoint_distance  waypoint_heading  waypoint_target_lat  waypoint_target_lon
waypoint_target_alt  waypoint_leg_progress  waypoint_mission_progress
waypoint_index  waypoint_total  waypoint_mission_complete
imu_healthy  baro_healthy  gps_healthy  rx_healthy
roll_pid_out  pitch_pid_out  yaw_pid_out
rx_throttle_pwm  rx_aileron_pwm  rx_elevator_pwm  rx_rudder_pwm  rx_mode_pwm
```

Any columns not in the list above still appear in Custom Plot and can be plotted freely.

## Receiver and RC Input

This section is about flight-control RC input, not the LoRa telemetry receiver described above.

The firmware already has the following logic:

- RC roll / pitch / throttle conversion helpers
- flight-mode PWM threshold selection

The receiver backend in `src/hal/comms/rx_spektrum.cpp` uses the ESP32's MCPWM hardware blocks to simultaneously capture up to 6 channels of standard PWM RC signals with hardware-level microsecond precision.

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
  rc_log_viewer.html     Standalone browser tool for analysing SD card flight logs
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
- status LED timing and brightness
- PID gains and limits
- default flight mode
- RC PWM thresholds for flight-mode switching
- waypoint mission list
- GPS timezone offset
- LoRa settings
- IMU body-frame signs, magnetometer offsets/scales, level offsets, and startup calibration flags

## Current Limitations

This README tries to reflect the code as it exists now, not an idealized finished system.

Known limitations:

- failsafe logic is incomplete
- telemetry uses raw binary struct layout instead of a stable serialized protocol
- yaw control is not actively used in waypoint steering


## Quick Start Checklist

1. Wire the Feather ESP32 V2, ICM-20948, BMP3XX, GPS, LoRa radio, ESC, and servos according to the tables above.
2. If you want a simple ground receiver, flash [`test/arduino_lora_receiver/arduino_lora_receiver.ino`](test/arduino_lora_receiver/arduino_lora_receiver.ino) to the ESP32-WROOM DevKit and wire its LoRa module to `SCK=18`, `MISO=19`, `MOSI=23`, `CS=5`, `RST=14`, `IRQ=2`.
3. Update `missionwaypoints[]` in `include/config.h`.
4. Adjust `SEALEVELPRESSURE_HPA`, IMU signs, and any IMU calibration offsets in `include/config.h` for your airframe and location.
5. Build with `pio run`.
6. Upload and monitor at `115200`.
7. Verify ICM-20948 startup, barometer readings, GPS lock, and LoRa telemetry before attempting closed-loop flight.

## Next Steps / TODO
- Complete the failsafe logic implementation (RTL/Loiter on link loss)
- Add yaw coordination / rudder mixing for cleaner turns
- Validate and tune magnetometer calibration values per airframe