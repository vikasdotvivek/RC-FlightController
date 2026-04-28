#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "datatypes.h"
#include "math/pid.h"
#include "config.h"
#include "flight/flightmodes.h"
#include "flight/motormixer.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/baro.h"
#include "hal/sensors/sensor_bus.h"
#include "hal/sensors/gps.h"
#include "hal/sensors/airspeed.h"
#include "hal/comms/lora.h"
#include "hal/comms/rx_spektrum.h"
#include "hal/actuators/pwm_out.h"
#include "flight/arming.h"
#include "flight/telemetry.h"
#include "logging/sd_logger.h"
#include "flight/gcs_commands.h"
#include "nav/waypoint.h"
#include "flight/home.h"
#include "status/status_led.h"

IMUData_raw currentIMU; // Global variable to hold our sensor state
IMUData_filtered imu_data = {};
BarometerData baro_data = {};
GPSData gps_data = {};  // Global variable to hold GPS state
AirspeedData airspeed_data = {};
FlightMode active_flight_mode = DEFAULT_FLIGHT_MODE;

///pid initialisations
PIDController roll_pid(roll_kp, roll_ki, roll_kd, max_roll_output, max_roll_integral);
PIDController pitch_pid(pitch_kp, pitch_ki, pitch_kd, max_pitch_output, max_pitch_integral);
PIDController yaw_pid(yaw_kp, yaw_ki, yaw_kd, max_yaw_output, max_yaw_integral);
PIDController altitude_pid(alt_kp, alt_ki, alt_kd, max_alt_output, max_alt_integral);
PIDController headingerror_pid(headingerror_kp, headingerror_ki, headingerror_kd, max_headingerror_output, max_headingerror_integral);

namespace {

bool g_flight_mode_initialized = false;

void RunStartupIMUCalibrationIfEnabled() {
    if (IMU_RUN_STARTUP_GYRO_CALIBRATION) {
        (void)IMU_Calibrate_Gyro();
    }

    if (IMU_RUN_STARTUP_LEVEL_CALIBRATION) {
        float roll_offset_deg = 0.0f;
        float pitch_offset_deg = 0.0f;
        (void)IMU_Run_Level_Calibration(roll_offset_deg, pitch_offset_deg);
    }

    if (IMU_RUN_MAG_CALIBRATION) {
        float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;
        float scale_x = 1.0f, scale_y = 1.0f, scale_z = 1.0f;
        (void)IMU_Run_Mag_Calibration(offset_x, offset_y, offset_z, scale_x, scale_y, scale_z);
    }
}

void UpdateFilteredIMUData() {
    if (currentIMU.healthy) {
        imu_data.roll = currentIMU.roll;
        imu_data.pitch = currentIMU.pitch;
        imu_data.yaw = currentIMU.yaw;
    }
}

FlightMode ResolveFlightModeFallback(FlightMode requested_mode) {
    if (requested_mode != FlightMode::Manual && !currentIMU.healthy) {
        return FlightMode::Manual;
    }

    return requested_mode;
}

FlightMode DetermineFlightMode() {
    const float flight_mode_pwm = get_flight_mode_pwm();

    if (flight_mode_pwm <= FLIGHT_MODE_PWM_MANUAL_MAX) {
        return FlightMode::Manual;
    }

    if (flight_mode_pwm <= FLIGHT_MODE_PWM_STABILIZE_MAX) {
        return FlightMode::Stabilize;
    }

    if (flight_mode_pwm <= FLIGHT_MODE_PWM_ALT_HOLD_MAX) {
        return FlightMode::AltHold;
    }
    return FlightMode::Glide;
}

void InitializeFlightMode(FlightMode mode) {
    switch (mode) {
        case FlightMode::Manual:
            mode_manual_init();
            break;
        case FlightMode::Stabilize:
            mode_stabilize_init();
            break;
        case FlightMode::AltHold:
            mode_alt_hold_init();
            break;
        case FlightMode::Glide:
            mode_glide_init();
            break;
        case FlightMode::Waypoint:
            navigation.restart_mission();
            mode_waypoint_init();
            break;
    }
}

void RunFlightMode(FlightMode mode) {
    switch (mode) {
        case FlightMode::Manual:
            mode_manual_run();
            break;
        case FlightMode::Stabilize:
            mode_stabilize_run();
            break;
        case FlightMode::AltHold:
            mode_alt_hold_run();
            break;
        case FlightMode::Glide:
            mode_glide_run();
            break;
        case FlightMode::Waypoint:
            mode_waypoint_run();
            break;
    }
}

} // namespace

telemetrydata BuildTelemetrySnapshot() {
    telemetrydata snapshot = {};

    snapshot.roll = imu_data.roll;
    snapshot.pitch = imu_data.pitch;
    snapshot.yaw = imu_data.yaw;
    snapshot.des_roll     = get_des_roll();
    snapshot.des_pitch    = get_des_pitch();
    snapshot.des_yaw      = get_des_yaw();
    snapshot.des_throttle = get_des_throttle();
    snapshot.altitude = baro_data.healthy ? baro_data.altitude : gps_data.altitude;
    snapshot.des_altitude = navigation.get_target_altitude();
    snapshot.airspeed = airspeed_data.healthy ? airspeed_data.airspeed_mps : 0.0f;
    snapshot.gps_lat = static_cast<float>(gps_data.latitude);
    snapshot.gps_long = static_cast<float>(gps_data.longitude);
    snapshot.gps_alt = gps_data.altitude;
    snapshot.gps_speed = gps_data.speed;
    snapshot.gps_heading = gps_data.heading;
    snapshot.gps_sats = gps_data.satellites;
    snapshot.gps_fix_quality = gps_data.fix_quality;
    snapshot.gps_lock_acquired = gps_data.lock_acquired ? 1 : 0;
    snapshot.baro_altitude = baro_data.altitude;
    snapshot.flightmode = static_cast<float>(static_cast<int>(active_flight_mode));
    snapshot.waypoint_distance = navigation.get_target_distance();
    snapshot.waypoint_heading = navigation.get_target_heading();
    snapshot.waypoint_target_alt = navigation.get_target_altitude();
    snapshot.waypoint_leg_progress = navigation.get_leg_progress_percent();
    snapshot.waypoint_mission_progress = navigation.get_mission_progress_percent();
    snapshot.waypoint_index = navigation.get_current_waypoint_index();
    snapshot.waypoint_total = navigation.get_total_waypoint_count();
    snapshot.waypoint_mission_complete = navigation.mission_completed() ? 1 : 0;

    snapshot.imu_healthy  = currentIMU.healthy  ? 1 : 0;
    snapshot.baro_healthy = baro_data.healthy   ? 1 : 0;
    snapshot.gps_healthy  = gps_data.healthy    ? 1 : 0;
    snapshot.rx_healthy   = rc_data.healthy     ? 1 : 0;
    snapshot.armed        = is_armed() ? 1 : 0;

    snapshot.roll_pid_out  = roll_pid.getLastOutput();
    snapshot.pitch_pid_out = pitch_pid.getLastOutput();
    snapshot.yaw_pid_out   = yaw_pid.getLastOutput();

    GCS_PopulateTelemetryTuning(snapshot);

    snapshot.rx_throttle_pwm = rc_data.throttle_pwm;
    snapshot.rx_aileron_pwm  = rc_data.aileron_pwm;
    snapshot.rx_elevator_pwm = rc_data.elevator_pwm;
    snapshot.rx_rudder_pwm   = rc_data.rudder_pwm;
    snapshot.rx_mode_pwm     = rc_data.flightmode_pwm;

    if (snapshot.waypoint_index >= 0 && snapshot.waypoint_index < num_waypoints) {
        snapshot.waypoint_target_lat =
            static_cast<float>(missionwaypoints[snapshot.waypoint_index].lat);
        snapshot.waypoint_target_lon =
            static_cast<float>(missionwaypoints[snapshot.waypoint_index].lon);
    }

    return snapshot;
}

void TaskIMURead(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        IMU_Read(currentIMU);
        UpdateFilteredIMUData();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskBarometerRead(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(BARO_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        Barometer_Read(baro_data);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskAirspeedRead(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(AIRSPEED_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        Airspeed_Read(airspeed_data);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskGPSRead(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(GPS_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        if (GPS_Read(gps_data)) {
            if (gps_data.lock_acquired) {

                if(!home_is_set()) {
                    float home_msl = baro_data.healthy ? baro_data.altitude : gps_data.altitude;
                    set_home_location(gps_data.latitude, gps_data.longitude, home_msl);
                    Serial.println("Home location set.");

                }

                navigation.update(gps_data.latitude, gps_data.longitude, gps_data.altitude);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskFlightControl(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(FLIGHT_CONTROL_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();
    bool prev_armed = false;

    for (;;) {
        GCS_ApplyPendingCommands();
        arming_update();
        const bool armed = is_armed();

        if (!armed) {
            motormixer_compute(0.0f, 0.0f, 0.0f, 0.0f);
            prev_armed = false;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // Force flight mode re-init on the first tick after arming.
        if (!prev_armed) {
            g_flight_mode_initialized = false;
        }
        prev_armed = true;

        const FlightMode desired_mode = DetermineFlightMode();
        const FlightMode effective_mode = ResolveFlightModeFallback(desired_mode);
        if (!g_flight_mode_initialized || effective_mode != active_flight_mode) {
            active_flight_mode = effective_mode;
            InitializeFlightMode(active_flight_mode);
            g_flight_mode_initialized = true;
        }

        RunFlightMode(active_flight_mode);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskLoRaRx(void *pvParameters) {
    if (!LORA_LOGGING_ENABLED) {
        vTaskDelete(NULL);
        return;
    }

    uint8_t buffer[256] = {};

    for (;;) {
        const size_t bytes_read = lora_receive(buffer, sizeof(buffer));
        if (bytes_read > 0U) {
            GCS_ProcessIncomingPacket(buffer, bytes_read);
        }

        vTaskDelay(pdMS_TO_TICKS(LORA_RX_TASK_PERIOD_MS));
    }
}

void TaskTelemetryTx(void *pvParameters) {
    if (!LORA_LOGGING_ENABLED) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        const telemetrydata snapshot = BuildTelemetrySnapshot();
        (void)telemetry_send(snapshot);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskSDLog(void *pvParameters) {
    if (!SD_LOGGING_ENABLED) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(SD_LOG_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        const telemetrydata snapshot = BuildTelemetrySnapshot();
        SD_Logger_LogData(snapshot);

        // Periodically flush the file to prevent data loss on power failure
        static int flush_counter = 0;
        if (++flush_counter >= 20) { // Flush every 1 second at 20Hz
            SD_Logger_Flush();
            flush_counter = 0;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskStatusLED(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(STATUS_LED_TASK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        StatusLED_Update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial); 

    // Shared SPI bus safety: deselect peripherals before driver init.
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);

    StatusLED_Init();
    
    if (!SensorBus_Init()) {
        Serial.println("Sensor I2C bus init failed.");
    }

    pwm_init();
    motormixer_init();
    rx_init();
    arming_init();
    IMU_Init();
    RunStartupIMUCalibrationIfEnabled();
    Barometer_Init();
    Airspeed_Init();
    GPS_Init();
    navigation.restart_mission();

    if (LORA_LOGGING_ENABLED) {        
        if (!lora_init()) {
            Serial.println("LoRa init failed.");
        }
        else {
            Serial.println("LoRa init successful.");
        }
    }
    else {
        Serial.println("LoRa Logging Disabled by config.");
    }

    if (SD_LOGGING_ENABLED) {
        if (SD_Logger_Init()) {
            if (SD_Logger_CreateNewLog()) {
                SD_Logger_WriteHeader();
            }
        } else {
            // If init fails, the logger functions have checks for a valid file handle,
            // so the logging task will safely do nothing.
            Serial.println("SD Card Init Failed. Logging will be disabled.");
        }
    }
    else {
        Serial.println("SD Logging Disabled by config.");
    }

    GCS_LoadPIDTuningsFromConfig();

    xTaskCreatePinnedToCore(
        TaskIMURead,
        "IMU_Task",
        IMU_TASK_STACK_SIZE,
        NULL,
        IMU_TASK_PRIORITY,
        NULL,
        IMU_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        TaskBarometerRead,
        "Baro_Task",
        BARO_TASK_STACK_SIZE,
        NULL,
        BARO_TASK_PRIORITY,
        NULL,
        BARO_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        TaskAirspeedRead,
        "Airspeed_Task",
        AIRSPEED_TASK_STACK_SIZE,
        NULL,
        AIRSPEED_TASK_PRIORITY,
        NULL,
        AIRSPEED_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        TaskGPSRead,            // Function to implement the task
        "GPS_Task",             // Name of the task
        GPS_TASK_STACK_SIZE,    // Stack size in words
        NULL,                   // Task input parameter
        GPS_TASK_PRIORITY,      // Priority
        NULL,                   // Task handle
        GPS_TASK_CORE           // Pin to Core 1
    );

    xTaskCreatePinnedToCore(
        TaskFlightControl,
        "FlightCtrl_Task",
        FLIGHT_CONTROL_TASK_STACK_SIZE,
        NULL,
        FLIGHT_CONTROL_TASK_PRIORITY,
        NULL,
        FLIGHT_CONTROL_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        TaskLoRaRx,
        "LoRa_RX_Task",
        LORA_RX_TASK_STACK_SIZE,
        NULL,
        LORA_RX_TASK_PRIORITY,
        NULL,
        LORA_RX_TASK_CORE
    );

    xTaskCreatePinnedToCore(
        TaskTelemetryTx,
        "Telemetry_Task",
        TELEMETRY_TASK_STACK_SIZE,
        NULL,
        TELEMETRY_TASK_PRIORITY,
        NULL,
        TELEMETRY_TASK_CORE
    );

    if (SD_LOGGING_ENABLED) {
        xTaskCreatePinnedToCore(
            TaskSDLog,
            "SD_Log_Task",
            SD_LOG_TASK_STACK_SIZE,
            NULL,
            SD_LOG_TASK_PRIORITY,
            NULL,
            SD_LOG_TASK_CORE
        );
    }

    xTaskCreatePinnedToCore(
        TaskStatusLED,
        "Status_LED_Task",
        STATUS_LED_TASK_STACK_SIZE,
        NULL,
        STATUS_LED_TASK_PRIORITY,
        NULL,
        STATUS_LED_TASK_CORE
    );
}

void loop() {
    // Core 0 handles the default loop(). We will just use it to print data.

    if (IMU_DEBUG_OUTPUT_ENABLED) {
        if (currentIMU.healthy) {
            Serial.printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f\n", currentIMU.roll, currentIMU.pitch, currentIMU.yaw);
        }
        else {
            Serial.println("IMU reading unhealthy");
        }
    }

    if (AIRSPEED_DEBUG_OUTPUT_ENABLED) {
        if (airspeed_data.healthy) {
            Serial.printf("Airspeed: %6.2f m/s | Pressure: %6.2f Pa\n", airspeed_data.airspeed_mps, airspeed_data.pressure_pa);
        }
        else {
            Serial.println("Airspeed reading unhealthy");
        }
    }

    if (BARO_DEBUG_OUTPUT_ENABLED) {
        if (baro_data.healthy) {
            Serial.printf("Baro Altitude: %6.2f m | Pressure: %7.2f hPa\n", baro_data.altitude, baro_data.pressure);
        }
            else {
                Serial.println("Barometer reading unhealthy");
            }
    }

    if (GPS_DEBUG_OUTPUT_ENABLED) {
        if (gps_data.healthy) {
            Serial.printf("GPS Lat: %f | Lon: %f | Alt: %f | Speed: %f | Heading: %f | Satellites: %d\n", gps_data.latitude, gps_data.longitude, gps_data.altitude, gps_data.speed, gps_data.heading, gps_data.satellites);
        } else {
            Serial.println("GPS reading unhealthy");
        }
    }

    if(RX_DEBUG_OUTPUT_ENABLED) {
        Serial.printf("RC Data - Throttle: %u | Elevator: %u | Aileron: %u | Rudder: %u | FlightMode PWM: %u (Mode: %d)\n",
                      rc_data.throttle_pwm, rc_data.elevator_pwm, rc_data.aileron_pwm, rc_data.rudder_pwm, rc_data.flightmode_pwm, (int)DetermineFlightMode());
    }

    delay(50); 
}
