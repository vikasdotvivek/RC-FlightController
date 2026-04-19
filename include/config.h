//////// config file for all pin assignments, tuning parameters and other constants


// RX_Throttle INPUT_PIN 37    // D37
// RX_Rudder INPUT_PIN 33     // D33  
// RX Elevator INPUT_PIN 32    // D32
// RX_Mode INPUT_PIN 15       // D15


#pragma once
#include <cstdint>

#include "datatypes.h"
#include "flight/flightmodes.h"

///pin definintions


// LoRa-------------
constexpr bool LORA_LOGGING_ENABLED = true;
// LoRa SPI pins
constexpr int SCK_PIN = 5;
constexpr int MOSI_PIN = 19;
constexpr int MISO_PIN = 21;
// LoRa control pins
constexpr int CS_PIN = 26; // Connected to A0
constexpr int RST_PIN = 4;  // Connected to A5
constexpr int IRQ_PIN = 39;  // Connected to A3
// LoRa parameters
constexpr long LORA_FREQ = 915000000L;
constexpr uint8_t SYNC_WORD = 0xF3;
// Shared SPI bus on this airframe: choose highest common stable clock for LoRa + SD.
constexpr uint32_t SPI_BUS_FREQUENCY_HZ = 10000000UL;
constexpr int SPI_BUS_LOCK_TIMEOUT_MS = 50;
constexpr int TELEMETRY_TASK_PERIOD_MS = 500;
constexpr int TELEMETRY_TASK_STACK_SIZE = 4096;
constexpr int TELEMETRY_TASK_PRIORITY = 1;
constexpr int TELEMETRY_TASK_CORE = 1;
// LoRa-------------

// SD Card Logger-------------
constexpr bool SD_LOGGING_ENABLED = false;
constexpr int SD_LOG_TASK_PERIOD_MS = 50;       // 20 Hz logging
constexpr int SD_LOG_TASK_STACK_SIZE = 4096;
constexpr int SD_LOG_TASK_PRIORITY = 1;
constexpr int SD_LOG_TASK_CORE = 1;
constexpr uint8_t SD_SCK = 5;
constexpr uint8_t SD_MOSI = 19;
constexpr uint8_t SD_MISO = 21;
constexpr uint8_t SD_CS = 25;   // Connected to A1
// SD Card Logger-------------

// GPS-------------
constexpr int GPS_UART_NUM = 2;
constexpr unsigned long GPS_BAUD_RATE = 115200UL;
constexpr int GPS_TX_PIN = 8;
constexpr int GPS_RX_PIN = 7;
constexpr int GPS_TIME_ZONE_OFFSET = -4;    //  UTC-4 for Eastern Daylight Time (EDT)
constexpr int GPS_TASK_PERIOD_MS = 50;
constexpr int GPS_TASK_STACK_SIZE = 4096;
constexpr int GPS_TASK_PRIORITY = 1;
constexpr int GPS_TASK_CORE = 1;
constexpr int GPS_SENTENCE_BUFFER_SIZE = 128;
constexpr int GPS_MAX_FIELDS = 20;
constexpr bool GPS_DEBUG_OUTPUT_ENABLED = false;
constexpr bool ROLL_PID_DEBUG_OUTPUT_ENABLED = true;
// GPS-------------

//IMU-------------
constexpr bool IMU_DEBUG_OUTPUT_ENABLED = false;
constexpr int IMU_TASK_PERIOD_MS = 10;
constexpr int IMU_TASK_STACK_SIZE = 4096;
constexpr int IMU_TASK_PRIORITY = 1;
constexpr int IMU_TASK_CORE = 1;
constexpr float IMU_BODY_FRAME_X_SIGN = -1.0f;      // Matches Mukund's ICM-20948 mounting
constexpr float IMU_BODY_FRAME_Y_SIGN = -1.0f;
constexpr float IMU_BODY_FRAME_Z_SIGN = 1.0f;
constexpr float IMU_MAG_OFFSET_X = 0.0f;
constexpr float IMU_MAG_OFFSET_Y = 0.0f;
constexpr float IMU_MAG_OFFSET_Z = 0.0f;
constexpr float IMU_LEVEL_ROLL_OFFSET_DEG = 0.0f;
constexpr float IMU_LEVEL_PITCH_OFFSET_DEG = 0.0f;
constexpr bool IMU_RUN_STARTUP_GYRO_CALIBRATION = false;
constexpr bool IMU_RUN_STARTUP_LEVEL_CALIBRATION = false;
constexpr int IMU_GYRO_CALIBRATION_SAMPLES = 200;
constexpr int IMU_GYRO_CALIBRATION_SAMPLE_DELAY_MS = 5;
constexpr int IMU_LEVEL_CALIBRATION_SAMPLES = 500;
constexpr int IMU_LEVEL_CALIBRATION_SAMPLE_DELAY_MS = 10;
//IMU-------------

//Barometer-------------
constexpr bool BARO_DEBUG_OUTPUT_ENABLED = false;
constexpr float SEALEVELPRESSURE_HPA = 1031.2f; // adjust based on local sea level press
//Barometer-------------

// I2C-------------
constexpr int I2C_SDA_PIN = 22;
constexpr int I2C_SCL_PIN = 20;
constexpr uint32_t I2C_BUS_FREQUENCY_HZ = 400000UL;
constexpr uint16_t SENSOR_I2C_TIMEOUT_MS = 20;
constexpr int SENSOR_I2C_LOCK_TIMEOUT_MS = 20;
constexpr uint32_t SENSOR_RECONNECT_INTERVAL_MS = 1000;
constexpr bool SENSOR_STATUS_LOGGING_ENABLED = true;
// I2C-------------


// Control-------------
constexpr FlightMode DEFAULT_FLIGHT_MODE = FlightMode::Manual;
constexpr float FLIGHT_MODE_PWM_MANUAL_MAX = 1200.0f;   //  these thresholds will need to be tuned based on the actual PWM values from the receiver for each mode
constexpr float FLIGHT_MODE_PWM_STABILIZE_MAX = 1400.0f;
constexpr float FLIGHT_MODE_PWM_ALT_HOLD_MAX = 1600.0f;
constexpr float FLIGHT_MODE_PWM_GLIDE_MAX = 1800.0f;
constexpr int BARO_TASK_PERIOD_MS = 50;
constexpr int BARO_TASK_STACK_SIZE = 4096;
constexpr int BARO_TASK_PRIORITY = 1;
constexpr int BARO_TASK_CORE = 1;
constexpr int FLIGHT_CONTROL_TASK_PERIOD_MS = 100;
constexpr int FLIGHT_CONTROL_TASK_STACK_SIZE = 4096;
constexpr int FLIGHT_CONTROL_TASK_PRIORITY = 1;
constexpr int FLIGHT_CONTROL_TASK_CORE = 1;
// Control-------------


///Motor and servo pins

#define esc_pin 13
#define aileron_pin 12
#define elevator_pin 27
#define rudder_pin 14

//channels

const int esc_channel = 0;
const int aileron_channel = 1;
const int elevator_channel = 2;
const int rudder_channel = 3;

//PWM limits needed for hardware interfacing and safety

//Limits

const float max_roll_angle = 45.0f; 
const float max_pitch_angle = 15.0f; 
const float max_yaw_angle = 30.0f;

///tuning parameters
const float roll_kp = 20.0f;
const float roll_ki = 0.0f;
const float roll_kd = 0.1f;
const float max_roll_output = 500.0f;
const float max_roll_integral = 200.0f;

const float pitch_kp = 20.0f;
const float pitch_ki = 0.0f;
const float pitch_kd = 0.1f;
const float max_pitch_output = 500.0f;
const float max_pitch_integral = 200.0f;

const float yaw_kp = 1.0f;
const float yaw_ki = 0.0f;
const float yaw_kd = 0.1f;
const float max_yaw_output = 500.0f;
const float max_yaw_integral = 200.0f;

const float headingerror_kp = 1.0f;
const float headingerror_ki = 0.0f;
const float headingerror_kd = 0.1f;
const float max_headingerror_output = max_roll_angle;
const float max_headingerror_integral = 10.0f;


const float flight_control_dt_seconds = FLIGHT_CONTROL_TASK_PERIOD_MS / 1000.0f;


/// for althold pid
const float alt_kp = 1.0f;
const float alt_ki = 0.0f;
const float alt_kd = 0.1f;
const float max_alt_output = max_pitch_angle;
const float max_alt_integral = 10.0f;


//althold

const float target_alt_agl = 50.0f; // target altitude relative to takeoff point, in meters


//waypoints
constexpr float WAYPOINT_ACCEPTANCE_RADIUS_METERS = 5.0f;
constexpr float WAYPOINT_CONTROL_DT_SECONDS = 0.1f;
constexpr float WAYPOINT_HEADING_TO_ROLL_KP = 0.5f;
constexpr float WAYPOINT_MIN_GROUND_SPEED_MPS = 1.5f;

const int num_waypoints = 2;
const waypoint missionwaypoints[]= {
    
    {2.0536, 1.2189333333333333333, 100},   //  lat, long, alt_agl in m
    {2, 2, 100},
    
};
