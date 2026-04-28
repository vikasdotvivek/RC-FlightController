//////// config file for all pin assignments, tuning parameters and other constants
#pragma once
#include <cstdint>

#include "datatypes.h"
#include "flight/flightmodes.h"

// Set to false to bypass the stick-gesture arm/disarm requirement (bench testing)
constexpr bool ARMING_ENABLED = false;

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
constexpr int LORA_RX_TASK_PERIOD_MS = 20;
constexpr int LORA_RX_TASK_STACK_SIZE = 4096;
constexpr int LORA_RX_TASK_PRIORITY = 1;
constexpr int LORA_RX_TASK_CORE = 0;
// LoRa-------------

// SD Card Logger-------------
constexpr bool SD_LOGGING_ENABLED = true;
constexpr int SD_LOG_TASK_PERIOD_MS = 50;       // 20 Hz logging
constexpr int SD_LOG_TASK_STACK_SIZE = 4096;
constexpr int SD_LOG_TASK_PRIORITY = 1;
constexpr int SD_LOG_TASK_CORE = 1;
constexpr uint8_t SD_SCK = 5;
constexpr uint8_t SD_MOSI = 19;
constexpr uint8_t SD_MISO = 21;
constexpr uint8_t SD_CS = 25;   // Connected to A1
// SD Card Logger-------------

// Status LED-------------
constexpr int STATUS_LED_TASK_PERIOD_MS = 100;
constexpr int STATUS_LED_TASK_STACK_SIZE = 4096;
constexpr int STATUS_LED_TASK_PRIORITY = 1;
constexpr int STATUS_LED_TASK_CORE = 1;
constexpr uint32_t STATUS_LED_CYCLE_PERIOD_MS = 1200;
constexpr uint32_t STATUS_LED_BLINK_PERIOD_MS = 500;
constexpr uint8_t STATUS_LED_BRIGHTNESS = 32;
// Status LED-------------

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
constexpr bool ROLL_PID_DEBUG_OUTPUT_ENABLED = false;
// GPS-------------

// IMU-------------
constexpr bool IMU_DEBUG_OUTPUT_ENABLED = false;
constexpr int IMU_TASK_PERIOD_MS = 10;
constexpr int IMU_TASK_STACK_SIZE = 4096;
constexpr int IMU_TASK_PRIORITY = 1;
constexpr int IMU_TASK_CORE = 1;
constexpr float IMU_BODY_FRAME_X_SIGN = -1.0f;
constexpr float IMU_BODY_FRAME_Y_SIGN = -1.0f;
constexpr float IMU_BODY_FRAME_Z_SIGN = 1.0f;
constexpr float IMU_MAG_OFFSET_X = -41.17f; //  Ken will have to calibrate these for his own board using the IMU_Run_Mag_Calibration function and update these values accordingly
constexpr float IMU_MAG_OFFSET_Y = -45.83f;
constexpr float IMU_MAG_OFFSET_Z = -86.10f;
constexpr float IMU_MAG_SCALE_X =  1.073f;
constexpr float IMU_MAG_SCALE_Y =  0.976f;
constexpr float IMU_MAG_SCALE_Z =  0.959f;
constexpr float IMU_LEVEL_ROLL_OFFSET_DEG = 0.0f;
constexpr float IMU_LEVEL_PITCH_OFFSET_DEG = 0.0f;
constexpr bool IMU_RUN_STARTUP_GYRO_CALIBRATION = false;
// Startup magnetometer calibration is mutually exclusive with gyro and level calibration.
// If IMU_RUN_MAG_CALIBRATION is true, keep IMU_RUN_STARTUP_GYRO_CALIBRATION and
// IMU_RUN_STARTUP_LEVEL_CALIBRATION false.
constexpr bool IMU_RUN_STARTUP_LEVEL_CALIBRATION = false;
constexpr bool IMU_RUN_MAG_CALIBRATION = false;
constexpr int IMU_GYRO_CALIBRATION_SAMPLES = 200;
constexpr int IMU_GYRO_CALIBRATION_SAMPLE_DELAY_MS = 5;
constexpr int IMU_LEVEL_CALIBRATION_SAMPLES = 500;
constexpr int IMU_LEVEL_CALIBRATION_SAMPLE_DELAY_MS = 10;
// IMU-------------

//Barometer-------------
constexpr bool BARO_DEBUG_OUTPUT_ENABLED = false;
constexpr float SEALEVELPRESSURE_HPA = 1031.2f; // adjust based on local sea level press
//Barometer-------------

//Airspeed-------------
constexpr bool AIRSPEED_DEBUG_OUTPUT_ENABLED = false;
constexpr int AIRSPEED_TASK_PERIOD_MS = 50;
constexpr int AIRSPEED_TASK_STACK_SIZE = 4096;
constexpr int AIRSPEED_TASK_PRIORITY = 1;
constexpr int AIRSPEED_TASK_CORE = 1;
constexpr uint8_t AIRSPEED_I2C_ADDRESS = 0x28; // MS4525DO default address
//Airspeed-------------

// I2C-------------
constexpr int I2C_SDA_PIN = 22;
constexpr int I2C_SCL_PIN = 20;
constexpr uint32_t I2C_BUS_FREQUENCY_HZ = 400000UL;
constexpr uint16_t SENSOR_I2C_TIMEOUT_MS = 20;
constexpr int SENSOR_I2C_LOCK_TIMEOUT_MS = 20;
constexpr uint32_t SENSOR_RECONNECT_INTERVAL_MS = 1000;
constexpr bool SENSOR_STATUS_LOGGING_ENABLED = true;
// I2C-------------


//SPI and Telemtry-------------
constexpr int SPI_BUS_LOCK_TIMEOUT_MS = 250;
// Shared SPI bus on this airframe: choose highest common stable clock for LoRa + SD.
constexpr uint32_t SPI_BUS_FREQUENCY_HZ = 10000000UL;
constexpr int TELEMETRY_TASK_PERIOD_MS = 500;
constexpr int TELEMETRY_TASK_STACK_SIZE = 4096;
constexpr int TELEMETRY_TASK_PRIORITY = 1;
constexpr int TELEMETRY_TASK_CORE = 1;
//SPI and Telemetry-------------

// Control-------------
constexpr FlightMode DEFAULT_FLIGHT_MODE = FlightMode::Manual;
constexpr unsigned int FLIGHT_MODE_PWM_MANUAL_MAX = 1500;   //  these thresholds will need to be tuned based on the actual PWM values from the receiver for each mode
constexpr unsigned int FLIGHT_MODE_PWM_STABILIZE_MAX = 1850;
constexpr unsigned int FLIGHT_MODE_PWM_ALT_HOLD_MAX = 2000;
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
constexpr bool PWM_DEBUG_OUTPUT_ENABLED = false;
#define esc_pin 13
#define aileron_pin 12
#define elevator_pin 27
#define rudder_pin 14

///rx pins
constexpr bool RX_DEBUG_OUTPUT_ENABLED = false;
#define USE_ESC
#define USE_ELEVATOR
#define USE_RUDDER
//#define USE_AILERON
#define rx_esc_pin 37
#define rx_elevator_pin 32
#define rx_rudder_pin 33
#define rx_mode_pin 15
#define rx_aileron_pin 33

//channels

const int esc_channel = 0;
const int aileron_channel = 1;
const int elevator_channel = 2;
const int rudder_channel = 3;

//pwm settings
const int pwm_freq = 50; // 50 Hz for standard servos
const int pwm_resolution = 16; // 16-bit resolution for finer control


//PWM limits needed for hardware interfacing and safety

//esc/throttle
constexpr float throttle_slope = 6.44;   // 10 us per percent throttle
constexpr int throttle_int = 1171; ///1171 microseconds corresponds to 0% throttle
constexpr int throttle_safe = 500; //to cutoff esc

//aileron
constexpr float aileron_slope = 10.0f;   // microsseconds per degree of deflection
constexpr int aileron_int = 1500; // 1500 microseconds corresponds to neutral posi (0 degrees) deflection

//elevator
constexpr float elevator_slope = -32.81;   
constexpr int elevator_int = 1547; 

// rudder
constexpr float rudder_slope = 12.32;
constexpr int rudder_int = 1547;

//PWM limits--------------------


//Control Surface Hardware Limits
constexpr float aileron_max_deflection_deg = 30.0f; /// maximum deflection in degrees for aileron
constexpr float aileron_min_deflection_deg = -30.0f;   // minimum deflection in degrees for aileron
constexpr float elevator_max_deflection_deg = 15.0f;
constexpr float elevator_min_deflection_deg = -10.0f; 
constexpr float rudder_max_deflection_deg = 40.0f;   
constexpr float rudder_min_deflection_deg = -40.0f; 
//control surface hardware limits--------------------


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
