#include "hal/sensors/imu.h"

#include <cmath>

#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#include "config.h"
#include "hal/sensors/sensor_bus.h"

Adafruit_ICM20948 icm;

namespace {

constexpr uint8_t kIMUAddress = ICM20948_I2CADDR_DEFAULT;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kComplementaryFilterGyroWeight = 0.98f;
constexpr float kComplementaryFilterAccelWeight = 0.02f;

uint32_t g_last_time_us = 0;
uint32_t g_last_init_attempt_ms = 0;
bool g_imu_ready = false;
bool g_imu_missing_logged = false;
bool g_orientation_seeded = false;
float g_roll_deg = 0.0f;
float g_pitch_deg = 0.0f;
float g_gyro_bias_x_dps = 0.0f;
float g_gyro_bias_y_dps = 0.0f;
float g_gyro_bias_z_dps = 0.0f;

bool ProbeIMULocked() {
    Wire.beginTransmission(kIMUAddress);
    return Wire.endTransmission(true) == 0;
}

void ResetOrientationState() {
    g_last_time_us = 0;
    g_orientation_seeded = false;
    g_roll_deg = 0.0f;
    g_pitch_deg = 0.0f;
}

float ApplyBodyFrameAxis(float value, float axis_sign) {
    return value * axis_sign;
}

void PopulateIMUData(const sensors_event_t &a,
                     const sensors_event_t &g,
                     const sensors_event_t &m,
                     IMUData_raw &data) {
    data.accel_x = ApplyBodyFrameAxis(a.acceleration.x, IMU_BODY_FRAME_X_SIGN);
    data.accel_y = ApplyBodyFrameAxis(a.acceleration.y, IMU_BODY_FRAME_Y_SIGN);
    data.accel_z = ApplyBodyFrameAxis(a.acceleration.z, IMU_BODY_FRAME_Z_SIGN);

    const float gyro_x_dps = ApplyBodyFrameAxis(g.gyro.x * kRadToDeg, IMU_BODY_FRAME_X_SIGN);
    const float gyro_y_dps = ApplyBodyFrameAxis(g.gyro.y * kRadToDeg, IMU_BODY_FRAME_Y_SIGN);
    const float gyro_z_dps = ApplyBodyFrameAxis(g.gyro.z * kRadToDeg, IMU_BODY_FRAME_Z_SIGN);

    data.gyro_x = gyro_x_dps - g_gyro_bias_x_dps;
    data.gyro_y = gyro_y_dps - g_gyro_bias_y_dps;
    data.gyro_z = gyro_z_dps - g_gyro_bias_z_dps;

    data.mag_x = ApplyBodyFrameAxis(m.magnetic.x - IMU_MAG_OFFSET_X, IMU_BODY_FRAME_X_SIGN);
    data.mag_y = ApplyBodyFrameAxis(m.magnetic.y - IMU_MAG_OFFSET_Y, IMU_BODY_FRAME_Y_SIGN);
    data.mag_z = ApplyBodyFrameAxis(m.magnetic.z - IMU_MAG_OFFSET_Z, IMU_BODY_FRAME_Z_SIGN);
}

void UpdateOrientation(IMUData_raw &data) {
    const uint32_t now_us = micros();
    float dt = 0.0f;
    if (g_last_time_us != 0) {
        dt = static_cast<float>(now_us - g_last_time_us) / 1000000.0f;
    }
    g_last_time_us = now_us;

    const float accel_roll =
        atan2f(data.accel_y, data.accel_z) * 180.0f / PI;
    const float accel_pitch =
        atan2f(-data.accel_x,
               sqrtf((data.accel_y * data.accel_y) + (data.accel_z * data.accel_z))) *
        180.0f / PI;

    if (!g_orientation_seeded || dt <= 0.0f || dt > 0.5f) {
        g_roll_deg = accel_roll;
        g_pitch_deg = accel_pitch;
        g_orientation_seeded = true;
    } else {
        g_roll_deg = (kComplementaryFilterGyroWeight * (g_roll_deg + (data.gyro_x * dt))) +
                     (kComplementaryFilterAccelWeight * accel_roll);
        g_pitch_deg = (kComplementaryFilterGyroWeight * (g_pitch_deg + (data.gyro_y * dt))) +
                      (kComplementaryFilterAccelWeight * accel_pitch);
    }

    data.roll = g_roll_deg - IMU_LEVEL_ROLL_OFFSET_DEG;
    data.pitch = g_pitch_deg - IMU_LEVEL_PITCH_OFFSET_DEG;


    // Yaw will be provided by GPS heading until magnetometer fusion is implemented, so we won't populate it here
    // why is there no yaw
}

void UpdateIMUAvailability(bool available) {
    if (available) {
        if (!g_imu_ready && SENSOR_STATUS_LOGGING_ENABLED) {
            Serial.println("ICM-20948 available.");
        }

        g_imu_ready = true;
        g_imu_missing_logged = false;
        return;
    }

    if ((g_imu_ready || !g_imu_missing_logged) && SENSOR_STATUS_LOGGING_ENABLED) {
        Serial.println("ICM-20948 unavailable. Continuing without IMU data.");
    }

    g_imu_ready = false;
    g_imu_missing_logged = true;
    ResetOrientationState();
}

bool ReadIMUEvents(sensors_event_t &a,
                   sensors_event_t &g,
                   sensors_event_t &temp,
                   sensors_event_t &m) {
    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
        return false;
    }

    if (!ProbeIMULocked()) {
        SensorBus_Unlock();
        UpdateIMUAvailability(false);
        return false;
    }

    icm.getEvent(&a, &g, &temp, &m);
    SensorBus_Unlock();
    return true;
}

bool TryInitializeIMU() {
    const uint32_t now_ms = millis();
    if (g_last_init_attempt_ms != 0 &&
        (now_ms - g_last_init_attempt_ms) < SENSOR_RECONNECT_INTERVAL_MS) {
        return g_imu_ready;
    }

    g_last_init_attempt_ms = now_ms;

    if (!SensorBus_Init()) {
        UpdateIMUAvailability(false);
        return false;
    }

    bool begin_ok = false;
    if (SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
        begin_ok = icm.begin_I2C(kIMUAddress, &Wire);
        if (begin_ok) {
            icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);    // 8G handles flight vibration spikes well
            icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);  // 500 dps is a good flight-control range
            icm.setMagDataRate(AK09916_MAG_DATARATE_50_HZ); // Keep compass data fresh for future yaw fusion
        }
        SensorBus_Unlock();
    }

    if (!begin_ok) {
        UpdateIMUAvailability(false);
        return false;
    }

    ResetOrientationState();
    UpdateIMUAvailability(true);
    return true;
}

} // namespace

void IMU_Init() {
    (void)TryInitializeIMU();
}

bool IMU_Calibrate_Gyro() {
    if (!g_imu_ready && !TryInitializeIMU()) {
        return false;
    }

    Serial.println("Calibrating gyroscope... Keep the aircraft still.");

    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;
    int captured_samples = 0;

    for (int sample_index = 0; sample_index < IMU_GYRO_CALIBRATION_SAMPLES; ++sample_index) {
        sensors_event_t a = {};
        sensors_event_t g = {};
        sensors_event_t temp = {};
        sensors_event_t m = {};

        if (!ReadIMUEvents(a, g, temp, m)) {
            delay(IMU_GYRO_CALIBRATION_SAMPLE_DELAY_MS);
            continue;
        }

        sum_gx += ApplyBodyFrameAxis(g.gyro.x * kRadToDeg, IMU_BODY_FRAME_X_SIGN);
        sum_gy += ApplyBodyFrameAxis(g.gyro.y * kRadToDeg, IMU_BODY_FRAME_Y_SIGN);
        sum_gz += ApplyBodyFrameAxis(g.gyro.z * kRadToDeg, IMU_BODY_FRAME_Z_SIGN);
        ++captured_samples;

        delay(IMU_GYRO_CALIBRATION_SAMPLE_DELAY_MS);
    }

    if (captured_samples == 0) {
        Serial.println("Gyro calibration failed: no valid samples captured.");
        return false;
    }

    g_gyro_bias_x_dps = sum_gx / static_cast<float>(captured_samples);
    g_gyro_bias_y_dps = sum_gy / static_cast<float>(captured_samples);
    g_gyro_bias_z_dps = sum_gz / static_cast<float>(captured_samples);
    ResetOrientationState();

    Serial.printf(
        "Gyro bias [deg/s] -> X: %.2f | Y: %.2f | Z: %.2f\n",
        g_gyro_bias_x_dps,
        g_gyro_bias_y_dps,
        g_gyro_bias_z_dps
    );

    return true;
}

bool IMU_Run_Level_Calibration(float &roll_offset_deg, float &pitch_offset_deg) {
    roll_offset_deg = 0.0f;
    pitch_offset_deg = 0.0f;

    if (!g_imu_ready && !TryInitializeIMU()) {
        return false;
    }

    Serial.println();
    Serial.println("========================================");
    Serial.println("   AIRCRAFT LEVEL CALIBRATION MODE      ");
    Serial.println("========================================");
    Serial.println("Hold the aircraft steady in level flight attitude.");

    for (int seconds_remaining = 5; seconds_remaining > 0; --seconds_remaining) {
        Serial.printf("Starting in %d...\n", seconds_remaining);
        delay(1000);
    }

    float sum_roll = 0.0f;
    float sum_pitch = 0.0f;
    int captured_samples = 0;

    Serial.println("Recording samples...");
    for (int sample_index = 0; sample_index < IMU_LEVEL_CALIBRATION_SAMPLES; ++sample_index) {
        IMUData_raw sample = {};
        IMU_Read(sample);
        if (!sample.healthy) {
            delay(IMU_LEVEL_CALIBRATION_SAMPLE_DELAY_MS);
            continue;
        }

        sum_roll += g_roll_deg;
        sum_pitch += g_pitch_deg;
        ++captured_samples;

        if ((sample_index + 1) % 50 == 0) {
            Serial.print("#");
        }

        delay(IMU_LEVEL_CALIBRATION_SAMPLE_DELAY_MS);
    }

    if (captured_samples == 0) {
        Serial.println();
        Serial.println("Level calibration failed: no valid IMU samples captured.");
        return false;
    }

    roll_offset_deg = sum_roll / static_cast<float>(captured_samples);
    pitch_offset_deg = sum_pitch / static_cast<float>(captured_samples);

    Serial.println();
    Serial.println();
    Serial.println("--- CALIBRATION RESULTS ---");
    Serial.printf("Average Roll Error : %6.2f degrees\n", roll_offset_deg);
    Serial.printf("Average Pitch Error: %6.2f degrees\n", pitch_offset_deg);
    Serial.println("---------------------------");
    Serial.println("Copy these values into include/config.h:");
    Serial.printf("constexpr float IMU_LEVEL_ROLL_OFFSET_DEG = %6.2ff;\n", roll_offset_deg);
    Serial.printf("constexpr float IMU_LEVEL_PITCH_OFFSET_DEG = %6.2ff;\n", pitch_offset_deg);
    Serial.println("========================================");
    Serial.println();

    return true;
}

void IMU_Read(IMUData_raw &data) {
    if (!g_imu_ready && !TryInitializeIMU()) {
        data.healthy = false;
        return;
    }

    sensors_event_t a = {};
    sensors_event_t g = {};
    sensors_event_t temp = {};
    sensors_event_t m = {};

    if (!ReadIMUEvents(a, g, temp, m)) {
        data.healthy = false;
        return;
    }

    PopulateIMUData(a, g, m, data);
    UpdateOrientation(data);
    data.healthy = true;
    UpdateIMUAvailability(true);
}