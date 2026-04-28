#include "hal/sensors/imu.h"

#include <cmath>

#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#include "config.h"
#include "hal/sensors/sensor_bus.h"
#include "math/utils.h"

Adafruit_ICM20948 icm;

namespace
{

    constexpr uint8_t kIMUAddress = ICM20948_I2CADDR_DEFAULT;
    constexpr float kRadToDeg = 57.2957795f;
    constexpr float kFilterTimeConstantXY = 0.5f; // Seconds to trust gyro over accel
    constexpr float kFilterTimeConstantZ = 0.5f;  // Seconds to trust gyro over mag

    uint32_t g_last_time_us = 0;
    uint32_t g_last_mag_time_us = 0;
    float g_last_mag_x = 0.0f;
    float g_last_mag_y = 0.0f;
    float g_last_mag_z = 0.0f;

    uint32_t g_last_init_attempt_ms = 0;
    bool g_imu_ready = false;
    bool g_imu_missing_logged = false;
    bool g_orientation_seeded = false;
    float g_roll_deg = 0.0f;
    float g_pitch_deg = 0.0f;
    float g_yaw_deg = 0.0f;
    float g_gyro_bias_x_dps = 0.0f;
    float g_gyro_bias_y_dps = 0.0f;
    float g_gyro_bias_z_dps = 0.0f;

    bool ProbeIMULocked() {
        Wire.beginTransmission(kIMUAddress);
        return Wire.endTransmission(true) == 0;
    }

    void ResetOrientationState() {
        g_last_time_us = 0;
        g_last_mag_time_us = 0;
        g_last_mag_x = 0.0f;
        g_last_mag_y = 0.0f;
        g_last_mag_z = 0.0f;
        g_orientation_seeded = false;
        g_roll_deg = 0.0f;
        g_pitch_deg = 0.0f;
        g_yaw_deg = 0.0f;
    }

    float ApplyBodyFrameAxis(float value, float axis_sign) {
        return value * axis_sign;
    }

    void PopulateIMUData(const sensors_event_t &a,
                         const sensors_event_t &g,
                         const sensors_event_t &m,
                         IMUData_raw &data)
    {
        data.accel_x = ApplyBodyFrameAxis(a.acceleration.x, IMU_BODY_FRAME_X_SIGN);
        data.accel_y = ApplyBodyFrameAxis(a.acceleration.y, IMU_BODY_FRAME_Y_SIGN);
        data.accel_z = ApplyBodyFrameAxis(a.acceleration.z, IMU_BODY_FRAME_Z_SIGN);

        const float gyro_x_dps = ApplyBodyFrameAxis(g.gyro.x * kRadToDeg, IMU_BODY_FRAME_X_SIGN);
        const float gyro_y_dps = ApplyBodyFrameAxis(g.gyro.y * kRadToDeg, IMU_BODY_FRAME_Y_SIGN);
        const float gyro_z_dps = ApplyBodyFrameAxis(g.gyro.z * kRadToDeg, IMU_BODY_FRAME_Z_SIGN);

        data.gyro_x = gyro_x_dps - g_gyro_bias_x_dps;
        data.gyro_y = gyro_y_dps - g_gyro_bias_y_dps;
        data.gyro_z = gyro_z_dps - g_gyro_bias_z_dps;

        // FIX 1: Magnetometer axis mapping. The AK09916 inside the ICM20948 has a different 
        // physical orientation than the gyro/accel. By default: Mag X = Gyro Y, Mag Y = Gyro X, Mag Z = -Gyro Z.
        // If your mag heading moves backward during rotation, swap these assignments.
        float raw_mag_x = m.magnetic.x; // Try mapping to m.magnetic.y if axes fight
        float raw_mag_y = m.magnetic.y; // Try mapping to m.magnetic.x if axes fight
        float raw_mag_z = m.magnetic.z; // Try mapping to -m.magnetic.z if axes fight

        data.mag_x = ApplyBodyFrameAxis((raw_mag_x - IMU_MAG_OFFSET_X) * IMU_MAG_SCALE_X, IMU_BODY_FRAME_X_SIGN);
        data.mag_y = ApplyBodyFrameAxis((raw_mag_y - IMU_MAG_OFFSET_Y) * IMU_MAG_SCALE_Y, IMU_BODY_FRAME_Y_SIGN);
        data.mag_z = ApplyBodyFrameAxis((raw_mag_z - IMU_MAG_OFFSET_Z) * IMU_MAG_SCALE_Z, IMU_BODY_FRAME_Z_SIGN);
    }

    void UpdateOrientation(IMUData_raw &data) {
        const uint32_t now_us = micros();
        float dt = 0.0f;
        if (g_last_time_us != 0)
        {
            dt = static_cast<float>(now_us - g_last_time_us) / 1000000.0f;
        }
        g_last_time_us = now_us;

        const float accel_roll =
            atan2f(data.accel_y, data.accel_z) * 180.0f / PI;
        const float accel_pitch =
            atan2f(-data.accel_x,
                   sqrtf((data.accel_y * data.accel_y) + (data.accel_z * data.accel_z))) *
            180.0f / PI;

        // FIX 2: Level the angles BEFORE computing tilt compensation. 
        // Failing to do this bends your horizontal magnetic projection.
        const float leveled_roll = g_roll_deg - IMU_LEVEL_ROLL_OFFSET_DEG;
        const float leveled_pitch = g_pitch_deg - IMU_LEVEL_PITCH_OFFSET_DEG;

        const float roll_rad = leveled_roll * PI / 180.0f;
        const float pitch_rad = leveled_pitch * PI / 180.0f;
        
        const float cos_roll = cosf(roll_rad);
        const float sin_roll = sinf(roll_rad);
        const float cos_pitch = cosf(pitch_rad);
        const float sin_pitch = sinf(pitch_rad);

        const float mag_x_h = data.mag_x * cos_pitch + data.mag_y * sin_roll * sin_pitch + data.mag_z * cos_roll * sin_pitch;
        const float mag_y_h = data.mag_y * cos_roll - data.mag_z * sin_roll;
        const float mag_heading = atan2f(-mag_y_h, mag_x_h) * 180.0f / PI;

        // FIX 3: Stop using floating point equality. Use an epsilon threshold for data freshness.
        const float epsilon = 0.001f;
        const bool mag_is_fresh = (std::abs(data.mag_x - g_last_mag_x) > epsilon || 
                                   std::abs(data.mag_y - g_last_mag_y) > epsilon || 
                                   std::abs(data.mag_z - g_last_mag_z) > epsilon);

        if (!g_orientation_seeded || dt <= 0.0f || dt > 0.5f) {
            g_roll_deg = accel_roll;
            g_pitch_deg = accel_pitch;
            g_yaw_deg = mag_heading;
            g_orientation_seeded = true;

            g_last_mag_time_us = now_us;
            g_last_mag_x = data.mag_x;
            g_last_mag_y = data.mag_y;
            g_last_mag_z = data.mag_z;
        }
        else {
            // Dynamic filter weights based on actual loop time
            const float alpha_xy = kFilterTimeConstantXY / (kFilterTimeConstantXY + dt);

            g_roll_deg = (alpha_xy * (g_roll_deg + (data.gyro_x * dt))) +
                         ((1.0f - alpha_xy) * accel_roll);
            g_pitch_deg = (alpha_xy * (g_pitch_deg + (data.gyro_y * dt))) +
                          ((1.0f - alpha_xy) * accel_pitch);

            // FIX 4: Proper Earth-frame yaw kinematics. 
            // Body-Z gyro rate != Earth yaw rate unless the aircraft is perfectly flat.
            // Formula: psi_dot = (q * sin(phi) + r * cos(phi)) / cos(theta)
            float safe_cos_pitch = cos_pitch;
            if (std::abs(safe_cos_pitch) < 0.01f) {
                safe_cos_pitch = (safe_cos_pitch < 0.0f) ? -0.01f : 0.01f; // Prevent division by zero at 90 deg pitch
            }
            
            const float yaw_rate = (data.gyro_y * sin_roll + data.gyro_z * cos_roll) / safe_cos_pitch;
            g_yaw_deg += yaw_rate * dt;
            g_yaw_deg = math::wrap_heading_error(g_yaw_deg);

            // Only pull towards compass when fresh data arrives
            if (mag_is_fresh) {
                const float mag_dt = (g_last_mag_time_us != 0) ? (static_cast<float>(now_us - g_last_mag_time_us) / 1000000.0f) : 0.0f;
                g_last_mag_time_us = now_us;
                g_last_mag_x = data.mag_x;
                g_last_mag_y = data.mag_y;
                g_last_mag_z = data.mag_z;

                if (mag_dt > 0.0f) {
                    const float alpha_z = kFilterTimeConstantZ / (kFilterTimeConstantZ + mag_dt);
                    const float yaw_error = math::wrap_heading_error(mag_heading - g_yaw_deg);
                    g_yaw_deg = math::wrap_heading_error(g_yaw_deg + ((1.0f - alpha_z) * yaw_error));
                }
            }
        }

        data.roll = g_roll_deg - IMU_LEVEL_ROLL_OFFSET_DEG;
        data.pitch = g_pitch_deg - IMU_LEVEL_PITCH_OFFSET_DEG;
        data.yaw = g_yaw_deg;
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
                       sensors_event_t &m)
    {
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
                icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);    
                icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);  
                icm.setMagDataRate(AK09916_MAG_DATARATE_50_HZ); 
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

    double sum_gx = 0.0;
    double sum_gy = 0.0;
    double sum_gz = 0.0;
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

        sum_gx += static_cast<double>(ApplyBodyFrameAxis(g.gyro.x * kRadToDeg, IMU_BODY_FRAME_X_SIGN));
        sum_gy += static_cast<double>(ApplyBodyFrameAxis(g.gyro.y * kRadToDeg, IMU_BODY_FRAME_Y_SIGN));
        sum_gz += static_cast<double>(ApplyBodyFrameAxis(g.gyro.z * kRadToDeg, IMU_BODY_FRAME_Z_SIGN));
        ++captured_samples;

        delay(IMU_GYRO_CALIBRATION_SAMPLE_DELAY_MS);
    }

    if (captured_samples == 0) {
        Serial.println("Gyro calibration failed: no valid samples captured.");
        return false;
    }

    g_gyro_bias_x_dps = static_cast<float>(sum_gx / captured_samples);
    g_gyro_bias_y_dps = static_cast<float>(sum_gy / captured_samples);
    g_gyro_bias_z_dps = static_cast<float>(sum_gz / captured_samples);
    ResetOrientationState();

    Serial.printf(
        "Gyro bias [deg/s] -> X: %.2f | Y: %.2f | Z: %.2f\n",
        g_gyro_bias_x_dps,
        g_gyro_bias_y_dps,
        g_gyro_bias_z_dps);

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

    for (int seconds_remaining = 5; seconds_remaining > 0; --seconds_remaining)  {
        Serial.printf("Starting in %d...\n", seconds_remaining);
        delay(1000);
    }

    double sum_roll = 0.0;
    double sum_pitch = 0.0;
    int captured_samples = 0;

    Serial.println("Recording samples...");
    for (int sample_index = 0; sample_index < IMU_LEVEL_CALIBRATION_SAMPLES; ++sample_index) {
        IMUData_raw sample = {};
        IMU_Read(sample);
        if (!sample.healthy) {
            delay(IMU_LEVEL_CALIBRATION_SAMPLE_DELAY_MS);
            continue;
        }

        sum_roll += static_cast<double>(g_roll_deg);
        sum_pitch += static_cast<double>(g_pitch_deg);
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

    roll_offset_deg = static_cast<float>(sum_roll / captured_samples);
    pitch_offset_deg = static_cast<float>(sum_pitch / captured_samples);

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

bool IMU_Run_Mag_Calibration(float &offset_x, float &offset_y, float &offset_z,
                             float &scale_x, float &scale_y, float &scale_z)
{
    offset_x = 0.0f;
    offset_y = 0.0f;
    offset_z = 0.0f;
    scale_x = 1.0f;
    scale_y = 1.0f;
    scale_z = 1.0f;

    if (!g_imu_ready && !TryInitializeIMU()) {
        return false;
    }

    Serial.println();
    Serial.println("========================================");
    Serial.println("   MAGNETOMETER CALIBRATION MODE        ");
    Serial.println("========================================");
    Serial.println("Rotate and tumble the aircraft in ALL directions (figure 8).");

    for (int seconds_remaining = 5; seconds_remaining > 0; --seconds_remaining) {
        Serial.printf("Starting in %d...\n", seconds_remaining);
        delay(1000);
    }

    float min_x = 100000.0f, max_x = -100000.0f;
    float min_y = 100000.0f, max_y = -100000.0f;
    float min_z = 100000.0f, max_z = -100000.0f;

    int captured_samples = 0;
    constexpr int kMagCalibrationSamples = 1500;
    constexpr int kMagSampleDelayMs = 20; 

    Serial.println("Recording samples... Keep tumbling!");
    for (int sample_index = 0; sample_index < kMagCalibrationSamples; ++sample_index) {
        sensors_event_t a = {};
        sensors_event_t g = {};
        sensors_event_t temp = {};
        sensors_event_t m = {};

        if (!ReadIMUEvents(a, g, temp, m)) {
            delay(kMagSampleDelayMs);
            continue;
        }

        if (m.magnetic.x != 0.0f || m.magnetic.y != 0.0f || m.magnetic.z != 0.0f) {
            if (m.magnetic.x < min_x)
                min_x = m.magnetic.x;
            if (m.magnetic.x > max_x)
                max_x = m.magnetic.x;
            if (m.magnetic.y < min_y)
                min_y = m.magnetic.y;
            if (m.magnetic.y > max_y)
                max_y = m.magnetic.y;
            if (m.magnetic.z < min_z)
                min_z = m.magnetic.z;
            if (m.magnetic.z > max_z)
                max_z = m.magnetic.z;
            ++captured_samples;
        }

        if ((sample_index + 1) % 50 == 0) {
            Serial.print("#");
        }

        delay(kMagSampleDelayMs);
    }

    if (captured_samples == 0) {
        Serial.println("\nMag calibration failed: no valid samples captured.");
        return false;
    }

    offset_x = (max_x + min_x) / 2.0f;
    offset_y = (max_y + min_y) / 2.0f;
    offset_z = (max_z + min_z) / 2.0f;

    const float radius_x = (max_x - min_x) / 2.0f;
    const float radius_y = (max_y - min_y) / 2.0f;
    const float radius_z = (max_z - min_z) / 2.0f;
    const float avg_radius = (radius_x + radius_y + radius_z) / 3.0f;

    if (radius_x > 0.0f)
        scale_x = avg_radius / radius_x;
    if (radius_y > 0.0f)
        scale_y = avg_radius / radius_y;
    if (radius_z > 0.0f)
        scale_z = avg_radius / radius_z;

    Serial.println("\n\n--- CALIBRATION RESULTS ---");
    Serial.printf("Hard-Iron Offsets : X: %6.2f | Y: %6.2f | Z: %6.2f\n", offset_x, offset_y, offset_z);
    Serial.printf("Soft-Iron Scales  : X: %6.2f | Y: %6.2f | Z: %6.2f\n", scale_x, scale_y, scale_z);
    Serial.println("---------------------------");
    Serial.println("Copy these values into include/config.h:");
    Serial.printf("constexpr float IMU_MAG_OFFSET_X = %6.2ff;\n", offset_x);
    Serial.printf("constexpr float IMU_MAG_OFFSET_Y = %6.2ff;\n", offset_y);
    Serial.printf("constexpr float IMU_MAG_OFFSET_Z = %6.2ff;\n", offset_z);
    Serial.printf("constexpr float IMU_MAG_SCALE_X = %6.3ff;\n", scale_x);
    Serial.printf("constexpr float IMU_MAG_SCALE_Y = %6.3ff;\n", scale_y);
    Serial.printf("constexpr float IMU_MAG_SCALE_Z = %6.3ff;\n", scale_z);
    Serial.println("========================================\n");

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

    // Serial.printf("Mag X: %.2f | Mag Y: %.2f | Mag Z: %.2f\n", 
    //           m.magnetic.x, m.magnetic.y, m.magnetic.z);
}