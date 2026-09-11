/**
 * BNO085 Hardware Abstraction Layer
 *
 * Wraps the Adafruit BNO08x library and the native Hillcrest SH-2 C-API to communicate
 * with the BNO085 over a shared I2C bus. Extracts quaternions, converts them to Euler
 * angles, keeps an absolute heading for navigation, and exposes a separately tared yaw.
 */
#include "hal/sensors/bno085.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <sh2.h>
#include <cmath>
#include "config.h"
#include "hal/sensors/sensor_bus.h"

namespace
{
    Adafruit_BNO08x bno08x;
    bool g_bno_ready = false;
    uint32_t g_last_init_attempt_ms = 0;
    uint32_t g_last_rotation_event_ms = 0;
    uint8_t g_last_calibration_status = 0;
    float g_yaw_tare_offset = 0.0f;

    bool g_background_calibration_enabled = false;
    float g_mag_bias_x = 0.0f;
    float g_mag_bias_y = 0.0f;
    float g_mag_bias_z = 0.0f;
    float g_gyro_bias_x = 0.0f;
    float g_gyro_bias_y = 0.0f;
    float g_gyro_bias_z = 0.0f;

    bool g_initial_yaw_tared = false;
    IMUData_raw g_cached_data = {};

    float Wrap180(float angle_deg)
    {
        while (angle_deg > 180.0f)
            angle_deg -= 360.0f;
        while (angle_deg < -180.0f)
            angle_deg += 360.0f;
        return angle_deg;
    }

    float Normalize360(float angle_deg)
    {
        while (angle_deg < 0.0f)
            angle_deg += 360.0f;
        while (angle_deg >= 360.0f)
            angle_deg -= 360.0f;
        return angle_deg;
    }

    bool EnableCoreReportsLocked()
    {
        const bool rotation_ok = bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
        const bool accel_ok = bno08x.enableReport(SH2_ACCELEROMETER, 10000);
        const bool gyro_ok = bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
        const bool mag_ok = bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000);
        return rotation_ok && accel_ok && gyro_ok && mag_ok;
    }

    bool EnableCalibrationReportsLocked()
    {
        const bool mag_uncal_ok = bno08x.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 20000);
        const bool gyro_uncal_ok = bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 20000);
        return mag_uncal_ok && gyro_uncal_ok;
    }

    void MarkBNOUnavailable()
    {
        g_bno_ready = false;
        g_last_rotation_event_ms = 0;
    }

    bool TryInitializeBNO()
    {
        const uint32_t now_ms = millis();
        if (g_last_init_attempt_ms != 0 &&
            (now_ms - g_last_init_attempt_ms) < SENSOR_RECONNECT_INTERVAL_MS)
        {
            return g_bno_ready;
        }
        g_last_init_attempt_ms = now_ms;

        if (!SensorBus_Init() || !SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
        {
            MarkBNOUnavailable();
            return false;
        }

        const bool begin_ok = bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire);
        bool reports_ok = false;
        if (begin_ok)
        {
            reports_ok = EnableCoreReportsLocked();
            if (reports_ok && g_background_calibration_enabled)
            {
                reports_ok = EnableCalibrationReportsLocked();
            }
        }

        SensorBus_Unlock();

        const bool ready = begin_ok && reports_ok;
        if (ready && !g_bno_ready && SENSOR_STATUS_LOGGING_ENABLED)
        {
            Serial.println("BNO085 available and reports enabled.");
        }
        else if (!ready && SENSOR_STATUS_LOGGING_ENABLED)
        {
            Serial.println("BNO085 unavailable or report configuration failed.");
        }

        g_bno_ready = ready;
        if (!ready)
        {
            g_last_rotation_event_ms = 0;
        }
        return ready;
    }
} // namespace

void BNO085_Init()
{
    (void)TryInitializeBNO();
}

void BNO085_Read(IMUData_raw &data)
{
    if (!g_bno_ready && !TryInitializeBNO())
    {
        data = g_cached_data;
        data.healthy = false;
        return;
    }

    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
    {
        data = g_cached_data;
        data.healthy = false;
        return;
    }

    if (bno08x.wasReset())
    {
        bool reports_ok = EnableCoreReportsLocked();
        if (reports_ok && g_background_calibration_enabled)
        {
            reports_ok = EnableCalibrationReportsLocked();
        }

        if (!reports_ok)
        {
            SensorBus_Unlock();
            MarkBNOUnavailable();
            data = g_cached_data;
            data.healthy = false;
            return;
        }

        // A reset invalidates our freshness timestamp until a new rotation vector arrives.
        g_last_rotation_event_ms = 0;
    }

    sh2_SensorValue_t sensorValue;
    while (bno08x.getSensorEvent(&sensorValue))
    {
        switch (sensorValue.sensorId)
        {
        case SH2_ACCELEROMETER:
            g_cached_data.accel_x = sensorValue.un.accelerometer.x * IMU_BODY_FRAME_X_SIGN;
            g_cached_data.accel_y = sensorValue.un.accelerometer.y * IMU_BODY_FRAME_Y_SIGN;
            g_cached_data.accel_z = sensorValue.un.accelerometer.z * IMU_BODY_FRAME_Z_SIGN;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            g_cached_data.gyro_x = (sensorValue.un.gyroscope.x * 57.2957795f) * IMU_BODY_FRAME_X_SIGN;
            g_cached_data.gyro_y = (sensorValue.un.gyroscope.y * 57.2957795f) * IMU_BODY_FRAME_Y_SIGN;
            g_cached_data.gyro_z = (sensorValue.un.gyroscope.z * 57.2957795f) * IMU_BODY_FRAME_Z_SIGN;
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            g_cached_data.mag_x = sensorValue.un.magneticField.x * IMU_MAG_SENSOR_ALIGN_X_SIGN;
            g_cached_data.mag_y = sensorValue.un.magneticField.y * IMU_MAG_SENSOR_ALIGN_Y_SIGN;
            g_cached_data.mag_z = sensorValue.un.magneticField.z * IMU_MAG_SENSOR_ALIGN_Z_SIGN;
            break;

        case SH2_MAGNETIC_FIELD_UNCALIBRATED:
            g_mag_bias_x = sensorValue.un.magneticFieldUncal.biasX * IMU_MAG_SENSOR_ALIGN_X_SIGN;
            g_mag_bias_y = sensorValue.un.magneticFieldUncal.biasY * IMU_MAG_SENSOR_ALIGN_Y_SIGN;
            g_mag_bias_z = sensorValue.un.magneticFieldUncal.biasZ * IMU_MAG_SENSOR_ALIGN_Z_SIGN;
            break;

        case SH2_GYROSCOPE_UNCALIBRATED:
            g_gyro_bias_x = sensorValue.un.gyroscopeUncal.biasX * IMU_BODY_FRAME_X_SIGN;
            g_gyro_bias_y = sensorValue.un.gyroscopeUncal.biasY * IMU_BODY_FRAME_Y_SIGN;
            g_gyro_bias_z = sensorValue.un.gyroscopeUncal.biasZ * IMU_BODY_FRAME_Z_SIGN;
            break;

        case SH2_ROTATION_VECTOR:
        {
            g_last_calibration_status = sensorValue.status;

            const float qr = sensorValue.un.rotationVector.real;
            const float qi = sensorValue.un.rotationVector.i;
            const float qj = sensorValue.un.rotationVector.j;
            const float qk = sensorValue.un.rotationVector.k;
            const float ysqr = qj * qj;

            const float t0 = 2.0f * (qr * qi + qj * qk);
            const float t1 = 1.0f - 2.0f * (qi * qi + ysqr);
            const float raw_roll = atan2f(t0, t1) * 57.2957795f;

            float t2 = 2.0f * (qr * qj - qk * qi);
            t2 = t2 > 1.0f ? 1.0f : (t2 < -1.0f ? -1.0f : t2);
            const float raw_pitch = asinf(t2) * 57.2957795f;

            const float t3 = 2.0f * (qr * qk + qi * qj);
            const float t4 = 1.0f - 2.0f * (ysqr + qk * qk);
            const float raw_yaw = atan2f(t3, t4) * 57.2957795f;

            g_cached_data.roll = (raw_roll * IMU_BODY_FRAME_X_SIGN) - IMU_LEVEL_ROLL_OFFSET_DEG;
            g_cached_data.pitch = (raw_pitch * IMU_BODY_FRAME_Y_SIGN) - IMU_LEVEL_PITCH_OFFSET_DEG;

            const float signed_yaw = raw_yaw * IMU_BODY_FRAME_Z_SIGN;
            g_cached_data.heading = Normalize360(signed_yaw);

            if (!g_initial_yaw_tared)
            {
                g_yaw_tare_offset = signed_yaw;
                g_initial_yaw_tared = true;
            }

            g_cached_data.yaw = Wrap180(signed_yaw - g_yaw_tare_offset);
            g_last_rotation_event_ms = millis();
            break;
        }
        }
    }

    SensorBus_Unlock();

    const uint32_t now_ms = millis();
    const bool attitude_fresh =
        (g_last_rotation_event_ms != 0) &&
        ((now_ms - g_last_rotation_event_ms) <= BNO085_DATA_TIMEOUT_MS);

    if (!attitude_fresh)
    {
        MarkBNOUnavailable();
        data = g_cached_data;
        data.healthy = false;
        return;
    }

    g_cached_data.healthy = true;
    data = g_cached_data;
}

bool BNO085_EnableBackgroundCalibration()
{
    if (!g_bno_ready)
        return false;

    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
        return false;

    const bool cal_command_ok =
        (sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG) == SH2_OK);
    const bool reports_ok = cal_command_ok && EnableCalibrationReportsLocked();
    g_background_calibration_enabled = reports_ok;

    SensorBus_Unlock();
    return reports_ok;
}

bool BNO085_SaveCalibrationToFlash()
{
    if (!g_bno_ready)
        return false;

    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS)))
        return false;

    const bool save_ok = (sh2_saveDcdNow() == SH2_OK);
    if (save_ok)
    {
        // Pump the event loop so the queued SH-2 command is actually transmitted.
        sh2_SensorValue_t dummy;
        for (int i = 0; i < 3; i++)
        {
            bno08x.getSensorEvent(&dummy);
            delay(5);
        }
    }

    SensorBus_Unlock();

    if (!save_ok)
        return false;

    // The sensor may briefly stop responding while committing DCD. Force a fresh
    // initialization and fresh rotation vector before declaring the IMU healthy again.
    delay(2000);
    MarkBNOUnavailable();
    return true;
}

uint8_t BNO085_GetCalibrationStatus()
{
    return g_last_calibration_status;
}

void BNO085_TareYaw()
{
    // Adjust only the relative yaw reference. Absolute heading remains untouched.
    g_yaw_tare_offset += g_cached_data.yaw;
    g_cached_data.yaw = 0.0f;
}

void BNO085_GetCalibrationBiases(float &mag_x, float &mag_y, float &mag_z,
                                 float &gyro_x, float &gyro_y, float &gyro_z)
{
    mag_x = g_mag_bias_x;
    mag_y = g_mag_bias_y;
    mag_z = g_mag_bias_z;
    gyro_x = g_gyro_bias_x;
    gyro_y = g_gyro_bias_y;
    gyro_z = g_gyro_bias_z;
}
