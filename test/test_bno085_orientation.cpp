#include <Arduino.h>
#include "config.h"
#include "hal/sensors/bno085.h"
#include "hal/sensors/sensor_bus.h"

namespace
{
    constexpr uint32_t kImuReadPeriodMs = 10;  // 100 Hz
    constexpr uint32_t kPrintPeriodMs = 200;   // 5 Hz

    IMUData_raw g_imu_data = {};
    uint32_t g_last_imu_read_ms = 0;
    uint32_t g_last_print_ms = 0;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    Serial.println("\n============================================");
    Serial.println("   BNO085 RAW + ORIENTATION TEST            ");
    Serial.println("============================================");

    if (!SensorBus_Init())
    {
        Serial.println("Sensor I2C bus init failed.");
    }

    BNO085_Init();

    Serial.println("BNO085 initialized.");
    Serial.println("Streaming data at 5 Hz...");
    Serial.println();
}

void loop()
{
    const uint32_t now_ms = millis();

    if ((now_ms - g_last_imu_read_ms) >= kImuReadPeriodMs)
    {
        g_last_imu_read_ms = now_ms;
        BNO085_Read(g_imu_data);
    }

    if ((now_ms - g_last_print_ms) >= kPrintPeriodMs)
    {
        g_last_print_ms = now_ms;

        if (g_imu_data.healthy)
        {
            Serial.printf("Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | Heading: %6.2f || "
                          "Accel [X: %6.2f Y: %6.2f Z: %6.2f] | "
                          "Gyro [X: %6.2f Y: %6.2f Z: %6.2f] | "
                          "Mag [X: %6.2f Y: %6.2f Z: %6.2f]\n",
                          g_imu_data.roll, g_imu_data.pitch, g_imu_data.yaw, g_imu_data.heading,
                          g_imu_data.accel_x, g_imu_data.accel_y, g_imu_data.accel_z,
                          g_imu_data.gyro_x, g_imu_data.gyro_y, g_imu_data.gyro_z,
                          g_imu_data.mag_x, g_imu_data.mag_y, g_imu_data.mag_z);
        }
        else
        {
            Serial.println("BNO085 data stale/unhealthy or sensor disconnected.");
        }
    }
}
