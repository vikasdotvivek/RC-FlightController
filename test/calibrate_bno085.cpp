/**
 * BNO085 Interactive Calibration Utility
 */
#include <Arduino.h>
#include "config.h"
#include "hal/sensors/bno085.h"
#include "hal/sensors/sensor_bus.h"

namespace
{
    constexpr uint32_t kImuReadPeriodMs = 10;
    constexpr uint32_t kPrintPeriodMs = 250;
    constexpr uint8_t kRequiredCalibrationAccuracy = 3;

    IMUData_raw g_imu_data = {};
    uint32_t g_last_imu_read_ms = 0;
    uint32_t g_last_print_ms = 0;

    enum CalibState
    {
        STATE_WAIT_START,
        STATE_MAG_PREP,
        STATE_MAG_RECORD,
        STATE_LEVEL_PREP,
        STATE_LEVEL_RECORD,
        STATE_YAW_PREVIEW,
        STATE_FINISHED
    };

    CalibState g_state = STATE_WAIT_START;
    uint32_t g_state_start_ms = 0;
    int g_last_countdown_sec = -1;
    double g_sum_roll = 0.0;
    double g_sum_pitch = 0.0;
    int g_sample_count = 0;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}

    Serial.println("\n============================================");
    Serial.println("   BNO085 CALIBRATION & EEPROM UTILITY      ");
    Serial.println("============================================");

    if (!SensorBus_Init())
    {
        Serial.println("Sensor I2C bus init failed.");
    }

    BNO085_Init();

    if (!BNO085_EnableBackgroundCalibration())
    {
        Serial.println("ERROR: Failed to enable BNO085 background calibration/reports.");
        Serial.println("Calibration cannot safely continue.");
        while (true) delay(1000);
    }

    Serial.println("BNO085 initialized and background calibration enabled.");
    Serial.println("\nReading existing calibration biases from flash...");

    const uint32_t wait_start = millis();
    while (millis() - wait_start < 1500)
    {
        BNO085_Read(g_imu_data);
        delay(10);
    }

    float mx, my, mz, gx, gy, gz;
    BNO085_GetCalibrationBiases(mx, my, mz, gx, gy, gz);
    Serial.println("\n--- EXISTING SENSOR STATE ---");
    Serial.printf(" Flash Mag Bias  [uT]    : X: %6.2f | Y: %6.2f | Z: %6.2f\n", mx, my, mz);
    Serial.printf(" Flash Gyro Bias [rad/s] : X: %6.4f | Y: %6.4f | Z: %6.4f\n", gx, gy, gz);
    Serial.printf(" Config.h Roll Offset    : %6.2f deg\n", IMU_LEVEL_ROLL_OFFSET_DEG);
    Serial.printf(" Config.h Pitch Offset   : %6.2f deg\n", IMU_LEVEL_PITCH_OFFSET_DEG);
    Serial.println("-----------------------------\n");

    Serial.println("Press ENTER in the serial monitor to begin the calibration sequence...");
}

void loop()
{
    const uint32_t now_ms = millis();

    if ((now_ms - g_last_imu_read_ms) >= kImuReadPeriodMs)
    {
        g_last_imu_read_ms = now_ms;
        BNO085_Read(g_imu_data);

        if (g_state == STATE_LEVEL_RECORD && g_imu_data.healthy)
        {
            g_sum_roll += (g_imu_data.roll + IMU_LEVEL_ROLL_OFFSET_DEG);
            g_sum_pitch += (g_imu_data.pitch + IMU_LEVEL_PITCH_OFFSET_DEG);
            ++g_sample_count;
        }
    }

    switch (g_state)
    {
    case STATE_WAIT_START:
        if (Serial.available())
        {
            while (Serial.available()) Serial.read();
            g_state = STATE_MAG_PREP;
            g_state_start_ms = now_ms;
            g_last_countdown_sec = -1;
            Serial.println("\n>>> PHASE 1: MAGNETOMETER CALIBRATION <<<");
            Serial.println("Pick up the aircraft. Get ready to tumble it in figure-8 patterns.");
        }
        break;

    case STATE_MAG_PREP:
    {
        const int remain = 5 - ((now_ms - g_state_start_ms) / 1000);
        if (remain != g_last_countdown_sec)
        {
            g_last_countdown_sec = remain;
            if (remain > 0) Serial.printf("Starting in %d...\n", remain);
        }
        if (remain <= 0)
        {
            g_state = STATE_MAG_RECORD;
            g_state_start_ms = now_ms;
            g_last_countdown_sec = -1;
            Serial.println("\n[RECORDING] Tumble the aircraft NOW! Keep moving for 15 seconds...");
        }
        break;
    }

    case STATE_MAG_RECORD:
    {
        const int remain = 15 - ((now_ms - g_state_start_ms) / 1000);
        if (remain != g_last_countdown_sec)
        {
            g_last_countdown_sec = remain;
            const uint8_t accuracy = BNO085_GetCalibrationStatus();
            if (remain > 0)
                Serial.printf("  Time left: %2ds | Current Accuracy: %d/3\n", remain, accuracy);
        }

        if (remain <= 0)
        {
            const uint8_t accuracy = BNO085_GetCalibrationStatus();
            if (accuracy < kRequiredCalibrationAccuracy)
            {
                Serial.printf("\nCalibration accuracy is only %d/3. NOT saving to flash.\n", accuracy);
                Serial.println("Continue tumbling; another 15-second calibration window is starting.");
                g_state_start_ms = now_ms;
                g_last_countdown_sec = -1;
                break;
            }

            Serial.println("\nAccuracy is 3/3. Saving dynamic calibration to BNO085 flash...");
            if (!BNO085_SaveCalibrationToFlash())
            {
                Serial.println("ERROR: BNO085 rejected or failed the DCD save command.");
                Serial.println("Calibration was NOT confirmed saved. Continue tumbling and retrying.");
                g_state_start_ms = millis();
                g_last_countdown_sec = -1;
                break;
            }

            Serial.println("*** FLASH SAVE COMMAND ACCEPTED ***\n");
            // Save forces the driver to require a fresh reconnect. Read until fresh again.
            const uint32_t reconnect_start = millis();
            while (!g_imu_data.healthy && (millis() - reconnect_start) < 5000)
            {
                BNO085_Read(g_imu_data);
                delay(20);
            }

            g_state = STATE_LEVEL_PREP;
            g_state_start_ms = millis();
            g_last_countdown_sec = -1;
            Serial.println("\n>>> PHASE 2: LEVEL CALIBRATION <<<");
            Serial.println("Place the aircraft PERFECTLY FLAT AND STILL on the desk.");
        }
        break;
    }

    case STATE_LEVEL_PREP:
    {
        const int remain = 5 - ((now_ms - g_state_start_ms) / 1000);
        if (remain != g_last_countdown_sec)
        {
            g_last_countdown_sec = remain;
            if (remain > 0) Serial.printf("Starting in %d...\n", remain);
        }
        if (remain <= 0)
        {
            g_sum_roll = 0.0;
            g_sum_pitch = 0.0;
            g_sample_count = 0;
            g_state = STATE_LEVEL_RECORD;
            g_state_start_ms = now_ms;
            g_last_countdown_sec = -1;
            Serial.println("\n[RECORDING] Keep aircraft STILL for 5 seconds...");
        }
        break;
    }

    case STATE_LEVEL_RECORD:
    {
        const int remain = 5 - ((now_ms - g_state_start_ms) / 1000);
        if (remain != g_last_countdown_sec)
        {
            g_last_countdown_sec = remain;
            if (remain > 0) Serial.printf("  Recording... %ds left\n", remain);
        }
        if (remain <= 0)
        {
            if (g_sample_count > 0)
            {
                const float new_roll_offset = g_sum_roll / g_sample_count;
                const float new_pitch_offset = g_sum_pitch / g_sample_count;

                Serial.println("\n============================================");
                Serial.println("         CALIBRATION COMPLETE               ");
                Serial.println("============================================");
                Serial.println("BNO085 Mag/Gyro calibration save was accepted.");
                Serial.println("\nFor Roll/Pitch leveling, copy these lines into include/config.h:");
                Serial.printf("constexpr float IMU_LEVEL_ROLL_OFFSET_DEG = %6.2ff;\n", new_roll_offset);
                Serial.printf("constexpr float IMU_LEVEL_PITCH_OFFSET_DEG = %6.2ff;\n", new_pitch_offset);
                Serial.println("============================================");
                Serial.println("\nStreaming data for 5 seconds before taring relative Yaw to 0...");
            }
            else
            {
                Serial.println("\nError: No fresh IMU samples collected during level calibration.");
            }

            g_state = STATE_YAW_PREVIEW;
            g_state_start_ms = now_ms;
            g_last_countdown_sec = -1;
        }
        break;
    }

    case STATE_YAW_PREVIEW:
    {
        const int remain = 5 - ((now_ms - g_state_start_ms) / 1000);

        if ((now_ms - g_last_print_ms) >= kPrintPeriodMs)
        {
            g_last_print_ms = now_ms;
            if (g_imu_data.healthy)
            {
                const uint8_t accuracy = BNO085_GetCalibrationStatus();
                Serial.printf("Acc: %d/3 | Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | Heading: %6.2f\n",
                              accuracy, g_imu_data.roll, g_imu_data.pitch,
                              g_imu_data.yaw, g_imu_data.heading);
            }
        }

        if (remain <= 0)
        {
            BNO085_TareYaw();
            Serial.println("\n*** RELATIVE YAW SNAPPED TO 0 DEGREES ***");
            Serial.println("Absolute heading remains unchanged for navigation.");
            Serial.println("Press 't' to tare relative Yaw again.");
            g_state = STATE_FINISHED;
        }
        break;
    }

    case STATE_FINISHED:
        if (Serial.available())
        {
            const char c = Serial.read();
            if (c == 't' || c == 'T')
            {
                BNO085_TareYaw();
                Serial.println("\n*** RELATIVE YAW TARED TO 0 DEGREES ***\n");
            }
        }

        if ((now_ms - g_last_print_ms) >= kPrintPeriodMs)
        {
            g_last_print_ms = now_ms;
            if (g_imu_data.healthy)
            {
                const uint8_t accuracy = BNO085_GetCalibrationStatus();
                Serial.printf("Acc: %d/3 | Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f | Heading: %6.2f\n",
                              accuracy, g_imu_data.roll, g_imu_data.pitch,
                              g_imu_data.yaw, g_imu_data.heading);
            }
        }
        break;
    }
}
