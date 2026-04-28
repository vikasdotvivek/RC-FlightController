#include "logging/sd_logger.h"
#include "config.h"
#include "hal/comms/spi_bus.h"

#include <SPI.h>
#include <SD.h>

#include <freertos/FreeRTOS.h>

namespace {

void DeselectLoRaOnSharedSPI() {
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
}

bool g_sd_card_ready = false;

} // namespace

File logFile; // Global file object for the current log

bool SD_Logger_Init() {
    Serial.println("\n--- Adafruit Feather V2 SD Init ---");

    if (!SPIBus_Init()) {
        Serial.println("SPI bus mutex init failed.");
        g_sd_card_ready = false;
        return false;
    }

    if (!SPIBus_Lock(pdMS_TO_TICKS(SPI_BUS_LOCK_TIMEOUT_MS))) {
        Serial.println("Timed out waiting for SPI bus lock.");
        g_sd_card_ready = false;
        return false;
    }

    DeselectLoRaOnSharedSPI();
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);

    // Bring up the shared SPI bus on the board pin map.
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, -1);

    // SD and LoRa share one bus, so keep one shared max frequency.
    if (!SD.begin(SD_CS, SPI, SPI_BUS_FREQUENCY_HZ)) {
        SPIBus_Unlock();
        Serial.println("Card Mount Failed.");
        Serial.println("Troubleshooting: Check wiring, SD format (FAT32), or power supply.");
        g_sd_card_ready = false;
        return false;
    }
    
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        SPIBus_Unlock();
        Serial.println("No SD card attached (Hardware detected, but no card found).");
        g_sd_card_ready = false;
        return false;
    }

    Serial.println("SD Card Successfully Mounted!");
    
    Serial.print("Card Type: ");
    if (cardType == CARD_MMC) Serial.println("MMC");
    else if (cardType == CARD_SD) Serial.println("SDSC");
    else if (cardType == CARD_SDHC) Serial.println("SDHC");
    else Serial.println("UNKNOWN");

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("Card Size: %lluMB\n", cardSize);
    Serial.printf("Shared SPI Frequency: %luHz\n", static_cast<unsigned long>(SPI_BUS_FREQUENCY_HZ));

    SPIBus_Unlock();
    g_sd_card_ready = true;

    return true;
}

bool SD_Logger_CreateNewLog() {
    if (!SPIBus_Lock(pdMS_TO_TICKS(SPI_BUS_LOCK_TIMEOUT_MS))) {
        Serial.println("Timed out waiting for SPI bus lock while creating log file.");
        return false;
    }

    DeselectLoRaOnSharedSPI();

    // Find an unused log file name
    char filename[] = "/log_000.csv";
    for (int i = 0; i < 1000; i++) {
        filename[5] = i / 100 + '0';
        filename[6] = (i / 10) % 10 + '0';
        filename[7] = i % 10 + '0';
        if (!SD.exists(filename)) {
            logFile = SD.open(filename, FILE_WRITE);
            if (logFile) {
                Serial.printf("Created new log file: %s\n", filename);
                SPIBus_Unlock();
                return true;
            } else {
                Serial.printf("Failed to create log file: %s\n", filename);
                SPIBus_Unlock();
                return false;
            }
        }
    }
    SPIBus_Unlock();
    Serial.println("Could not create a new log file (all 000-999 exist).");
    return false;
}

void SD_Logger_WriteHeader() {
    if (!logFile || !SPIBus_Lock(pdMS_TO_TICKS(SPI_BUS_LOCK_TIMEOUT_MS))) {
        return;
    }

    DeselectLoRaOnSharedSPI();
    logFile.println(
        "roll,pitch,yaw,des_roll,des_pitch,des_yaw,des_throttle,altitude,des_altitude,"
        "gps_lat,gps_long,gps_alt,gps_speed,gps_heading,"
        "gps_sats,gps_fix_quality,gps_lock_acquired,baro_altitude,flightmode,"
        "waypoint_distance,waypoint_heading,waypoint_target_alt,waypoint_leg_progress,"
        "waypoint_mission_progress,waypoint_index,waypoint_total,waypoint_mission_complete,"
        "waypoint_target_lat,waypoint_target_lon,airspeed,"
        "imu_healthy,baro_healthy,gps_healthy,rx_healthy,armed,"
        "roll_pid_out,pitch_pid_out,yaw_pid_out,"
        "roll_pid_kp,roll_pid_ki,roll_pid_kd,"
        "pitch_pid_kp,pitch_pid_ki,pitch_pid_kd,"
        "yaw_pid_kp,yaw_pid_ki,yaw_pid_kd,"
        "altitude_pid_kp,altitude_pid_ki,altitude_pid_kd,"
        "headingerror_pid_kp,headingerror_pid_ki,headingerror_pid_kd,"
        "rx_throttle_pwm,rx_aileron_pwm,rx_elevator_pwm,rx_rudder_pwm,rx_mode_pwm"
    );
    logFile.flush(); // Ensure header is written immediately
    SPIBus_Unlock();
}

void SD_Logger_LogData(const telemetrydata& data) {
    if (!logFile || !SPIBus_Lock(pdMS_TO_TICKS(SPI_BUS_LOCK_TIMEOUT_MS))) {
        return;
    }

    DeselectLoRaOnSharedSPI();

    logFile.print(data.roll, 2); logFile.print(",");
    logFile.print(data.pitch, 2); logFile.print(",");
    logFile.print(data.yaw, 2); logFile.print(",");
    logFile.print(data.des_roll, 2); logFile.print(",");
    logFile.print(data.des_pitch, 2); logFile.print(",");
    logFile.print(data.des_yaw, 2); logFile.print(",");
    logFile.print(data.des_throttle, 2); logFile.print(",");
    logFile.print(data.altitude, 2); logFile.print(",");
    logFile.print(data.des_altitude, 2); logFile.print(",");
    logFile.print(data.gps_lat, 6); logFile.print(",");
    logFile.print(data.gps_long, 6); logFile.print(",");
    logFile.print(data.gps_alt, 2); logFile.print(",");
    logFile.print(data.gps_speed, 2); logFile.print(",");
    logFile.print(data.gps_heading, 2); logFile.print(",");
    logFile.print(data.gps_sats); logFile.print(",");
    logFile.print(data.gps_fix_quality); logFile.print(",");
    logFile.print(data.gps_lock_acquired); logFile.print(",");
    logFile.print(data.baro_altitude, 2); logFile.print(",");
    logFile.print(data.flightmode); logFile.print(",");
    logFile.print(data.waypoint_distance, 2); logFile.print(",");
    logFile.print(data.waypoint_heading, 2); logFile.print(",");
    logFile.print(data.waypoint_target_alt, 2); logFile.print(",");
    logFile.print(data.waypoint_leg_progress, 2); logFile.print(",");
    logFile.print(data.waypoint_mission_progress, 2); logFile.print(",");
    logFile.print(data.waypoint_index); logFile.print(",");
    logFile.print(data.waypoint_total); logFile.print(",");
    logFile.print(data.waypoint_mission_complete); logFile.print(",");
    logFile.print(data.waypoint_target_lat, 6); logFile.print(",");
    logFile.print(data.waypoint_target_lon, 6); logFile.print(",");
    logFile.print(data.airspeed, 2); logFile.print(",");
    logFile.print(data.imu_healthy); logFile.print(",");
    logFile.print(data.baro_healthy); logFile.print(",");
    logFile.print(data.gps_healthy); logFile.print(",");
    logFile.print(data.rx_healthy); logFile.print(",");
    logFile.print(data.armed); logFile.print(",");
    logFile.print(data.roll_pid_out, 2); logFile.print(",");
    logFile.print(data.pitch_pid_out, 2); logFile.print(",");
    logFile.print(data.yaw_pid_out, 2); logFile.print(",");
    logFile.print(data.roll_pid_kp, 3); logFile.print(",");
    logFile.print(data.roll_pid_ki, 3); logFile.print(",");
    logFile.print(data.roll_pid_kd, 3); logFile.print(",");
    logFile.print(data.pitch_pid_kp, 3); logFile.print(",");
    logFile.print(data.pitch_pid_ki, 3); logFile.print(",");
    logFile.print(data.pitch_pid_kd, 3); logFile.print(",");
    logFile.print(data.yaw_pid_kp, 3); logFile.print(",");
    logFile.print(data.yaw_pid_ki, 3); logFile.print(",");
    logFile.print(data.yaw_pid_kd, 3); logFile.print(",");
    logFile.print(data.altitude_pid_kp, 3); logFile.print(",");
    logFile.print(data.altitude_pid_ki, 3); logFile.print(",");
    logFile.print(data.altitude_pid_kd, 3); logFile.print(",");
    logFile.print(data.headingerror_pid_kp, 3); logFile.print(",");
    logFile.print(data.headingerror_pid_ki, 3); logFile.print(",");
    logFile.print(data.headingerror_pid_kd, 3); logFile.print(",");
    logFile.print(data.rx_throttle_pwm, 0); logFile.print(",");
    logFile.print(data.rx_aileron_pwm, 0); logFile.print(",");
    logFile.print(data.rx_elevator_pwm, 0); logFile.print(",");
    logFile.print(data.rx_rudder_pwm, 0); logFile.print(",");
    logFile.print(data.rx_mode_pwm, 0);
    logFile.println();
    SPIBus_Unlock();
}

void SD_Logger_Flush() {
    if (logFile && SPIBus_Lock(pdMS_TO_TICKS(SPI_BUS_LOCK_TIMEOUT_MS))) {
        DeselectLoRaOnSharedSPI();
        logFile.flush();
        SPIBus_Unlock();
    }
}

void SD_Logger_CloseLog() {
    if (logFile && SPIBus_Lock(pdMS_TO_TICKS(SPI_BUS_LOCK_TIMEOUT_MS))) {
        DeselectLoRaOnSharedSPI();
        logFile.close();
        SPIBus_Unlock();
        Serial.println("Log file closed.");
    }
}

bool SD_Logger_IsReady() {
    return g_sd_card_ready && static_cast<bool>(logFile);
}

bool SD_Logger_SavePIDConfig(float r_kp, float r_ki, float r_kd,
                             float p_kp, float p_ki, float p_kd,
                             float y_kp, float y_ki, float y_kd) {
    if (!g_sd_card_ready || !SPIBus_Lock(pdMS_TO_TICKS(SPI_BUS_LOCK_TIMEOUT_MS))) {
        return false;
    }
    DeselectLoRaOnSharedSPI();

    if (SD.exists("/pid_config.json")) {
        SD.remove("/pid_config.json");
    }

    File file = SD.open("/pid_config.json", FILE_WRITE);
    if (!file) {
        SPIBus_Unlock();
        return false;
    }

    file.printf("{\"roll\":[%f,%f,%f],\"pitch\":[%f,%f,%f],\"yaw\":[%f,%f,%f]}\n",
                r_kp, r_ki, r_kd, p_kp, p_ki, p_kd, y_kp, y_ki, y_kd);
    file.close();
    SPIBus_Unlock();
    return true;
}

bool SD_Logger_LoadPIDConfig(float &r_kp, float &r_ki, float &r_kd,
                             float &p_kp, float &p_ki, float &p_kd,
                             float &y_kp, float &y_ki, float &y_kd) {
    if (!g_sd_card_ready || !SPIBus_Lock(pdMS_TO_TICKS(SPI_BUS_LOCK_TIMEOUT_MS))) {
        return false;
    }
    DeselectLoRaOnSharedSPI();

    if (!SD.exists("/pid_config.json")) {
        SPIBus_Unlock();
        return false;
    }

    File file = SD.open("/pid_config.json", FILE_READ);
    if (!file) {
        SPIBus_Unlock();
        return false;
    }

    String content = file.readStringUntil('\n');
    file.close();
    SPIBus_Unlock();

    int parsed = sscanf(content.c_str(), "{\"roll\":[%f,%f,%f],\"pitch\":[%f,%f,%f],\"yaw\":[%f,%f,%f]}",
                        &r_kp, &r_ki, &r_kd, &p_kp, &p_ki, &p_kd, &y_kp, &y_ki, &y_kd);
    
    return (parsed == 9);
}
