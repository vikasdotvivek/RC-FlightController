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

} // namespace

File logFile; // Global file object for the current log

bool SD_Logger_Init() {
    Serial.println("\n--- Adafruit Feather V2 SD Init ---");

    if (!SPIBus_Init()) {
        Serial.println("SPI bus mutex init failed.");
        return false;
    }

    if (!SPIBus_Lock(pdMS_TO_TICKS(SPI_BUS_LOCK_TIMEOUT_MS))) {
        Serial.println("Timed out waiting for SPI bus lock.");
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
        return false;
    }
    
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        SPIBus_Unlock();
        Serial.println("No SD card attached (Hardware detected, but no card found).");
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
    logFile.println("roll,pitch,yaw,altitude,des_altitude,gps_lat,gps_long,gps_alt,gps_speed,gps_heading,gps_sats,gps_fix_quality,gps_lock_acquired,baro_altitude,flightmode,waypoint_distance,waypoint_heading,waypoint_target_alt,waypoint_leg_progress,waypoint_mission_progress,waypoint_index,waypoint_total,waypoint_mission_complete,waypoint_target_lat,waypoint_target_lon");
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
    logFile.print(data.waypoint_target_lon, 6);
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
