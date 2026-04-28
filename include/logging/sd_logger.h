#pragma once

#include <Arduino.h>
#include "datatypes.h"

// Initializes the SD card using the custom SPI bus
bool SD_Logger_Init();

// Creates a new, unique log file on the SD card.
bool SD_Logger_CreateNewLog();

// Writes the CSV header to the currently open log file.
void SD_Logger_WriteHeader();

// Logs a telemetry data snapshot as a CSV row.
void SD_Logger_LogData(const telemetrydata& data);

// Flushes the write buffer to the SD card.
void SD_Logger_Flush();

// Closes the currently open log file.
void SD_Logger_CloseLog();

// Returns true when the SD card is mounted and a log file is open.
bool SD_Logger_IsReady();

// Saves the current PID config to a JSON file on the SD card
bool SD_Logger_SavePIDConfig(float r_kp, float r_ki, float r_kd,
                             float p_kp, float p_ki, float p_kd,
                             float y_kp, float y_ki, float y_kd);

// Loads the PID config from a JSON file on the SD card
bool SD_Logger_LoadPIDConfig(float &r_kp, float &r_ki, float &r_kd,
                             float &p_kp, float &p_ki, float &p_kd,
                             float &y_kp, float &y_ki, float &y_kd);
