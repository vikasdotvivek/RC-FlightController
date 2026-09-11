/**
 * BNO085 Sensor Driver Interface
 *
 * Wraps the BNO085 fused attitude solution while exposing both relative/tared yaw
 * and an absolute magnetic heading suitable for navigation.
 */
#pragma once

#include <stdint.h>
#include "datatypes.h"

// Rotation vector is requested at 100 Hz. More than 200 ms without one is stale.
constexpr uint32_t BNO085_DATA_TIMEOUT_MS = 200;

void BNO085_Init();
void BNO085_Read(IMUData_raw &data);

// Returns true when the SH-2 calibration command and supporting report setup succeed.
bool BNO085_EnableBackgroundCalibration();

// Returns true when the DCD save command is accepted by SH-2.
bool BNO085_SaveCalibrationToFlash();

// 0 = unreliable, 1 = low, 2 = medium, 3 = high.
uint8_t BNO085_GetCalibrationStatus();

// Zero only the relative yaw value. IMUData_raw::heading remains absolute for navigation.
void BNO085_TareYaw();

void BNO085_GetCalibrationBiases(float &mag_x, float &mag_y, float &mag_z,
                                 float &gyro_x, float &gyro_y, float &gyro_z);
