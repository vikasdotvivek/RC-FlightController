#pragma once
#include "datatypes.h"

void IMU_Init();
bool IMU_Calibrate_Gyro();
bool IMU_Run_Level_Calibration(float &roll_offset_deg, float &pitch_offset_deg);
bool IMU_Run_Mag_Calibration(float &offset_x, float &offset_y, float &offset_z,
                             float &scale_x, float &scale_y, float &scale_z);
void IMU_Read(IMUData_raw &data);
