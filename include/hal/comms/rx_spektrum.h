#pragma once
#include "datatypes.h"

extern RCData rc_data;

void rx_init();

///rc channel pwm to target roll pitch setpoints
//all these return "DESIRED PLANE ANGLES" in degrees, throttle in 0-100 percent
float get_des_roll();
float get_des_pitch();
float get_des_yaw();
float get_des_throttle();

float get_flight_mode_pwm();

#define SERVO_MIN 500
#define SERVO_MAX 2500
