#pragma once
#include "Arduino.h"

void pwm_init();
void pwm_reset();

//write pwm values to esc and servos
void setThrottle(int pulse);
void setAileron(int pulse);
void setElevator(int pulse);
void setRudder(int pulse);

//using these to convert physical actuator limits to pwm values for safety and hardware interfacing
int ThrottleToPWM(float throttle_percent);
int AileronToPWM(float aileron_angle);
int ElevatorToPWM(float elevator_angle);
int RudderToPWM(float rudder_angle);

// Debugging
char* debugPWM();