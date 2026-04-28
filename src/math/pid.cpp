#include "math/pid.h"

PIDController::PIDController(float kp, float ki, float kd, float max_output, float max_integral) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->max_output = max_output;
    this->max_integral = max_integral;

    previous_error = 0.0f;
    integral = 0.0f;
    last_output = 0.0f;
}

float PIDController::compute(float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;

    integral += error * dt; 
    
    if(integral > max_integral) {
        integral = max_integral;
    } else if (integral < -max_integral) {
        integral = -max_integral;
    }

    float p = kp * error;
    float i = ki * integral; 
    float d = kd * (error - previous_error) / dt; 
    previous_error = error;

    float output = p + i + d;
    if (output>max_output)
    {
        output = max_output;
    }
    else if (output<-max_output)
    {
        output = -max_output;
    }
    last_output = output;
    return output;
}

void PIDController::PIDreset() {  //call before every controlled flight
    previous_error = 0.0;
    integral = 0.0;
}

void PIDController::setTuning(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    PIDreset();
    last_output = 0.0f;
}


float PIDController::getLastOutput() const {
    return last_output;
}

//telemetry getters
float PIDController::getkp() {
    return kp;
}

float PIDController::getki() {
    return ki;
}

float PIDController::getkd() {
    return kd;
}

