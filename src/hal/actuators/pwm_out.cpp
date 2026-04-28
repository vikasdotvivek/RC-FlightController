#include "hal/actuators/pwm_out.h"
#include "config.h"
#include <algorithm>
#include "math/utils.h"

#define US_TO_DUTY(us) ((uint32_t)(((us) * 65535ULL) / 20000)) // Convert microseconds to duty cycle for 20ms period

// Global Variables to hold the dynamic hardware limits
int AILERON_PWM_MIN, AILERON_PWM_MAX;
int ELEVATOR_PWM_MIN, ELEVATOR_PWM_MAX;
int RUDDER_PWM_MIN, RUDDER_PWM_MAX;

//Converters --converts throttle perccent and control surface deflections into pwm
int ThrottleToPWM(float throttle_percent) {
    return (int)((throttle_percent * throttle_slope) + throttle_int);
}
int AileronToPWM(float aileron_angle) {
    return (int)((aileron_angle * aileron_slope) + aileron_int);
}
int ElevatorToPWM(float elevator_angle) {
    return (int)((elevator_angle * elevator_slope) + elevator_int);
}
int RudderToPWM(float rudder_angle) {
    return (int)((rudder_angle * rudder_slope) + rudder_int);
}

void setThrottle(int pulse) {
    if(pulse < 0) pulse = throttle_safe; //to cutoff esc
    else pulse = math::clamp_value(pulse, throttle_int, throttle_int + (int)(100.0f * throttle_slope));
    ledcWrite(esc_channel, US_TO_DUTY(pulse));
}

void setAileron(int pulse) {
    pulse = math::clamp_value(pulse, AILERON_PWM_MIN, AILERON_PWM_MAX);
    ledcWrite(aileron_channel, US_TO_DUTY(pulse));
}

void setElevator(int pulse) {
    pulse = math::clamp_value(pulse, ELEVATOR_PWM_MIN, ELEVATOR_PWM_MAX);
    ledcWrite(elevator_channel, US_TO_DUTY(pulse));
}

void setRudder(int pulse) {
    pulse = math::clamp_value(pulse, RUDDER_PWM_MIN, RUDDER_PWM_MAX);
    ledcWrite(rudder_channel, US_TO_DUTY(pulse));
}

void pwm_init() {
    // Set up PWM channels with a frequency of 50Hz and 16-bit resolution
    ledcSetup(esc_channel, pwm_freq, pwm_resolution);
    ledcSetup(aileron_channel, pwm_freq, pwm_resolution);
    ledcSetup(elevator_channel, pwm_freq, pwm_resolution);
    ledcSetup(rudder_channel, pwm_freq, pwm_resolution);

    //////attach the channels to the corresponding GPIO pins
    ledcAttachPin(esc_pin, esc_channel);
    ledcAttachPin(aileron_pin, aileron_channel);
    ledcAttachPin(elevator_pin, elevator_channel);
    ledcAttachPin(rudder_pin, rudder_channel);


    //calc endpoints for control surfaces based on config deflection limits
    const int ail_1 = AileronToPWM(aileron_min_deflection_deg);
    const int ail_2 = AileronToPWM(aileron_max_deflection_deg);
    AILERON_PWM_MIN = std::min(ail_1, ail_2);
    AILERON_PWM_MAX = std::max(ail_1, ail_2);

    const int ele_1 = ElevatorToPWM(elevator_min_deflection_deg);
    const int ele_2 = ElevatorToPWM(elevator_max_deflection_deg);
    ELEVATOR_PWM_MIN = std::min(ele_1, ele_2);
    ELEVATOR_PWM_MAX = std::max(ele_1, ele_2);

    const int rud_1 = RudderToPWM(rudder_min_deflection_deg);
    const int rud_2 = RudderToPWM(rudder_max_deflection_deg);
    RUDDER_PWM_MIN = std::min(rud_1, rud_2);
    RUDDER_PWM_MAX = std::max(rud_1, rud_2);


    pwm_reset();
}


void pwm_reset() {
    setThrottle(-1); //idle throttle
    setAileron(aileron_int); 
    setElevator(elevator_int);
    setRudder(rudder_int);
}
