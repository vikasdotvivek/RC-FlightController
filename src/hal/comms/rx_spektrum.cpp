#include "hal/comms/rx_spektrum.h"
#include "config.h"
#include "math/utils.h"

/// TODO: delete after writing rx_read()
float raw_aileron_pwm = 1500;
float raw_elevator_pwm = 1500;
float raw_rudder_pwm = 1500;
float raw_throttle_pwm = 1000;
float raw_flightmode_pwm = 1300;

RCData rc_data; ////global variable to hold the latest RC data read from the receiver

void rx_init() {
    ////kens code
    ///defaults..will be overwritten by rx_read()
    rc_data.aileron_pwm = 1500.0f;
    rc_data.elevator_pwm = 1500.0f;
    rc_data.throttle_pwm = 1000.0f; 
    rc_data.rudder_pwm = 1500.0f;
    rc_data.flightmode_pwm = 1300.0f; //default manual
    rc_data.healthy = false;

}


void rx_read() {
    
    ///kens code(placeholder for reading raw pwm values from the rx)

    //    task should be added to main.cpp as well

    ////return  rc_data.aileron_pwm,  rc_data.elevator_pwm,  rc_data.throttle_pwm,  rc_data.rudder_pwm, rc_data.flightmode_pwm, rc_data.healthy
}

float rx_to_angle(float raw_pwm, float max_angle){
    
    raw_pwm = math::clamp_value(raw_pwm, 1000.0f, 2000.0f);
    return ((raw_pwm-1500.0f)/500.0f) * max_angle;
    
}

float rx_to_throttle(float raw_pwm){
    if (raw_pwm<1050)
    {
        raw_pwm = 1000;
    }
    else if (raw_pwm > 2000)
    {
        raw_pwm = 2000;
    }
    return raw_pwm -1000.0f ;
}

float get_des_roll() {
    float des_roll = rx_to_angle(rc_data.aileron_pwm, max_roll_angle); //////////////////outputs angles
    return des_roll;
}

float get_des_pitch() {
    float des_pitch = rx_to_angle(rc_data.elevator_pwm, max_pitch_angle);
    return des_pitch;
}
float get_des_yaw() {
    float des_yaw = rx_to_angle(rc_data.rudder_pwm, max_yaw_angle);
    return des_yaw;
}

float get_des_throttle(){
    float des_throttle = rx_to_throttle(rc_data.throttle_pwm);
    return des_throttle;
}

float get_flight_mode_pwm() {
    return rc_data.flightmode_pwm;
}

