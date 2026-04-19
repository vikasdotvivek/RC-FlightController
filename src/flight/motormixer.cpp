#include "flight/motormixer.h"
#include "hal/actuators/pwm_out.h"
#include "config.h"


void motormixer_init(){
    pwm_write(esc_channel, 1000); 
    pwm_write(aileron_channel, 1500);
    pwm_write(elevator_channel, 1500);  
    pwm_write(rudder_channel, 1500);
}

void motormixer_compute(float throttle_pid, float roll_pid, float pitch_pid, float yaw_pid){
    
    //calc motor pwm outputs
    int throttle_out = 1000 + throttle_pid;
    int aileron_out = 1500 + roll_pid;
    int elevator_out = 1500 + pitch_pid;
    int rudder_out = 1500 + yaw_pid;

    // Serial.printf("throttle_out=%d aileron_out=%d elevator_out=%d rudder_out=%d\n",
    //               throttle_out,
    //               aileron_out,
    //               elevator_out,
    //               rudder_out);

    //clamp pwm outputs
    throttle_out = std::max(1000, std::min(2000, throttle_out));
    aileron_out = std::max(1000, std::min(2000, aileron_out));
    elevator_out = std::max(1000, std::min(2000, elevator_out));
    rudder_out = std::max(1000, std::min(2000, rudder_out));

    // Serial.printf("throttle_out=%d aileron_out=%d elevator_out=%d rudder_out=%d\n",
    //               throttle_out,
    //               aileron_out,
    //               elevator_out,
    //               rudder_out);

    // write pwm to motors
    pwm_write(esc_channel, throttle_out);
    pwm_write(aileron_channel, aileron_out);
    pwm_write(elevator_channel, elevator_out);
    pwm_write(rudder_channel, rudder_out);
    
}