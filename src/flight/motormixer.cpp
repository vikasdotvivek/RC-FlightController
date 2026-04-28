#include "flight/motormixer.h"
#include "hal/actuators/pwm_out.h"
#include "config.h"


void motormixer_init(){
    pwm_reset();
}

// throttle_percent: 0-100 (not a PWM offset)
// roll/pitch/yaw:   microseconds of servo PWM offset from neutral
void motormixer_compute(float throttle_percent, float roll_pid, float pitch_pid, float yaw_pid){

    int throttle_out = ThrottleToPWM(throttle_percent);

    int aileron_out  = aileron_int + (int)roll_pid;
    int elevator_out = elevator_int + (int)pitch_pid;
    int rudder_out   = rudder_int + (int)yaw_pid;

    setThrottle(throttle_out);
    setAileron(aileron_out);
    setElevator(elevator_out);
    setRudder(rudder_out);
}