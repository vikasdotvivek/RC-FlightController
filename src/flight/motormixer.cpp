#include "flight/motormixer.h"
#include "hal/actuators/pwm_out.h"
#include "config.h"


void motormixer_init(){
    pwm_reset();
}

//pid controllers output the pwm offesets
// This function takes the outputs from the PID controllers for throttle, roll, pitch, and yaw..offests from neutral and writes the resultimg PWM signals to the ESC and servos.
void motormixer_compute(float throttle_pid, float roll_pid, float pitch_pid, float yaw_pid){
    
    //calc motor pwm outputs
    int throttle_out = throttle_int + (int)throttle_pid;

    int aileron_out  = aileron_int + (int)roll_pid;
    int elevator_out = elevator_int + (int)pitch_pid;
    int rudder_out   = rudder_int + (int)yaw_pid;

    //write to esc and servos
    setThrottle(throttle_out);
    setAileron(aileron_out);
    setElevator(elevator_out);
    setRudder(rudder_out);
    
}