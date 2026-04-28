#include "config.h"
#include "flight/flightmodes.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/baro.h"
#include "hal/comms/rx_spektrum.h"
#include "hal/actuators/pwm_out.h"
#include "flight/motormixer.h"
#include "hal/comms/rx_spektrum.h"

void mode_manual_init() {
    
}

void mode_manual_run(){

    if(!rc_data.healthy){
        motormixer_compute(0.0f, 0.0f, 0.0f, 0.0f); //kill motors if rc data is not healthy
        return;
    }

    float desired_throttle = get_des_throttle();                   // 0-100 percent, honors ESC calibration
    float desired_roll  = (float)rc_data.aileron_pwm  - 1500.0f;   // us of servo offset
    float desired_pitch = (float)rc_data.elevator_pwm - 1500.0f;
    float desired_yaw   = (float)rc_data.rudder_pwm   - 1500.0f;

    motormixer_compute(desired_throttle, desired_roll, desired_pitch, desired_yaw);
}
