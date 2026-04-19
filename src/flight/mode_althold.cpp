#include "flight/flightmodes.h"
# include "hal/comms/rx_spektrum.h"
#include "hal/sensors/imu.h"
#include "hal/sensors/baro.h"
#include "hal/sensors/gps.h"
#include "flight/motormixer.h"
#include "math/pid.h"
#include "flight/home.h"
#include "math/utils.h"
#include "config.h"
#include <Arduino.h>


extern IMUData_filtered imu_data;
extern BarometerData baro_data;
extern GPSData gps_data;

extern PIDController roll_pid;
extern PIDController pitch_pid;
extern PIDController altitude_pid;

void mode_alt_hold_init() {
    roll_pid.PIDreset();
    pitch_pid.PIDreset(); 
    altitude_pid.PIDreset();    
}

void mode_alt_hold_run(){
    float actual_roll = imu_data.roll;
    float actual_pitch = imu_data.pitch;
    
    float des_throttle = get_des_throttle();
    float des_roll = get_des_roll();
    float des_pitch = get_des_pitch();

       // only run altitude hold if we have a valid altitude reading and home is set.
    if ((baro_data.healthy || gps_data.lock_acquired) && home_is_set()) {
        
        float target_agl = target_alt_agl;
        float actual_alt_msl = baro_data.healthy ? baro_data.altitude : gps_data.altitude;
        float actual_alt_agl = calc_AGL(actual_alt_msl);

        des_pitch = altitude_pid.compute(target_agl, actual_alt_agl, flight_control_dt_seconds);   
        
        des_pitch = math::clamp_value(des_pitch, -max_pitch_angle, max_pitch_angle);
    }

    float roll = roll_pid.compute(des_roll,  actual_roll, flight_control_dt_seconds);
    float pitch = pitch_pid.compute(des_pitch, actual_pitch, flight_control_dt_seconds);
    float throttle = des_throttle;

    if (ROLL_PID_DEBUG_OUTPUT_ENABLED) {
        Serial.printf("target_roll=%.2f actual_roll=%.2f roll_pid_output=%.2f\n",
                      des_roll,
                      actual_roll,
                      roll);
    }

    motormixer_compute(throttle, roll, pitch, 0.0f);

}