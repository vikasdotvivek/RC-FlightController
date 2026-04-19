  /////////////////////confirm the logic///////////////////////////////////////////////////////////////////

#include "flight/flightmodes.h"
#include "flight/motormixer.h"
#include "math/pid.h"
#include "hal/sensors/imu.h"
#include "hal/comms/rx_spektrum.h"
#include "config.h"
#include <Arduino.h>

extern IMUData_filtered imu_data;
extern PIDController roll_pid;
extern PIDController pitch_pid;


void mode_stabilize_init() {
    roll_pid.PIDreset();
    pitch_pid.PIDreset();     
}

void mode_stabilize_run() {
    //from imu
    float actual_roll = imu_data.roll;
    float actual_pitch = imu_data.pitch;
    //stick inputs
    float des_roll = get_des_roll();
    float des_pitch = get_des_pitch();
    float des_throttle = get_des_throttle();


  const float target_roll = 0.0f;
  float roll = roll_pid.compute(target_roll, actual_roll, flight_control_dt_seconds);
    float pitch = pitch_pid.compute(0.0f, actual_pitch, flight_control_dt_seconds);
    float throttle = des_throttle;

//   if (ROLL_PID_DEBUG_OUTPUT_ENABLED) {
    // Serial.printf("target_roll=%.2f actual_roll=%.2f roll_pid_output=%.2f\n",
    //         target_roll,
    //         actual_roll,
    //         roll);
//   }

    motormixer_compute(throttle, roll, pitch, 0.0f); ///////////////change throttle tp rc_throttle pwm

  
}
