#include "config.h"
#include "datatypes.h"
#include "flight/flightmodes.h"
#include "flight/motormixer.h"
#include "nav/waypoint.h"
#include "math/pid.h"
#include "hal/comms/rx_spektrum.h"
#include "flight/home.h"
#include "math/utils.h"
#include <Arduino.h>

extern PIDController roll_pid;
extern PIDController pitch_pid;
extern PIDController altitude_pid;
extern PIDController headingerror_pid;

extern IMUData_filtered imu_data;
extern IMUData_raw currentIMU;
extern BarometerData baro_data;
extern GPSData gps_data;

void mode_waypoint_init() {
    roll_pid.PIDreset();
    pitch_pid.PIDreset();
    altitude_pid.PIDreset();
    headingerror_pid.PIDreset();
}

void mode_waypoint_run(){
    const float throttle_output = get_des_throttle();

    if (navigation.mission_completed() || !gps_data.lock_acquired || !home_is_set()) {
        const float level_roll_output =
            roll_pid.compute(0.0f, imu_data.roll, flight_control_dt_seconds);
        const float level_pitch_output =
            pitch_pid.compute(2.0f, imu_data.pitch, flight_control_dt_seconds);

        motormixer_compute(throttle_output, level_roll_output, level_pitch_output, 0.0f);
        return;
    }

    const float target_heading = navigation.get_target_heading();
    const float target_altitude = navigation.get_target_altitude();
    const float target_altitude_agl = target_altitude;

    // IMU heading is absolute magnetic heading from the BNO085 and is deliberately
    // separate from software-tared relative yaw. GPS heading is course over ground.
    const float actual_heading = WAYPOINT_USE_IMU_YAW ? currentIMU.heading : gps_data.heading;
    const float actual_altitude_msl = baro_data.healthy ? baro_data.altitude : gps_data.altitude;
    const float actual_altitude_agl = calc_AGL(actual_altitude_msl);

    const bool heading_valid = WAYPOINT_USE_IMU_YAW
        ? currentIMU.healthy
        : (gps_data.lock_acquired && gps_data.speed >= WAYPOINT_MIN_GROUND_SPEED_MPS);

    float desired_roll = 0.0f;
    if (heading_valid) {
        const float heading_error = math::wrap_heading_error(target_heading - actual_heading);
        const float desired_heading_error = 0.0f;
        desired_roll = headingerror_pid.compute(heading_error, desired_heading_error, flight_control_dt_seconds);
        desired_roll = -desired_roll;
    }
    desired_roll = math::clamp_value(desired_roll, -max_roll_angle, max_roll_angle);

    float desired_pitch =
        altitude_pid.compute(target_altitude_agl, actual_altitude_agl, flight_control_dt_seconds);
    desired_pitch = math::clamp_value(desired_pitch, -max_pitch_angle, max_pitch_angle);

    const float roll_output =
        roll_pid.compute(desired_roll, imu_data.roll, flight_control_dt_seconds);
    const float pitch_output =
        pitch_pid.compute(desired_pitch, imu_data.pitch, flight_control_dt_seconds);

    if (ROLL_PID_DEBUG_OUTPUT_ENABLED) {
        Serial.printf("target_roll=%.2f actual_roll=%.2f roll_pid_output=%.2f\n",
                      desired_roll,
                      imu_data.roll,
                      roll_output);
    }

    motormixer_compute(throttle_output, roll_output, pitch_output, 0.0f);
}
