#pragma once
#include <stdint.h>

struct IMUData_raw {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float roll;
    float pitch;
    float yaw;
    bool healthy; // True if sensor is connected and reading
};

struct IMUData_filtered{
    float roll;
    float pitch;
    float yaw;
};

struct GPSLocalTime {
    int hour;
    int minute;
    int second;
    bool valid;
};

struct GPSRawCoordinates {
    char latitude[16];
    char latitude_dir;
    char longitude[16];
    char longitude_dir;
};

struct GPSData {
    double latitude;
    double longitude;
    float altitude;
    float speed;
    float heading;
    int satellites;
    int fix_quality;
    GPSLocalTime local_time;
    GPSRawCoordinates raw_coordinates;
    bool lock_acquired;
    bool healthy; 
};

struct BarometerData {
    float pressure;
    float altitude;
    bool healthy; 
};

struct AirspeedData {
    double pressure_pa;
    float airspeed_mps;
    bool healthy;
};


struct RCData {
    unsigned int aileron_pwm;
    unsigned int elevator_pwm;
    unsigned int throttle_pwm;
    unsigned int rudder_pwm;
    unsigned int flightmode_pwm;
    bool healthy; 
};


struct telemetrydata{
    float roll;
    float pitch;
    float throttle;
    float yaw;
    float des_roll;
    float des_pitch;    
    float des_throttle;
    float des_yaw;
    float altitude;
    float des_altitude;
    float airspeed;
    float gps_lat;
    float gps_long;
    float gps_alt;
    float gps_speed;
    float gps_heading;
    int gps_sats;
    int gps_fix_quality;
    int gps_lock_acquired;
    float baro_altitude;
    float flightmode;
    float failsafe_status;
    float waypoint_distance;
    float waypoint_heading;
    float waypoint_target_lat;
    float waypoint_target_lon;
    float waypoint_target_alt;
    float waypoint_leg_progress;
    float waypoint_mission_progress;
    int waypoint_index;
    int waypoint_total;
    int waypoint_mission_complete;
    // sensor health + arming
    uint8_t imu_healthy;
    uint8_t baro_healthy;
    uint8_t gps_healthy;
    uint8_t rx_healthy;
    uint8_t armed;
    // PID outputs (microseconds of servo offset from neutral)
    float roll_pid_out;
    float pitch_pid_out;
    float yaw_pid_out;
    // Active PID tuning gains
    float roll_pid_kp;
    float roll_pid_ki;
    float roll_pid_kd;
    float pitch_pid_kp;
    float pitch_pid_ki;
    float pitch_pid_kd;
    float yaw_pid_kp;
    float yaw_pid_ki;
    float yaw_pid_kd;
    float altitude_pid_kp;
    float altitude_pid_ki;
    float altitude_pid_kd;
    float headingerror_pid_kp;
    float headingerror_pid_ki;
    float headingerror_pid_kd;
    // raw RX PWM inputs
    float rx_throttle_pwm;
    float rx_aileron_pwm;
    float rx_elevator_pwm;
    float rx_rudder_pwm;
    float rx_mode_pwm;
};


struct waypoint{
    double lat;
    double lon;
    float alt_AGL;          ///////////////agl in m
};

struct HomeState{
    double lat;
    double lon;
    float alt_MSL;          //msl in m
    bool is_set;
};
