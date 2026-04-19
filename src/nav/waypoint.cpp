#include "nav/waypoint.h"
#include "math/utils.h"
#include <cmath>
#include "config.h"

Navigation navigation;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double NAV_EARTH_RADIUS_METERS = 6371000.0;

double degrees_to_radians(double degrees) {
    return degrees * kPi / 180.0;
}

float radians_to_degrees(double radians) {
    return static_cast<float>(radians * 180.0 / kPi);
}

float normalize_heading_degrees(float heading_degrees) {
    while (heading_degrees < 0.0f) {
        heading_degrees += 360.0f;
    }
    while (heading_degrees >= 360.0f) {
        heading_degrees -= 360.0f;
    }
    return heading_degrees;
}

float calculate_haversine_distance_meters(
    double current_latitude_deg,
    double current_longitude_deg,
    double target_latitude_deg,
    double target_longitude_deg
) {
    const double current_latitude_rad = degrees_to_radians(current_latitude_deg);
    const double target_latitude_rad = degrees_to_radians(target_latitude_deg);
    const double delta_latitude_rad = degrees_to_radians(target_latitude_deg - current_latitude_deg);
    const double delta_longitude_rad = degrees_to_radians(target_longitude_deg - current_longitude_deg);

    const double sin_latitude = std::sin(delta_latitude_rad / 2.0);
    const double sin_longitude = std::sin(delta_longitude_rad / 2.0);

    const double haversine_a =
        (sin_latitude * sin_latitude) +
        (std::cos(current_latitude_rad) * std::cos(target_latitude_rad) * sin_longitude * sin_longitude);
    const double clamped_haversine_a = std::fmin(1.0, std::fmax(0.0, haversine_a));
    const double haversine_c =
        2.0 * std::atan2(std::sqrt(clamped_haversine_a), std::sqrt(1.0 - clamped_haversine_a));

    return static_cast<float>(NAV_EARTH_RADIUS_METERS * haversine_c);
}

float calculate_initial_bearing_degrees(
    double current_latitude_deg,
    double current_longitude_deg,
    double target_latitude_deg,
    double target_longitude_deg
) {
    const double current_latitude_rad = degrees_to_radians(current_latitude_deg);
    const double target_latitude_rad = degrees_to_radians(target_latitude_deg);
    const double delta_longitude_rad = degrees_to_radians(target_longitude_deg - current_longitude_deg);

    const double y = std::sin(delta_longitude_rad) * std::cos(target_latitude_rad);
    const double x =
        (std::cos(current_latitude_rad) * std::sin(target_latitude_rad)) -
        (std::sin(current_latitude_rad) * std::cos(target_latitude_rad) * std::cos(delta_longitude_rad));

    return normalize_heading_degrees(radians_to_degrees(std::atan2(y, x)));
}

} // namespace

Navigation::Navigation() {
    current_waypoint_index = 0;
    mission_complete = false;
    target_heading = 0.0f;
    target_distance = 0.0f;
    leg_reference_distance = 0.0f;
    leg_progress_percent = 0.0f;
}

void Navigation::restart_mission() {
    current_waypoint_index = 0;
    mission_complete = false;
    target_heading = 0.0f;
    target_distance = 0.0f;
    leg_reference_distance = 0.0f;
    leg_progress_percent = 0.0f;
}

void Navigation::update(double current_lat, double current_lon, float current_alt){
    (void)current_alt;

    if (current_waypoint_index >= num_waypoints){
        mission_complete = true;
        target_distance = 0.0f;
        leg_reference_distance = 0.0f;
        leg_progress_percent = 100.0f;
        return;
    }
    
    mission_complete = false;

    while (current_waypoint_index < num_waypoints) {
        const waypoint &target_wp = missionwaypoints[current_waypoint_index];

        target_distance = calculate_haversine_distance_meters(
            current_lat,
            current_lon,
            target_wp.lat,
            target_wp.lon
        );
        target_heading = calculate_initial_bearing_degrees(
            current_lat,
            current_lon,
            target_wp.lat,
            target_wp.lon
        );

        if (leg_reference_distance <= WAYPOINT_ACCEPTANCE_RADIUS_METERS) {
            leg_reference_distance = target_distance;
        }

        const float effective_leg_distance =
            std::fmax(leg_reference_distance, WAYPOINT_ACCEPTANCE_RADIUS_METERS);
        leg_progress_percent =
            100.0f * (1.0f - (target_distance / effective_leg_distance));
        leg_progress_percent = std::fmax(0.0f, std::fmin(100.0f, leg_progress_percent));

        if (target_distance >= WAYPOINT_ACCEPTANCE_RADIUS_METERS) {
            return;
        }

        current_waypoint_index++;
        leg_reference_distance = 0.0f;
        leg_progress_percent = 100.0f;
    }

    if (current_waypoint_index >= num_waypoints) {
        mission_complete = true;
        target_distance = 0.0f;
        leg_reference_distance = 0.0f;
        leg_progress_percent = 100.0f;
    }
}

float Navigation::get_target_heading() {
    return target_heading;
}

float Navigation::get_target_distance() {
    return target_distance;
}

float Navigation::get_target_altitude() {
    if (current_waypoint_index >= num_waypoints && num_waypoints > 0) {
        return missionwaypoints[num_waypoints -1].alt_AGL;
    }

    return missionwaypoints[current_waypoint_index].alt_AGL;
}

float Navigation::get_leg_progress_percent() {
    return leg_progress_percent;
}

float Navigation::get_mission_progress_percent() {
    if (num_waypoints <= 0) {
        return 100.0f;
    }

    float completed_waypoints = static_cast<float>(current_waypoint_index);
    if (!mission_complete && current_waypoint_index < num_waypoints) {
        completed_waypoints += leg_progress_percent / 100.0f;
    }

    const float mission_progress =
        100.0f * (completed_waypoints / static_cast<float>(num_waypoints));
    return math::clamp_value(mission_progress, 0.0f, 100.0f);
}

int Navigation::get_current_waypoint_index() {
    return current_waypoint_index;
}

int Navigation::get_total_waypoint_count() {
    return num_waypoints;
}

bool Navigation::mission_completed() {
    return mission_complete;
}
