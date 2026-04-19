#pragma once
# include "datatypes.h"

//gps coord of start loc
void set_home_location(double latitude, double longitude, float msl_altitude);

//calculate relative altitudes wrt home
float calc_AGL(float current_msl);

waypoint get_home_location();

bool home_is_set();