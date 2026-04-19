#include "flight/home.h"
#include "config.h"

namespace{
HomeState home = {
     0.0,
     0.0,
     0.0f,
     false
};
}

void set_home_location(double latitude, double longitude, float msl_altitude){
    home.lat = latitude;
    home.lon = longitude;
    home.alt_MSL = msl_altitude;
    home.is_set = true;
}

float calc_AGL(float current_msl){
    if (!home.is_set) {
        return -1.0f; /////////error value 
    }
    return current_msl - home.alt_MSL;
}

waypoint get_home_location(){
    return {home.lat, home.lon, 0};//agl
}

bool home_is_set(){
    return home.is_set;
}