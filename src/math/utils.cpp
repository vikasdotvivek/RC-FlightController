#include "math/utils.h"
 
namespace math {

    float clamp_value(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
 
    if (value > maximum) {
        return maximum;
    }
 
    return value;
    }

    float wrap_heading_error(float heading_error_degrees) {
    while (heading_error_degrees > 180.0f) {
        heading_error_degrees -= 360.0f;
    }
 
    while (heading_error_degrees < -180.0f) {
        heading_error_degrees += 360.0f;
    }
 
    return heading_error_degrees;
    }
} // namespace math
