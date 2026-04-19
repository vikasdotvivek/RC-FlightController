#pragma once

namespace math
{
    float clamp_value(float value, float minimum, float maximum);

    float wrap_heading_error(float heading_error_degrees);
} 