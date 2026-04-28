#include "flight/arming.h"
#include "hal/comms/rx_spektrum.h"
#include "config.h"
#include <Arduino.h>

static constexpr uint32_t ARM_THROTTLE_MAX_PWM  = 1050;  // < 5% stick
static constexpr uint32_t ARM_YAW_RIGHT_MIN_PWM = 1850;  // > ~85% right
static constexpr uint32_t ARM_YAW_LEFT_MAX_PWM  = 1150;  // < ~85% left
static constexpr uint32_t ARM_HOLD_MS           = 3000;

static bool g_armed = false;
static bool g_gesture_active = false;
static uint32_t g_gesture_start_ms = 0;

static void reset_gesture() {
    g_gesture_active   = false;
    g_gesture_start_ms = 0;
}

void arming_init() {
    g_armed = false;
    reset_gesture();
}

void arming_update() {
    if (!rc_data.healthy) {
        g_armed = false;
        reset_gesture();
        return;
    }

    const uint32_t now  = millis();
    const bool throttle_low   = rc_data.throttle_pwm < ARM_THROTTLE_MAX_PWM;
    const bool yaw_full_right = rc_data.rudder_pwm   > ARM_YAW_RIGHT_MIN_PWM;
    const bool yaw_full_left  = rc_data.rudder_pwm   < ARM_YAW_LEFT_MAX_PWM;

    const bool active_gesture = g_armed ? (throttle_low && yaw_full_left)
                                        : (throttle_low && yaw_full_right);

    if (active_gesture) {
        if (!g_gesture_active) {
            g_gesture_active   = true;
            g_gesture_start_ms = now;
        }
        if (now - g_gesture_start_ms >= ARM_HOLD_MS) {
            g_armed = !g_armed;
            reset_gesture();
        }
    } else {
        reset_gesture();
    }
}

bool is_armed() {
    if (!ARMING_ENABLED) return true;
    return g_armed;
}
