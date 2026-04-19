#include "hal/actuators/pwm_out.h"
#include "config.h"


void pwm_init() {
    // Set up PWM channels with a frequency of 50Hz and 16-bit resolution
    ledcSetup(esc_channel, 50, 16);
    ledcSetup(aileron_channel, 50, 16);
    ledcSetup(elevator_channel, 50, 16);
    ledcSetup(rudder_channel, 50, 16);

    //////attach the channels to the corresponding GPIO pins
    ledcAttachPin(esc_pin, esc_channel);
    ledcAttachPin(aileron_pin, aileron_channel);
    ledcAttachPin(elevator_pin, elevator_channel);
    ledcAttachPin(rudder_pin, rudder_channel);

    pwm_reset();
}

void pwm_write(uint8_t channel, uint16_t microseconds) {

    microseconds = constrain(microseconds, 1000, 2000);

    // convert microseconds to duty cycle based on 16-bit resolution
    uint32_t duty = (microseconds * 65535) / 20000; // 20ms 
    ledcWrite(channel, duty);
}

void pwm_reset() {
    // Keep throttle at idle while resetting surfaces.
    pwm_write(esc_channel, PWM_ESC_MIN_US);

    // Sweep control surfaces from minimum to maximum PWM on reset.
    for (int pwm_us = 1000; // Start from minimum
         pwm_us <= 2000;   // End at maximum 
         pwm_us += PWM_RESET_SWEEP_STEP_US) {
        const uint16_t pulse = static_cast<uint16_t>(pwm_us);
        pwm_write(aileron_channel, pulse);
        pwm_write(elevator_channel, pulse);
        pwm_write(rudder_channel, pulse);
        delay(PWM_RESET_SWEEP_DELAY_MS);
    }

    // Return to neutral after sweep.
    pwm_write_surface_angle(aileron_channel, SURFACE_NEUTRAL_ANGLE_DEG);
    pwm_write_surface_angle(elevator_channel, SURFACE_NEUTRAL_ANGLE_DEG);
    pwm_write_surface_angle(rudder_channel, SURFACE_NEUTRAL_ANGLE_DEG);
}
