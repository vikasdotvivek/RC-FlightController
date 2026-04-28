#include "hal/comms/rx_spektrum.h"
#include "config.h"
#include "math/utils.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

RCData rc_data; ////global variable to hold the latest RC data read from the receiver

enum rx_input_t {
#ifdef USE_ESC
    RX_ESC_INP = 0,
#endif
#ifdef USE_AILERON
    RX_AILERON_INP,
#endif
#ifdef USE_ELEVATOR
    RX_ELEVATOR_INP,
#endif
#ifdef USE_RUDDER
    RX_RUDDER_INP,
#endif
    RX_MODE_INP,
    NUM_RX_CHANNELS
};

const int INPUT_GPIOS[NUM_RX_CHANNELS] = {
#ifdef USE_ESC
    rx_esc_pin,
#endif
#ifdef USE_AILERON
    rx_aileron_pin,
#endif
#ifdef USE_ELEVATOR
    rx_elevator_pin,
#endif
#ifdef USE_RUDDER
    rx_rudder_pin,
#endif
    rx_mode_pin
};

// Maps channel index -> physical MCPWM capture slot.
// ESP32 has 2 units x 3 capture channels = 6 slots total.
struct CaptureSlot {
    mcpwm_unit_t            unit;
    mcpwm_io_signals_t      io_sig;        // for mcpwm_gpio_init
    mcpwm_capture_signal_t  sel;           // for mcpwm_capture_enable / get_edge / get_value
    uint32_t                int_ena_bit;   // mask for MCPWMx.int_ena.val
    uint8_t                 st_shift;      // offset from MCPWM_CAP0_INT_ST_S / _CLR_S
};

constexpr CaptureSlot CAPTURE_SLOTS[] = {
    { MCPWM_UNIT_0, MCPWM_CAP_0, MCPWM_SELECT_CAP0, MCPWM_CAP0_INT_ENA, 0 },
    { MCPWM_UNIT_0, MCPWM_CAP_1, MCPWM_SELECT_CAP1, MCPWM_CAP1_INT_ENA, 1 },
    { MCPWM_UNIT_0, MCPWM_CAP_2, MCPWM_SELECT_CAP2, MCPWM_CAP2_INT_ENA, 2 },
    { MCPWM_UNIT_1, MCPWM_CAP_0, MCPWM_SELECT_CAP0, MCPWM_CAP0_INT_ENA, 0 },
    { MCPWM_UNIT_1, MCPWM_CAP_1, MCPWM_SELECT_CAP1, MCPWM_CAP1_INT_ENA, 1 },
    { MCPWM_UNIT_1, MCPWM_CAP_2, MCPWM_SELECT_CAP2, MCPWM_CAP2_INT_ENA, 2 },
};
constexpr size_t MAX_MCPWM_CAPTURE_SLOTS = sizeof(CAPTURE_SLOTS) / sizeof(CAPTURE_SLOTS[0]);
static_assert(NUM_RX_CHANNELS <= MAX_MCPWM_CAPTURE_SLOTS,
              "Too many RX channels: ESP32 MCPWM has only 6 capture slots.");

volatile unsigned int pulse_widths[NUM_RX_CHANNELS] = {0};
uint32_t rise_time[NUM_RX_CHANNELS] = {0};

static bool is_healthy_pwm() {
    int i = 0;
    for(i=0; i<NUM_RX_CHANNELS; i++){
        if (pulse_widths[i] < SERVO_MIN || pulse_widths[i] > SERVO_MAX) {
            return false;
        }
    }
    return true;
}

static void IRAM_ATTR isr_handler(void *arg) {
    const uint32_t status0 = MCPWM0.int_st.val;
    const uint32_t status1 = MCPWM1.int_st.val;

    for (int i = 0; i < NUM_RX_CHANNELS; i++) {
        const CaptureSlot &slot = CAPTURE_SLOTS[i];
        const uint32_t active_status = (slot.unit == MCPWM_UNIT_0) ? status0 : status1;
        const uint32_t bit_mask = 1u << (MCPWM_CAP0_INT_ST_S + slot.st_shift);

        if (!(active_status & bit_mask)) continue;

        const uint32_t edge_type = mcpwm_capture_signal_get_edge(slot.unit, slot.sel);
        const uint32_t val       = mcpwm_capture_signal_get_value(slot.unit, slot.sel);

        if (edge_type == MCPWM_NEG_EDGE) {
            rise_time[i] = val;
        } else {
            // Microseconds = Ticks / (Clock Frequency in MHz)
            pulse_widths[i] = (val - rise_time[i]) / 80;
        }

        // Clear the interrupt (W1C)
        if (slot.unit == MCPWM_UNIT_0) MCPWM0.int_clr.val = bit_mask;
        else                           MCPWM1.int_clr.val = bit_mask;
    }
    rc_data.healthy = is_healthy_pwm();
#ifdef USE_AILERON
    rc_data.aileron_pwm = pulse_widths[RX_AILERON_INP];
#endif
#ifdef USE_ELEVATOR
    rc_data.elevator_pwm = pulse_widths[RX_ELEVATOR_INP];
#endif
#ifdef USE_RUDDER
    rc_data.rudder_pwm = pulse_widths[RX_RUDDER_INP];
#endif
#ifdef USE_ESC
    rc_data.throttle_pwm = pulse_widths[RX_ESC_INP];
#endif
    rc_data.flightmode_pwm = pulse_widths[RX_MODE_INP];
}

void rx_init() {
    uint32_t int_mask_0 = 0;
    uint32_t int_mask_1 = 0;

    for (int i = 0; i < NUM_RX_CHANNELS; i++) {
        const CaptureSlot &slot = CAPTURE_SLOTS[i];
        mcpwm_gpio_init(slot.unit, slot.io_sig, INPUT_GPIOS[i]);
        mcpwm_capture_enable(slot.unit, slot.sel, MCPWM_BOTH_EDGE, 0);

        if (slot.unit == MCPWM_UNIT_0) int_mask_0 |= slot.int_ena_bit;
        else                           int_mask_1 |= slot.int_ena_bit;
    }

    // Only register ISRs on units actually in use.
    if (int_mask_0) {
        mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
        MCPWM0.int_ena.val |= int_mask_0;
    }
    if (int_mask_1) {
        mcpwm_isr_register(MCPWM_UNIT_1, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
        MCPWM1.int_ena.val |= int_mask_1;
    }
}

float rx_to_angle(float raw_pwm, float max_angle){
    
    raw_pwm = math::clamp_value(raw_pwm, 1000.0f, 2000.0f);
    return ((raw_pwm-1500.0f)/500.0f) * max_angle;
    
}

float rx_to_throttle(float raw_pwm){
    if (raw_pwm < 1050) {
        raw_pwm = 1000;
    } else if (raw_pwm > 2000) {
        raw_pwm = 2000;
    }
    return (raw_pwm - 1000.0f) / 10.0f;   // 0-100 percent of stick
}

float get_des_roll() {
    float des_roll = rx_to_angle(rc_data.aileron_pwm, max_roll_angle); //////////////////outputs angles
    return des_roll;
}

float get_des_pitch() {
    float des_pitch = rx_to_angle(rc_data.elevator_pwm, max_pitch_angle);
    return des_pitch;
}
float get_des_yaw() {
    float des_yaw = rx_to_angle(rc_data.rudder_pwm, max_yaw_angle);
    return des_yaw;
}

float get_des_throttle(){
    float des_throttle = rx_to_throttle(rc_data.throttle_pwm);
    return des_throttle;
}

float get_flight_mode_pwm() {
    return rc_data.flightmode_pwm;
}

