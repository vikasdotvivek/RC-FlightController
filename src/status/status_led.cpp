#include "status/status_led.h"

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "datatypes.h"
#include "hal/comms/lora.h"
#include "logging/sd_logger.h"

extern IMUData_raw currentIMU;
extern BarometerData baro_data;
extern GPSData gps_data;
extern AirspeedData airspeed_data;

namespace {

#if defined(PIN_NEOPIXEL)
constexpr int kNeoPixelPin = PIN_NEOPIXEL;
#else
constexpr int kNeoPixelPin = 0;
#endif

constexpr int kNeoPixelCount = 1;

enum class FaultMode {
    Solid,
    Blink
};

struct StatusFault {
    bool active;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    FaultMode mode;
};

Adafruit_NeoPixel g_status_pixel(kNeoPixelCount, kNeoPixelPin, NEO_GRB + NEO_KHZ800);
bool g_status_led_initialized = false;

void show_color(uint8_t red, uint8_t green, uint8_t blue) {
    g_status_pixel.setPixelColor(0, g_status_pixel.Color(red, green, blue));
    g_status_pixel.show();
}

void show_off() {
    g_status_pixel.clear();
    g_status_pixel.show();
}

void build_active_faults(StatusFault *faults, size_t max_faults, size_t &fault_count) {
    fault_count = 0;

    const auto push_fault = [&](bool active, uint8_t red, uint8_t green, uint8_t blue, FaultMode mode) {
        if (!active || fault_count >= max_faults) {
            return;
        }

        faults[fault_count++] = StatusFault{true, red, green, blue, mode};
    };

    push_fault(!currentIMU.healthy, 255, 0, 255, FaultMode::Solid);  // purple solid for IMU not healthy
    push_fault(!baro_data.healthy, 0, 0, 255, FaultMode::Solid);    // blue solid for barometer not healthy
    push_fault(!gps_data.healthy || !gps_data.lock_acquired, 255, 0, 0, FaultMode::Solid);  // red solid for GPS not healthy or no lock
    push_fault(LORA_LOGGING_ENABLED && !lora_is_ready(), 255, 180, 0, FaultMode::Solid);    // yellow/orange solid for LoRa not ready
    push_fault(SD_LOGGING_ENABLED && !SD_Logger_IsReady(), 255, 255, 255, FaultMode::Blink);  //  white blinking for SD card not ready
    push_fault(!airspeed_data.healthy, 0, 255, 255, FaultMode::Blink);  //  cyan blink
}

} // namespace

void StatusLED_Init() {
    g_status_pixel.begin();
    g_status_pixel.setBrightness(STATUS_LED_BRIGHTNESS);
    show_off();
    g_status_led_initialized = true;
}

void StatusLED_Update() {
    if (!g_status_led_initialized) {
        return;
    }

    StatusFault faults[6] = {};
    size_t fault_count = 0;
    build_active_faults(faults, 6, fault_count);

    if (fault_count == 0) {
        show_color(0, 255, 0);
        return;
    }

    const uint32_t now_ms = millis();
    const size_t active_index =
        (STATUS_LED_CYCLE_PERIOD_MS == 0)
            ? 0
            : ((now_ms / STATUS_LED_CYCLE_PERIOD_MS) % fault_count);
    const StatusFault &fault = faults[active_index];

    if (fault.mode == FaultMode::Blink) {
        const bool light_on =
            STATUS_LED_BLINK_PERIOD_MS == 0 ||
            ((now_ms / STATUS_LED_BLINK_PERIOD_MS) % 2U == 0U);
        if (!light_on) {
            show_off();
            return;
        }
    }

    show_color(fault.red, fault.green, fault.blue);
}
