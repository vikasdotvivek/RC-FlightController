#include "flight/telemetry.h"
#include "hal/comms/lora.h"

static_assert(sizeof(telemetrydata) <= 256,
              "telemetrydata exceeds LoRa packet size");

bool telemetry_send(const telemetrydata &data) {
    // Current transport sends a raw telemetry snapshot in a single LoRa packet.
    return lora_send(reinterpret_cast<const uint8_t *>(&data), sizeof(data));
}

//telemetry should have an interrupt for when a new packet is received from Lora?
