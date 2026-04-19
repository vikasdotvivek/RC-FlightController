#include "hal/comms/spi_bus.h"

#include <freertos/semphr.h>

namespace {

SemaphoreHandle_t g_spi_bus_mutex = nullptr;

} // namespace

bool SPIBus_Init() {
    if (g_spi_bus_mutex == nullptr) {
        g_spi_bus_mutex = xSemaphoreCreateMutex();
        if (g_spi_bus_mutex == nullptr) {
            return false;
        }
    }

    return true;
}

bool SPIBus_Lock(TickType_t timeout_ticks) {
    return (g_spi_bus_mutex != nullptr) &&
           (xSemaphoreTake(g_spi_bus_mutex, timeout_ticks) == pdTRUE);
}

void SPIBus_Unlock() {
    if (g_spi_bus_mutex != nullptr) {
        xSemaphoreGive(g_spi_bus_mutex);
    }
}
