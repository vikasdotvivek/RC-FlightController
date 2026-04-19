#pragma once

#include <freertos/FreeRTOS.h>

bool SPIBus_Init();
bool SPIBus_Lock(TickType_t timeout_ticks = portMAX_DELAY);
void SPIBus_Unlock();
