#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <string.h>
#include <freertos/semphr.h>
#include "hal/comms/lora.h"
#include "hal/comms/spi_bus.h"
#include "../include/config.h"

namespace {

SemaphoreHandle_t g_lora_mutex = nullptr;
bool g_lora_ready = false;

bool take_lora_lock() {
  return (g_lora_mutex != nullptr) &&
         (xSemaphoreTake(g_lora_mutex, portMAX_DELAY) == pdTRUE);
}

void give_lora_lock() {
  if (g_lora_mutex != nullptr) {
    xSemaphoreGive(g_lora_mutex);
  }
}

bool take_spi_lock() {
  return SPIBus_Lock(pdMS_TO_TICKS(SPI_BUS_LOCK_TIMEOUT_MS));
}

void deselect_sd() {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
}

void enter_receive_mode() {
  LoRa.receive();
}

}  // namespace

bool lora_init() {
  if (g_lora_ready) {
    return true;
  }

  if (g_lora_mutex == nullptr) {
    g_lora_mutex = xSemaphoreCreateMutex();
    if (g_lora_mutex == nullptr) {
      return false;
    }
  }

  if (!SPIBus_Init()) {
    return false;
  }

  if (!take_lora_lock()) {
    return false;
  }

  if (!take_spi_lock()) {
    give_lora_lock();
    return false;
  }

  deselect_sd();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
  LoRa.setSPI(SPI);
  LoRa.setSPIFrequency(SPI_BUS_FREQUENCY_HZ);
  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);

  if (!LoRa.begin(LORA_FREQ)) {
    SPIBus_Unlock();
    give_lora_lock();
    return false;
  }

  LoRa.setSignalBandwidth(250E3);
  LoRa.setSpreadingFactor(7);
  LoRa.setSyncWord(SYNC_WORD);
  enter_receive_mode();
  SPIBus_Unlock();
  give_lora_lock();
  g_lora_ready = true;
  return true;
}

bool lora_send(const uint8_t *data, size_t length) {
  if (!g_lora_ready || data == nullptr || length == 0 || !take_lora_lock()) {
    return false;
  }

  if (!take_spi_lock()) {
    give_lora_lock();
    return false;
  }

  deselect_sd();

  const int begin_result = LoRa.beginPacket();
  const size_t bytes_written =
      (begin_result == 1) ? LoRa.write(data, length) : 0;
  const int end_result = (begin_result == 1) ? LoRa.endPacket() : 0;
  if (begin_result == 1 && end_result == 1) {
    enter_receive_mode();
  }

  SPIBus_Unlock();
  give_lora_lock();
  return begin_result == 1 && bytes_written == length && end_result == 1;
}

bool lora_send(const char *message) {
  if (message == nullptr) {
    return false;
  }

  return lora_send(reinterpret_cast<const uint8_t *>(message), strlen(message));
}

size_t lora_receive(uint8_t *buffer, size_t max_length) {
  if (!g_lora_ready || buffer == nullptr || max_length == 0 || !take_lora_lock()) {
    return 0;
  }

  if (!take_spi_lock()) {
    give_lora_lock();
    return 0;
  }

  deselect_sd();

  const int packet_size = LoRa.parsePacket();
  if (packet_size <= 0) {
    SPIBus_Unlock();
    give_lora_lock();
    return 0;
  }

  size_t bytes_read = 0;
  while (LoRa.available() && bytes_read < max_length) {
    buffer[bytes_read++] = static_cast<uint8_t>(LoRa.read());
  }

  while (LoRa.available()) {
    (void)LoRa.read();
  }

  enter_receive_mode();

  SPIBus_Unlock();
  give_lora_lock();
  return bytes_read;
}

bool lora_is_ready() {
  return g_lora_ready;
}
