#pragma once

#include <stddef.h>
#include <stdint.h>

bool lora_init();
bool lora_send(const uint8_t *data, size_t length);
bool lora_send(const char *message);
size_t lora_receive(uint8_t *buffer, size_t max_length);
bool lora_is_ready();
