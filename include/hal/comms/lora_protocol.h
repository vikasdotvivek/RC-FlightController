#pragma once

#include <stdint.h>

struct PIDTuningValues {
    float kp;
    float ki;
    float kd;
};

constexpr uint32_t LORA_PID_PROTOCOL_MAGIC = 0x52434643UL;
constexpr uint8_t LORA_PID_AXIS_ROLL  = 1U << 0;
constexpr uint8_t LORA_PID_AXIS_PITCH = 1U << 1;
constexpr uint8_t LORA_PID_AXIS_YAW   = 1U << 2;
constexpr uint8_t LORA_PID_AXIS_ALL =
    LORA_PID_AXIS_ROLL | LORA_PID_AXIS_PITCH | LORA_PID_AXIS_YAW;

enum class LoRaMessageType : uint8_t {
    PIDCommand = 1,
    PIDAck = 2
};

enum class LoRaPIDAckStatus : uint8_t {
    Applied = 1,
    Invalid = 2
};

struct LoRaPIDCommandPacket {
    uint32_t magic;
    uint8_t type;
    uint8_t sequence;
    uint8_t axis_mask;
    uint8_t reserved;
    PIDTuningValues roll;
    PIDTuningValues pitch;
    PIDTuningValues yaw;
};

struct LoRaPIDAckPacket {
    uint32_t magic;
    uint8_t type;
    uint8_t sequence;
    uint8_t axis_mask;
    uint8_t status;
    PIDTuningValues roll;
    PIDTuningValues pitch;
    PIDTuningValues yaw;
};

static_assert(sizeof(LoRaPIDCommandPacket) == 44,
              "Unexpected LoRa PID command packet size");
static_assert(sizeof(LoRaPIDAckPacket) == 44,
              "Unexpected LoRa PID ack packet size");
