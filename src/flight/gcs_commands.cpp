#include "flight/gcs_commands.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "math/pid.h"
#include "hal/comms/lora.h"
#include "hal/comms/lora_protocol.h"
#include "logging/sd_logger.h"
#include "config.h"

extern PIDController roll_pid;
extern PIDController pitch_pid;
extern PIDController yaw_pid;
extern PIDController altitude_pid;
extern PIDController headingerror_pid;

namespace {

PIDTuningValues g_roll_pid_tuning = {roll_kp, roll_ki, roll_kd};
PIDTuningValues g_pitch_pid_tuning = {pitch_kp, pitch_ki, pitch_kd};
PIDTuningValues g_yaw_pid_tuning = {yaw_kp, yaw_ki, yaw_kd};
LoRaPIDCommandPacket g_pending_pid_command = {};
volatile bool g_pending_pid_command_valid = false;
LoRaPIDAckPacket g_last_pid_ack = {};
volatile bool g_last_pid_ack_valid = false;
portMUX_TYPE g_pid_command_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_tuning_lock = portMUX_INITIALIZER_UNLOCKED;

bool IsPIDTuningFinite(const PIDTuningValues &tuning) {
    return isfinite(tuning.kp) && isfinite(tuning.ki) && isfinite(tuning.kd);
}

bool IsValidPIDAxisMask(uint8_t axis_mask) {
    return (axis_mask & LORA_PID_AXIS_ALL) != 0U &&
           (axis_mask & ~LORA_PID_AXIS_ALL) == 0U;
}

bool IsValidPIDCommand(const LoRaPIDCommandPacket &packet) {
    if (packet.magic != LORA_PID_PROTOCOL_MAGIC ||
        packet.type != static_cast<uint8_t>(LoRaMessageType::PIDCommand) ||
        !IsValidPIDAxisMask(packet.axis_mask)) {
        return false;
    }

    if ((packet.axis_mask & LORA_PID_AXIS_ROLL) != 0U &&
        !IsPIDTuningFinite(packet.roll)) {
        return false;
    }

    if ((packet.axis_mask & LORA_PID_AXIS_PITCH) != 0U &&
        !IsPIDTuningFinite(packet.pitch)) {
        return false;
    }

    if ((packet.axis_mask & LORA_PID_AXIS_YAW) != 0U &&
        !IsPIDTuningFinite(packet.yaw)) {
        return false;
    }

    return true;
}

void PrintPIDTuning(const char *axis_name, const PIDTuningValues &tuning) {
    Serial.printf("%s PID: Kp=%.3f Ki=%.3f Kd=%.3f\n",
                  axis_name,
                  tuning.kp,
                  tuning.ki,
                  tuning.kd);
}

void SetControllerTuning(PIDController &controller,
                         PIDTuningValues &runtime_tuning,
                         const PIDTuningValues &new_tuning,
                         const char *axis_name) {
    portENTER_CRITICAL(&g_tuning_lock);
    runtime_tuning = new_tuning;
    controller.setTuning(new_tuning.kp, new_tuning.ki, new_tuning.kd);
    portEXIT_CRITICAL(&g_tuning_lock);
    Serial.printf("[PID] %s updated to Kp=%.3f Ki=%.3f Kd=%.3f\n",
                  axis_name,
                  new_tuning.kp,
                  new_tuning.ki,
                  new_tuning.kd);
}

void QueuePendingPIDCommand(const LoRaPIDCommandPacket &packet) {
    portENTER_CRITICAL(&g_pid_command_lock);
    g_pending_pid_command = packet;
    g_pending_pid_command_valid = true;
    portEXIT_CRITICAL(&g_pid_command_lock);
}

bool TakePendingPIDCommand(LoRaPIDCommandPacket &packet) {
    bool has_pending = false;
    portENTER_CRITICAL(&g_pid_command_lock);
    if (g_pending_pid_command_valid) {
        packet = g_pending_pid_command;
        g_pending_pid_command_valid = false;
        has_pending = true;
    }
    portEXIT_CRITICAL(&g_pid_command_lock);
    return has_pending;
}

void CacheLastPIDAck(const LoRaPIDAckPacket &packet) {
    portENTER_CRITICAL(&g_pid_command_lock);
    g_last_pid_ack = packet;
    g_last_pid_ack_valid = true;
    portEXIT_CRITICAL(&g_pid_command_lock);
}

bool GetCachedPIDAck(uint8_t sequence, LoRaPIDAckPacket &packet) {
    bool found = false;
    portENTER_CRITICAL(&g_pid_command_lock);
    if (g_last_pid_ack_valid && g_last_pid_ack.sequence == sequence) {
        packet = g_last_pid_ack;
        found = true;
    }
    portEXIT_CRITICAL(&g_pid_command_lock);
    return found;
}

void SendPIDAck(const LoRaPIDAckPacket &packet, const char *context) {
    const bool ack_sent = lora_send(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    Serial.printf("[LoRa ACK] seq=%u %s (%s)\n",
                  static_cast<unsigned>(packet.sequence),
                  ack_sent ? "sent" : "send failed",
                  context);
}

} // namespace

void GCS_LoadPIDTuningsFromConfig() {
    Serial.println("Loading PID defaults from config...");
    SetControllerTuning(roll_pid, g_roll_pid_tuning, {roll_kp, roll_ki, roll_kd}, "Roll");
    SetControllerTuning(pitch_pid, g_pitch_pid_tuning, {pitch_kp, pitch_ki, pitch_kd}, "Pitch");
    SetControllerTuning(yaw_pid, g_yaw_pid_tuning, {yaw_kp, yaw_ki, yaw_kd}, "Yaw");

    if (SD_LOGGING_ENABLED && SD_Logger_IsReady()) {
        float r_kp, r_ki, r_kd, p_kp, p_ki, p_kd, y_kp, y_ki, y_kd;
        if (SD_Logger_LoadPIDConfig(r_kp, r_ki, r_kd, p_kp, p_ki, p_kd, y_kp, y_ki, y_kd)) {
            Serial.println("Valid pid_config.json found. Applying saved PID values...");
            SetControllerTuning(roll_pid, g_roll_pid_tuning, {r_kp, r_ki, r_kd}, "Roll");
            SetControllerTuning(pitch_pid, g_pitch_pid_tuning, {p_kp, p_ki, p_kd}, "Pitch");
            SetControllerTuning(yaw_pid, g_yaw_pid_tuning, {y_kp, y_ki, y_kd}, "Yaw");
        } else {
            Serial.println("No valid pid_config.json found on SD card. Proceeding with defaults.");
        }
    }

    PrintPIDTuning("Roll", g_roll_pid_tuning);
    PrintPIDTuning("Pitch", g_pitch_pid_tuning);
    PrintPIDTuning("Yaw", g_yaw_pid_tuning);
}

void GCS_ProcessIncomingPacket(const uint8_t* buffer, size_t length) {
    if (length == sizeof(LoRaPIDCommandPacket)) {
        LoRaPIDCommandPacket packet = {};
        memcpy(&packet, buffer, sizeof(packet));

        if (!IsValidPIDCommand(packet)) {
            Serial.println("[LoRa RX] Ignored invalid PID command packet.");
        } else {
            LoRaPIDAckPacket cached_ack = {};
            if (GetCachedPIDAck(packet.sequence, cached_ack)) {
                Serial.printf("[LoRa RX] Duplicate PID command seq=%u. Resending ACK.\n",
                              static_cast<unsigned>(packet.sequence));
                SendPIDAck(cached_ack, "duplicate");
            } else {
                QueuePendingPIDCommand(packet);
                Serial.printf("[LoRa RX] Queued PID command seq=%u mask=0x%02X\n",
                              static_cast<unsigned>(packet.sequence),
                              static_cast<unsigned>(packet.axis_mask));
            }
        }
    } else {
        Serial.printf("[LoRa RX] Ignored unexpected packet of %u bytes.\n",
                      static_cast<unsigned>(length));
    }
}

void GCS_ApplyPendingCommands() {
    LoRaPIDCommandPacket packet = {};
    if (!TakePendingPIDCommand(packet)) {
        return;
    }

    Serial.printf("[LoRa RX] Applying PID command seq=%u mask=0x%02X\n",
                  static_cast<unsigned>(packet.sequence),
                  static_cast<unsigned>(packet.axis_mask));

    if ((packet.axis_mask & LORA_PID_AXIS_ROLL) != 0U) {
        SetControllerTuning(roll_pid, g_roll_pid_tuning, packet.roll, "Roll");
    }
    if ((packet.axis_mask & LORA_PID_AXIS_PITCH) != 0U) {
        SetControllerTuning(pitch_pid, g_pitch_pid_tuning, packet.pitch, "Pitch");
    }
    if ((packet.axis_mask & LORA_PID_AXIS_YAW) != 0U) {
        SetControllerTuning(yaw_pid, g_yaw_pid_tuning, packet.yaw, "Yaw");
    }

    LoRaPIDAckPacket ack = {};
    ack.magic = LORA_PID_PROTOCOL_MAGIC;
    ack.type = static_cast<uint8_t>(LoRaMessageType::PIDAck);
    ack.sequence = packet.sequence;
    ack.axis_mask = packet.axis_mask;
    ack.status = static_cast<uint8_t>(LoRaPIDAckStatus::Applied);

    portENTER_CRITICAL(&g_tuning_lock);
    ack.roll = g_roll_pid_tuning;
    ack.pitch = g_pitch_pid_tuning;
    ack.yaw = g_yaw_pid_tuning;
    portEXIT_CRITICAL(&g_tuning_lock);

    CacheLastPIDAck(ack);
    SendPIDAck(ack, "applied");

    if (SD_LOGGING_ENABLED && SD_Logger_IsReady()) {
        PIDTuningValues roll_copy, pitch_copy, yaw_copy;
        portENTER_CRITICAL(&g_tuning_lock);
        roll_copy = g_roll_pid_tuning;
        pitch_copy = g_pitch_pid_tuning;
        yaw_copy = g_yaw_pid_tuning;
        portEXIT_CRITICAL(&g_tuning_lock);

        if (SD_Logger_SavePIDConfig(
                roll_copy.kp, roll_copy.ki, roll_copy.kd,
                pitch_copy.kp, pitch_copy.ki, pitch_copy.kd,
                yaw_copy.kp, yaw_copy.ki, yaw_copy.kd)) {
            Serial.println("[SD] Updated PID config saved to pid_config.json");
        } else {
            Serial.println("[SD] Failed to save PID config to SD card.");
        }
    }
}

void GCS_PopulateTelemetryTuning(telemetrydata& snapshot) {
    portENTER_CRITICAL(&g_tuning_lock);
    snapshot.roll_pid_kp = roll_pid.getkp();
    snapshot.roll_pid_ki = roll_pid.getki();
    snapshot.roll_pid_kd = roll_pid.getkd();
    snapshot.pitch_pid_kp = pitch_pid.getkp();
    snapshot.pitch_pid_ki = pitch_pid.getki();
    snapshot.pitch_pid_kd = pitch_pid.getkd();
    snapshot.yaw_pid_kp = yaw_pid.getkp();
    snapshot.yaw_pid_ki = yaw_pid.getki();
    snapshot.yaw_pid_kd = yaw_pid.getkd();
    snapshot.altitude_pid_kp = altitude_pid.getkp();
    snapshot.altitude_pid_ki = altitude_pid.getki();
    snapshot.altitude_pid_kd = altitude_pid.getkd();
    snapshot.headingerror_pid_kp = headingerror_pid.getkp();
    snapshot.headingerror_pid_ki = headingerror_pid.getki();
    snapshot.headingerror_pid_kd = headingerror_pid.getkd();
    portEXIT_CRITICAL(&g_tuning_lock);
}