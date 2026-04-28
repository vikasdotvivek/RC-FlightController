// pid roll  <kp> <ki> <kd>
// pid pitch <kp> <ki> <kd>
// pid yaw   <kp> <ki> <kd>
// pid all   <rkp> <rki> <rkd> <pkp> <pki> <pkd> <ykp> <yki> <ykd>
// show pid
// cancel
// help

#include <SPI.h>
#include <LoRa.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ESP32-Feather + SX1276/SX1278 wiring.
// Change these only if your receiver module is wired differently.
constexpr int SCK_PIN = 5;
constexpr int MOSI_PIN = 19;
constexpr int MISO_PIN = 21;
constexpr int CS_PIN = 26;
constexpr int RST_PIN = 4;
constexpr int IRQ_PIN = 39;

constexpr long LORA_FREQ = 915E6;
constexpr uint8_t SYNC_WORD = 0xF3;
constexpr int SPREADING_FACTOR = 7;
constexpr unsigned long SERIAL_BAUD = 921600UL;
constexpr unsigned long PID_RETRY_INTERVAL_MS = 750UL;
constexpr uint32_t MAX_PID_RETRY_ATTEMPTS = 10UL;
constexpr size_t SERIAL_LINE_BUFFER_SIZE = 160U;

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

struct TelemetryPacket {
  float roll;
  float pitch;
  float throttle;
  float yaw;
  float des_roll;
  float des_pitch;
  float des_throttle;
  float des_yaw;
  float altitude;
  float des_altitude;
  float airspeed;
  float gps_lat;
  float gps_long;
  float gps_alt;
  float gps_speed;
  float gps_heading;
  int32_t gps_sats;
  int32_t gps_fix_quality;
  int32_t gps_lock_acquired;
  float baro_altitude;
  float flightmode;
  float failsafe_status;
  float waypoint_distance;
  float waypoint_heading;
  float waypoint_target_lat;
  float waypoint_target_lon;
  float waypoint_target_alt;
  float waypoint_leg_progress;
  float waypoint_mission_progress;
  int32_t waypoint_index;
  int32_t waypoint_total;
  int32_t waypoint_mission_complete;
  uint8_t imu_healthy;
  uint8_t baro_healthy;
  uint8_t gps_healthy;
  uint8_t rx_healthy;
  uint8_t armed;
  float roll_pid_out;
  float pitch_pid_out;
  float yaw_pid_out;
  float roll_pid_kp;
  float roll_pid_ki;
  float roll_pid_kd;
  float pitch_pid_kp;
  float pitch_pid_ki;
  float pitch_pid_kd;
  float yaw_pid_kp;
  float yaw_pid_ki;
  float yaw_pid_kd;
  float altitude_pid_kp;
  float altitude_pid_ki;
  float altitude_pid_kd;
  float headingerror_pid_kp;
  float headingerror_pid_ki;
  float headingerror_pid_kd;
  float rx_throttle_pwm;
  float rx_aileron_pwm;
  float rx_elevator_pwm;
  float rx_rudder_pwm;
  float rx_mode_pwm;
};

struct PendingPIDCommandState {
  bool active;
  LoRaPIDCommandPacket packet;
  uint32_t attempt_count;
  unsigned long last_send_ms;
};

static_assert(sizeof(LoRaPIDCommandPacket) == 44, "PID command packet size mismatch");
static_assert(sizeof(LoRaPIDAckPacket) == 44, "PID ack packet size mismatch");
static_assert(sizeof(TelemetryPacket) == 228, "Telemetry packet size mismatch");

uint32_t g_packet_counter = 0;
uint8_t g_next_pid_sequence = 1U;
char g_serial_line[SERIAL_LINE_BUFFER_SIZE] = {};
size_t g_serial_line_length = 0U;
PendingPIDCommandState g_pending_pid_command = {};
PIDTuningValues g_last_roll_pid = {};
PIDTuningValues g_last_pitch_pid = {};
PIDTuningValues g_last_yaw_pid = {};
bool g_have_remote_pid_state = false;

void PrintPIDTuning(const char *label, const PIDTuningValues &tuning) {
  Serial.printf("%s Kp=%.3f Ki=%.3f Kd=%.3f\n",
                label,
                tuning.kp,
                tuning.ki,
                tuning.kd);
}

void PrintPIDHelp() {
  Serial.println("Commands:");
  Serial.println("  pid roll  <kp> <ki> <kd>");
  Serial.println("  pid pitch <kp> <ki> <kd>");
  Serial.println("  pid yaw   <kp> <ki> <kd>");
  Serial.println("  pid all   <rkp> <rki> <rkd> <pkp> <pki> <pkd> <ykp> <yki> <ykd>");
  Serial.println("  show pid");
  Serial.println("  cancel");
  Serial.println("  help");
  Serial.println();
}

void DumpPacketHex(size_t bytes_to_read) {
  Serial.println("Raw payload:");

  for (size_t index = 0; index < bytes_to_read; ++index) {
    const uint8_t value = static_cast<uint8_t>(LoRa.read());
    if ((index % 16U) == 0U) {
      Serial.printf("\n%03u: ", static_cast<unsigned>(index));
    }
    Serial.printf("%02X ", value);
  }

  Serial.println();
  Serial.println();
}

void PrintTelemetryPacket(const TelemetryPacket &packet, int rssi, float snr) {
  Serial.printf(
      "[%lu] RSSI=%d dBm | SNR=%.1f | mode=%.0f | sats=%ld | lock=%ld\n",
      static_cast<unsigned long>(g_packet_counter),
      rssi,
      snr,
      packet.flightmode,
      static_cast<long>(packet.gps_sats),
      static_cast<long>(packet.gps_lock_acquired));

  Serial.printf(
      "Attitude  roll=%.2f  pitch=%.2f  yaw=%.2f\n",
      packet.roll,
      packet.pitch,
      packet.yaw);

  Serial.printf(
      "Setpoint  roll=%.2f  pitch=%.2f  yaw=%.2f\n",
      packet.des_roll,
      packet.des_pitch,
      packet.des_yaw);

  Serial.printf(
      "Control   thr=%.2f  des_thr=%.2f  airspeed=%.2f\n",
      packet.throttle,
      packet.des_throttle,
      packet.airspeed);

  Serial.printf(
      "Altitude  alt=%.2f  baro=%.2f  target=%.2f\n",
      packet.altitude,
      packet.baro_altitude,
      packet.des_altitude);

  Serial.printf(
      "GPS       lat=%.6f  lon=%.6f  alt=%.2f  spd=%.2f  hdg=%.2f\n",
      packet.gps_lat,
      packet.gps_long,
      packet.gps_alt,
      packet.gps_speed,
      packet.gps_heading);

  Serial.printf(
      "Waypoint  dist=%.2f  hdg=%.2f  idx=%ld/%ld  done=%ld\n",
      packet.waypoint_distance,
      packet.waypoint_heading,
      static_cast<long>(packet.waypoint_index),
      static_cast<long>(packet.waypoint_total),
      static_cast<long>(packet.waypoint_mission_complete));

  Serial.printf(
      "WPTarget  lat=%.6f  lon=%.6f  alt=%.2f  leg=%.2f  miss=%.2f\n",
      packet.waypoint_target_lat,
      packet.waypoint_target_lon,
      packet.waypoint_target_alt,
      packet.waypoint_leg_progress,
      packet.waypoint_mission_progress);

  Serial.printf(
      "Failsafe  fs=%.0f\n",
      packet.failsafe_status);

  Serial.printf(
      "ActivePID roll_p=%.3f roll_i=%.3f roll_d=%.3f pitch_p=%.3f pitch_i=%.3f pitch_d=%.3f yaw_p=%.3f yaw_i=%.3f yaw_d=%.3f\n",
      packet.roll_pid_kp, packet.roll_pid_ki, packet.roll_pid_kd,
      packet.pitch_pid_kp, packet.pitch_pid_ki, packet.pitch_pid_kd,
      packet.yaw_pid_kp, packet.yaw_pid_ki, packet.yaw_pid_kd);

  Serial.printf(
      "NavPID    alt_p=%.3f alt_i=%.3f alt_d=%.3f hdg_p=%.3f hdg_i=%.3f hdg_d=%.3f\n",
      packet.altitude_pid_kp, packet.altitude_pid_ki, packet.altitude_pid_kd,
      packet.headingerror_pid_kp, packet.headingerror_pid_ki, packet.headingerror_pid_kd);
  Serial.println();
}

void PrintLastKnownPIDState() {
  if (!g_have_remote_pid_state) {
    Serial.println("No PID acknowledgement received yet.");
    return;
  }

  PrintPIDTuning("Roll ", g_last_roll_pid);
  PrintPIDTuning("Pitch", g_last_pitch_pid);
  PrintPIDTuning("Yaw  ", g_last_yaw_pid);
}

bool SendLoRaPacket(const uint8_t *data, size_t length) {
  const int begin_result = LoRa.beginPacket();
  const size_t bytes_written =
      (begin_result == 1) ? LoRa.write(data, length) : 0U;
  const int end_result = (begin_result == 1) ? LoRa.endPacket() : 0;
  LoRa.receive();
  return begin_result == 1 && bytes_written == length && end_result == 1;
}

void QueuePIDCommand(const LoRaPIDCommandPacket &packet) {
  if (g_pending_pid_command.active) {
    Serial.printf(
        "Still waiting for ACK on seq=%u. Type 'cancel' before queueing another PID update.\n",
        static_cast<unsigned>(g_pending_pid_command.packet.sequence));
    return;
  }

  g_pending_pid_command.active = true;
  g_pending_pid_command.packet = packet;
  g_pending_pid_command.attempt_count = 0U;
  g_pending_pid_command.last_send_ms = 0UL;

  Serial.printf("Queued PID update seq=%u mask=0x%02X\n",
                static_cast<unsigned>(packet.sequence),
                static_cast<unsigned>(packet.axis_mask));
}

void QueueAxisPIDCommand(uint8_t axis_mask,
                         const PIDTuningValues &roll,
                         const PIDTuningValues &pitch,
                         const PIDTuningValues &yaw) {
  LoRaPIDCommandPacket packet = {};
  packet.magic = LORA_PID_PROTOCOL_MAGIC;
  packet.type = static_cast<uint8_t>(LoRaMessageType::PIDCommand);
  packet.sequence = g_next_pid_sequence++;
  packet.axis_mask = axis_mask;
  packet.roll = roll;
  packet.pitch = pitch;
  packet.yaw = yaw;
  QueuePIDCommand(packet);
}

void HandlePIDAck(const LoRaPIDAckPacket &packet, int rssi, float snr) {
  if (packet.magic != LORA_PID_PROTOCOL_MAGIC ||
      packet.type != static_cast<uint8_t>(LoRaMessageType::PIDAck)) {
    Serial.println("[ACK] Ignored malformed PID ack.");
    return;
  }

  g_last_roll_pid = packet.roll;
  g_last_pitch_pid = packet.pitch;
  g_last_yaw_pid = packet.yaw;
  g_have_remote_pid_state = true;

  Serial.printf("[ACK] seq=%u status=%u mask=0x%02X RSSI=%d SNR=%.1f\n",
                static_cast<unsigned>(packet.sequence),
                static_cast<unsigned>(packet.status),
                static_cast<unsigned>(packet.axis_mask),
                rssi,
                snr);
  PrintLastKnownPIDState();

  if (g_pending_pid_command.active &&
      g_pending_pid_command.packet.sequence == packet.sequence &&
      packet.status == static_cast<uint8_t>(LoRaPIDAckStatus::Applied)) {
    Serial.printf("[ACK] seq=%u confirmed after %lu attempt(s).\n\n",
                  static_cast<unsigned>(packet.sequence),
                  static_cast<unsigned long>(g_pending_pid_command.attempt_count));
    g_pending_pid_command.active = false;
  } else {
    Serial.println();
  }
}

void ServicePendingPIDCommand() {
  if (!g_pending_pid_command.active) {
    return;
  }

  const unsigned long now = millis();
  if (g_pending_pid_command.attempt_count > 0U &&
      (now - g_pending_pid_command.last_send_ms) < PID_RETRY_INTERVAL_MS) {
    return;
  }

  if (g_pending_pid_command.attempt_count >= MAX_PID_RETRY_ATTEMPTS) {
    Serial.printf("[TX] PID seq=%u failed after %lu attempts.\n\n",
                  static_cast<unsigned>(g_pending_pid_command.packet.sequence),
                  static_cast<unsigned long>(g_pending_pid_command.attempt_count));
    g_pending_pid_command.active = false;
    return;
  }

  g_pending_pid_command.attempt_count++;
  g_pending_pid_command.last_send_ms = now;
  const bool sent = SendLoRaPacket(
      reinterpret_cast<const uint8_t *>(&g_pending_pid_command.packet),
      sizeof(g_pending_pid_command.packet));

  Serial.printf("[TX] PID seq=%u attempt=%lu %s\n",
                static_cast<unsigned>(g_pending_pid_command.packet.sequence),
                static_cast<unsigned long>(g_pending_pid_command.attempt_count),
                sent ? "sent" : "failed");
}


void CancelPendingPIDCommand() {
  if (!g_pending_pid_command.active) {
    Serial.println("No pending PID update.");
    return;
  }

  Serial.printf("Cancelled pending PID update seq=%u.\n",
                static_cast<unsigned>(g_pending_pid_command.packet.sequence));
  g_pending_pid_command.active = false;
}

void HandleSerialCommand(char *line) {
  while (*line == ' ') {
    ++line;
  }

  if (*line == '\0') {
    return;
  }

  if (strcmp(line, "help") == 0) {
    PrintPIDHelp();
    return;
  }

  if (strcmp(line, "cancel") == 0) {
    CancelPendingPIDCommand();
    return;
  }

  if (strcmp(line, "show pid") == 0 || strcmp(line, "show") == 0) {
    PrintLastKnownPIDState();
    Serial.println();
    return;
  }

  float values[9] = {};
  if (sscanf(line, "pid roll %f %f %f", &values[0], &values[1], &values[2]) == 3) {
    QueueAxisPIDCommand(LORA_PID_AXIS_ROLL,
                        {values[0], values[1], values[2]},
                        {},
                        {});
    return;
  }

  if (sscanf(line, "pid pitch %f %f %f", &values[0], &values[1], &values[2]) == 3) {
    QueueAxisPIDCommand(LORA_PID_AXIS_PITCH,
                        {},
                        {values[0], values[1], values[2]},
                        {});
    return;
  }

  if (sscanf(line, "pid yaw %f %f %f", &values[0], &values[1], &values[2]) == 3) {
    QueueAxisPIDCommand(LORA_PID_AXIS_YAW,
                        {},
                        {},
                        {values[0], values[1], values[2]});
    return;
  }

  if (sscanf(line,
             "pid all %f %f %f %f %f %f %f %f %f",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5],
             &values[6], &values[7], &values[8]) == 9) {
    QueueAxisPIDCommand(LORA_PID_AXIS_ALL,
                        {values[0], values[1], values[2]},
                        {values[3], values[4], values[5]},
                        {values[6], values[7], values[8]});
    return;
  }

  Serial.println("Unknown command. Type 'help' for PID command syntax.");
}

void ProcessSerialInput() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r' || ch == '\n') {
      if (g_serial_line_length > 0U) {
        g_serial_line[g_serial_line_length] = '\0';
        HandleSerialCommand(g_serial_line);
        g_serial_line_length = 0U;
      }
      continue;
    }

    if (g_serial_line_length + 1U < SERIAL_LINE_BUFFER_SIZE) {
      g_serial_line[g_serial_line_length++] = ch;
    }
  }
}

bool InitLoRa() {
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  LoRa.setSPI(SPI);
  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);

  if (!LoRa.begin(LORA_FREQ)) {
    return false;
  }
  
  LoRa.setSignalBandwidth(250E3);
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.receive();
  return true;
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  const unsigned long serial_wait_start = millis();
  while (!Serial && (millis() - serial_wait_start) < 2000UL) {
    delay(10);
  }

  Serial.println();
  Serial.println("ESP32 LoRa telemetry receiver booting...");
  Serial.printf("Expected telemetry payload size: %u bytes\n",
                static_cast<unsigned>(sizeof(TelemetryPacket)));
  Serial.printf("Expected PID ack payload size: %u bytes\n",
                static_cast<unsigned>(sizeof(LoRaPIDAckPacket)));

  if (!InitLoRa()) {
    Serial.println("LoRa init failed. Check SPI and module wiring.");
    while (true) {
      delay(100);
    }
  }

  Serial.println("LoRa initialized. Waiting for telemetry and PID acknowledgements...");
  PrintPIDHelp();
}

void loop() {
  ProcessSerialInput();

  const int packet_size = LoRa.parsePacket();
  if (packet_size > 0) {
    if (packet_size == static_cast<int>(sizeof(LoRaPIDAckPacket))) {
      LoRaPIDAckPacket packet = {};
      const size_t bytes_read =
          LoRa.readBytes(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      if (bytes_read == sizeof(packet)) {
        HandlePIDAck(packet, LoRa.packetRssi(), LoRa.packetSnr());
      } else {
        Serial.printf("[ACK] Short read: got %u of %u bytes\n",
                      static_cast<unsigned>(bytes_read),
                      static_cast<unsigned>(sizeof(packet)));
      }
    } else if (packet_size == static_cast<int>(sizeof(TelemetryPacket))) {
      ++g_packet_counter;

      TelemetryPacket packet = {};
      const size_t bytes_read =
          LoRa.readBytes(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

      if (bytes_read != sizeof(packet)) {
        Serial.printf(
            "[%lu] Short read: got %u of %u bytes\n",
            static_cast<unsigned long>(g_packet_counter),
            static_cast<unsigned>(bytes_read),
            static_cast<unsigned>(sizeof(packet)));
        while (LoRa.available()) {
          (void)LoRa.read();
        }
      } else {
        PrintTelemetryPacket(packet, LoRa.packetRssi(), LoRa.packetSnr());
        
        // SYNCHRONIZED TX: The Flight Controller just finished sending this telemetry
        // and has returned to RX mode. If we have a command, send it right now!
        if (g_pending_pid_command.active) {
          delay(15); // Brief 15ms window to let the FC's SPI bus switch back to RX
          ServicePendingPIDCommand();
        }
      }
    } else {
      Serial.printf("Unexpected packet size: %d bytes\n", packet_size);
      DumpPacketHex(static_cast<size_t>(packet_size));
    }
  }

  // Fallback: If telemetry connection drops completely, still retry periodically
  if (g_pending_pid_command.active && (millis() - g_pending_pid_command.last_send_ms > PID_RETRY_INTERVAL_MS)) {
    ServicePendingPIDCommand();
  }
  
  delay(10);
}
