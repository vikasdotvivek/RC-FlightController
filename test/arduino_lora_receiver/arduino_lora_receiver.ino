#include <SPI.h>
#include <LoRa.h>
#include <stdint.h>

// ESP32-WROOM DevKit + SX1276/SX1278 wiring.
// Change these only if your receiver module is wired differently.
constexpr int SCK_PIN = 18;
constexpr int MISO_PIN = 19;
constexpr int MOSI_PIN = 23;
constexpr int CS_PIN = 5;
constexpr int RST_PIN = 4;
constexpr int IRQ_PIN = 2;

constexpr long LORA_FREQ = 915E6;
constexpr uint8_t SYNC_WORD = 0xF3;
constexpr int SPREADING_FACTOR = 7;
constexpr unsigned long SERIAL_BAUD = 115200UL;

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
};

static_assert(sizeof(TelemetryPacket) == 128, "Telemetry packet size mismatch");

uint32_t g_packet_counter = 0;

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

void PrintPacket(const TelemetryPacket &packet, int rssi, float snr) {
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

  Serial.println();
}

bool InitLoRa() {
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  LoRa.setSPI(SPI);
  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);

  if (!LoRa.begin(LORA_FREQ)) {
    return false;
  }

  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
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
  Serial.printf("Expected payload size: %u bytes\n", static_cast<unsigned>(sizeof(TelemetryPacket)));

  if (!InitLoRa()) {
    Serial.println("LoRa init failed. Check SPI and module wiring.");
    while (true) {
      delay(100);
    }
  }

  Serial.println("LoRa initialized. Waiting for telemetry packets...");
  Serial.println();
}

void loop() {
  const int packet_size = LoRa.parsePacket();
  if (packet_size <= 0) {
    delay(10);
    return;
  }

  ++g_packet_counter;

  if (packet_size != static_cast<int>(sizeof(TelemetryPacket))) {
    Serial.printf(
        "[%lu] Unexpected packet size: %d bytes, expected %u bytes\n",
        static_cast<unsigned long>(g_packet_counter),
        packet_size,
        static_cast<unsigned>(sizeof(TelemetryPacket)));
    DumpPacketHex(static_cast<size_t>(packet_size));
    return;
  }

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
    return;
  }

  PrintPacket(packet, LoRa.packetRssi(), LoRa.packetSnr());
}
