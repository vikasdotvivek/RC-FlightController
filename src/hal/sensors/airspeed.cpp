#include "hal/sensors/airspeed.h"

#include <Wire.h>
#include <cmath>

#include "config.h"
#include "hal/sensors/sensor_bus.h"

namespace {

uint32_t g_last_init_attempt_ms = 0;
bool g_airspeed_ready = false;
bool g_airspeed_missing_logged = false;
float g_zero_offset_pa = 0.0f;

static void UpdateAirspeedAvailability(bool available) {
  if (available) {
    if (!g_airspeed_ready && SENSOR_STATUS_LOGGING_ENABLED) {
      Serial.println("MS4525DO available.");
    }

    g_airspeed_ready = true;
    g_airspeed_missing_logged = false;
    return;
  }

  if ((g_airspeed_ready || !g_airspeed_missing_logged) && SENSOR_STATUS_LOGGING_ENABLED) {
    Serial.println("MS4525DO unavailable. Continuing without airspeed data.");
  }

  g_airspeed_ready = false;
  g_airspeed_missing_logged = true;
}

static bool ProbeAirspeedLocked() {
    Wire.beginTransmission(AIRSPEED_I2C_ADDRESS);
    return Wire.endTransmission(true) == 0;
}

static bool TryInitializeAirspeed() {
  const uint32_t now_ms = millis();
  if (g_last_init_attempt_ms != 0 &&
      (now_ms - g_last_init_attempt_ms) < SENSOR_RECONNECT_INTERVAL_MS) {
    return g_airspeed_ready;
  }

  g_last_init_attempt_ms = now_ms;

  if (!SensorBus_Init()) {
    UpdateAirspeedAvailability(false);
    return false;
  }

  if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
    return false;
  }

  const bool probe_ok = ProbeAirspeedLocked();
  SensorBus_Unlock();

  if (!probe_ok) {
    UpdateAirspeedAvailability(false);
    return false;
  }

  UpdateAirspeedAvailability(true);
  return true;
}

static float PressureToAirspeedMps(float pressure_pa) {
  // Air density at sea level, kg/m^3
  constexpr float rho = 1.225f;
  // Avoid negative sqrt
  if (pressure_pa <= 0.0f) return 0.0f;
  return sqrtf(2.0f * pressure_pa / rho);
}

} // namespace

static void Airspeed_CalibrateZero() {
  float sum = 0.0f;
  int samples = 50;
  uint8_t msb, lsb;
  uint16_t raw;
  float pressure;

  for (int i = 0; i < samples; i++) {

    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) continue;

    Wire.beginTransmission(AIRSPEED_I2C_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AIRSPEED_I2C_ADDRESS, (uint8_t)2);

    if (Wire.available() == 2) {
      msb = Wire.read();
      lsb = Wire.read();

      raw = ((msb & 0x3F) << 8) | lsb;

      pressure = ((static_cast<float>(raw) - 1638.0f) *
         (2.0f * 6894.76f) / 13107.0f) - 6894.76f;

      sum += pressure;
    }

    SensorBus_Unlock();
    delay(10);
  }

  g_zero_offset_pa = sum / samples;
}

void Airspeed_Init() {
  (void)TryInitializeAirspeed();
  Airspeed_CalibrateZero();
}

void Airspeed_Read(AirspeedData &data) {
  constexpr float P_min = -6894.76f;  // -1 psi in Pa
  constexpr float P_max = 6894.76f;  // +1 psi in Pa
  int error;
  int bytes_read;
  uint8_t msb, lsb;
  uint16_t raw;
  float pressure_pa;

  if (!g_airspeed_ready && !TryInitializeAirspeed()) {
    data.healthy = false;
    return;
  }

  if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
    data.healthy = false;
    return;
  }

  Wire.beginTransmission(AIRSPEED_I2C_ADDRESS);
  Wire.write(0x00); // Request data
  error = Wire.endTransmission(false);
  if (error != 0) {
    SensorBus_Unlock();
    UpdateAirspeedAvailability(false);
    data.healthy = false;
    return;
  }

  bytes_read = Wire.requestFrom((uint8_t)AIRSPEED_I2C_ADDRESS, (uint8_t)2);
  if (bytes_read != 2) {
    SensorBus_Unlock();
    UpdateAirspeedAvailability(false);
    data.healthy = false;
    return;
  }

  msb = Wire.read();
  lsb = Wire.read();
  SensorBus_Unlock();

  // MS4525DO 14-bit data
  raw = ((msb & 0x3F) << 8) | lsb;
  // Pressure in Pa, assuming 1 psi range, 1 psi = 6894.76 Pa
  // Output range 0-16383 for -1 to +1 psi
  pressure_pa = ((static_cast<float>(raw) - 1638.0f) * (P_max - P_min) / 13107.0f) + P_min;
  pressure_pa -= g_zero_offset_pa;
  data.pressure_pa = pressure_pa;
  data.airspeed_mps = PressureToAirspeedMps(data.pressure_pa);
  data.healthy = true;
}