#include "hal/sensors/airspeed.h"

#include <Wire.h>
#include <cmath>

#include "config.h"
#include "hal/sensors/sensor_bus.h"

namespace {

uint32_t g_last_init_attempt_ms = 0;
bool g_airspeed_ready = false;
bool g_airspeed_missing_logged = false;
bool g_zero_calibrated = false;
float g_zero_offset_pa = 0.0f;

constexpr int kZeroCalibrationAttempts = 50;
constexpr int kMinimumZeroCalibrationSamples = 40;
constexpr int kZeroCalibrationDelayMs = 10;
constexpr float kPressureMinPa = -6894.76f;
constexpr float kPressureMaxPa = 6894.76f;

enum class AirspeedReadResult {
  Good,
  Stale,
  Fault,
  BusError
};

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
  g_zero_calibrated = false;
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

static float RawPressureToPa(uint16_t raw) {
  // MS4525DO 10%-90% transfer function for a -1 psi to +1 psi differential part.
  return ((static_cast<float>(raw) - 1638.0f) *
          (kPressureMaxPa - kPressureMinPa) / 13107.0f) + kPressureMinPa;
}

static AirspeedReadResult ReadPressureSampleLocked(float &pressure_pa) {
  // The MS4525DO is not register-addressed. Reading starts by directly requesting
  // the four-byte measurement frame: status+pressure MSB, pressure LSB, temp MSB, temp LSB.
  const int bytes_read = Wire.requestFrom((uint8_t)AIRSPEED_I2C_ADDRESS, (uint8_t)4);
  if (bytes_read != 4 || Wire.available() < 4) {
    while (Wire.available() > 0) {
      (void)Wire.read();
    }
    return AirspeedReadResult::BusError;
  }

  const uint8_t msb = Wire.read();
  const uint8_t lsb = Wire.read();
  (void)Wire.read(); // temperature MSB
  (void)Wire.read(); // temperature LSB

  const uint8_t status = (msb >> 6) & 0x03;
  // 00 = normal, 01 = command mode, 10 = stale data, 11 = diagnostic fault.
  if (status == 0x02) {
    return AirspeedReadResult::Stale;
  }
  if (status != 0x00) {
    return AirspeedReadResult::Fault;
  }

  const uint16_t raw = (static_cast<uint16_t>(msb & 0x3F) << 8) | lsb;
  pressure_pa = RawPressureToPa(raw);
  return AirspeedReadResult::Good;
}

static float PressureToAirspeedMps(float pressure_pa) {
  constexpr float rho = 1.225f;
  if (pressure_pa <= 0.0f) return 0.0f;
  return sqrtf(2.0f * pressure_pa / rho);
}

static bool Airspeed_CalibrateZero() {
  if (!g_airspeed_ready) {
    return false;
  }

  float sum = 0.0f;
  int valid_samples = 0;

  for (int i = 0; i < kZeroCalibrationAttempts; ++i) {
    if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
      delay(kZeroCalibrationDelayMs);
      continue;
    }

    float pressure_pa = 0.0f;
    const AirspeedReadResult result = ReadPressureSampleLocked(pressure_pa);
    SensorBus_Unlock();

    if (result == AirspeedReadResult::Good) {
      sum += pressure_pa;
      ++valid_samples;
    } else if (result == AirspeedReadResult::BusError || result == AirspeedReadResult::Fault) {
      // Do not average corrupted frames into the zero point.
      if (result == AirspeedReadResult::BusError) {
        UpdateAirspeedAvailability(false);
      }
    }

    delay(kZeroCalibrationDelayMs);
  }

  if (valid_samples < kMinimumZeroCalibrationSamples) {
    g_zero_calibrated = false;
    if (SENSOR_STATUS_LOGGING_ENABLED) {
      Serial.printf("MS4525DO zero calibration failed: %d/%d valid samples.\n",
                    valid_samples, kZeroCalibrationAttempts);
    }
    return false;
  }

  g_zero_offset_pa = sum / static_cast<float>(valid_samples);
  g_zero_calibrated = true;

  if (SENSOR_STATUS_LOGGING_ENABLED) {
    Serial.printf("MS4525DO zero offset: %.2f Pa (%d samples).\n",
                  g_zero_offset_pa, valid_samples);
  }
  return true;
}

} // namespace

void Airspeed_Init() {
  if (TryInitializeAirspeed()) {
    (void)Airspeed_CalibrateZero();
  }
}

void Airspeed_Read(AirspeedData &data) {
  if (!g_airspeed_ready && !TryInitializeAirspeed()) {
    data.healthy = false;
    return;
  }

  // A reconnect must establish a new zero before its values are trusted.
  if (!g_zero_calibrated && !Airspeed_CalibrateZero()) {
    data.healthy = false;
    return;
  }

  if (!SensorBus_Lock(pdMS_TO_TICKS(SENSOR_I2C_LOCK_TIMEOUT_MS))) {
    data.healthy = false;
    return;
  }

  float pressure_pa = 0.0f;
  const AirspeedReadResult result = ReadPressureSampleLocked(pressure_pa);
  SensorBus_Unlock();

  if (result != AirspeedReadResult::Good) {
    data.healthy = false;

    if (result == AirspeedReadResult::BusError || result == AirspeedReadResult::Fault) {
      UpdateAirspeedAvailability(false);
    }
    return;
  }

  pressure_pa -= g_zero_offset_pa;
  data.pressure_pa = pressure_pa;
  data.airspeed_mps = PressureToAirspeedMps(pressure_pa);
  data.healthy = true;
  UpdateAirspeedAvailability(true);
}
