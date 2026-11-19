#pragma once

#include <Arduino.h>
#include <Wire.h>

struct BarometerReading {
  uint32_t timestamp_ms = 0;  // Time since MCU startup, not UTC.
  float temperature_c = NAN;
  float pressure_hpa = NAN;
  bool valid = false;
};

enum class BarometerResult {
  NotInitialized,
  NoNewData,
  ReadFailed,
  Updated
};

namespace Barometer {
  // The caller configures and starts the shared I2C bus.
  bool begin(TwoWire& bus, uint8_t address = 0x47);

  // Only replaces reading when a new, successful sample is available.
  BarometerResult read(BarometerReading& reading);
}