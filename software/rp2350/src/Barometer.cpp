#include "Barometer.h"

#include <Adafruit_BMP5xx.h>
#include <math.h>

namespace {
  Adafruit_BMP5xx bmp;
  bool initialized = false;
}

bool Barometer::begin(TwoWire& bus, uint8_t address) {
  initialized = false;

  if (!bmp.begin(address, &bus)) {
    return false;
  }

  // Configure in standby, then start periodic measurements.
  // These are initial bench-test settings, subject to validation.
  if (!bmp.setPowerMode(BMP5XX_POWERMODE_STANDBY)) {
    return false;
  }

  if (!bmp.enablePressure(true) ||
      !bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X) ||
      !bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X) ||
      !bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_BYPASS) ||
      !bmp.setOutputDataRate(BMP5XX_ODR_10_HZ)) {
    return false;
  }

  // Read the data-ready status through I2C.
  // No physical INT wire is needed.
  if (!bmp.configureInterrupt(
          BMP5XX_INTERRUPT_LATCHED,
          BMP5XX_INTERRUPT_ACTIVE_HIGH,
          BMP5XX_INTERRUPT_PUSH_PULL,
          BMP5XX_INTERRUPT_DATA_READY,
          true)) {
    return false;
  }

  if (!bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL)) {
    return false;
  }

  initialized = true;
  return true;
}

BarometerResult Barometer::read(BarometerReading& reading) {
  if (!initialized) {
    return BarometerResult::NotInitialized;
  }

  // The library returns false for either no new data or a status-read
  // failure. main.cpp detects prolonged silence using a timeout.
  if (!bmp.dataReady()) {
    return BarometerResult::NoNewData;
  }

  if (!bmp.performReading()) {
    reading.valid = false;
    return BarometerResult::ReadFailed;
  }

  const float temperature = bmp.temperature;
  const float pressure = bmp.pressure;

  if (!isfinite(temperature) ||
      !isfinite(pressure) ||
      pressure <= 0.0f) {
    reading.valid = false;
    return BarometerResult::ReadFailed;
  }

  reading.timestamp_ms = millis();
  reading.temperature_c = temperature;
  reading.pressure_hpa = pressure;
  reading.valid = true;

  return BarometerResult::Updated;
}