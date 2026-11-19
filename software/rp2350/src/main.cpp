#include <Arduino.h>
#include <Wire.h>

#include "Barometer.h"

namespace {
  constexpr uint8_t SDA_PIN = 0;
  constexpr uint8_t SCL_PIN = 1;
  constexpr uint8_t BMP_ADDRESS = 0x47;

  constexpr uint32_t STALE_TIMEOUT_MS = 2000;
  constexpr uint32_t STATUS_INTERVAL_MS = 1000;

  BarometerReading latestReading;

  bool barometerReady = false;
  bool headerPrinted = false;

  uint32_t lastSampleOrStartupMs = 0;
  uint32_t lastStatusMs = 0;

  void printReading(const BarometerReading& reading) {
    // Acquisition continues even without a connected Serial Monitor.
    if (!Serial) {
      return;
    }

    if (!headerPrinted) {
      Serial.println(
          "timestamp_ms,temperature_c,pressure_hpa,valid");
      headerPrinted = true;
    }

    Serial.print(reading.timestamp_ms);
    Serial.print(',');
    Serial.print(reading.temperature_c, 3);
    Serial.print(',');
    Serial.print(reading.pressure_hpa, 3);
    Serial.println(",1");
  }

  void printStatus(const char* message) {
    const uint32_t now = millis();

    if (Serial &&
        static_cast<uint32_t>(now - lastStatusMs) >=
            STATUS_INTERVAL_MS) {
      // Comment prefix distinguishes status from CSV sample rows.
      Serial.print("# ");
      Serial.println(message);
      lastStatusMs = now;
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Shared bus setup belongs here so the IMU can use it later.
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);

  barometerReady = Barometer::begin(Wire, BMP_ADDRESS);
  lastSampleOrStartupMs = millis();
}

void loop() {
  if (!Serial) {
    headerPrinted = false;
  }

  if (!barometerReady) {
    printStatus(
        "BMP initialization failed; check wiring/address, then restart.");
    delay(5);
    return;
  }

  const BarometerResult result =
      Barometer::read(latestReading);

  if (result == BarometerResult::Updated) {
    lastSampleOrStartupMs = latestReading.timestamp_ms;
    printReading(latestReading);
  } else if (result == BarometerResult::ReadFailed) {
    printStatus("BMP read failed; sample invalid.");
  }

  if (static_cast<uint32_t>(
          millis() - lastSampleOrStartupMs) >= STALE_TIMEOUT_MS) {
    latestReading.valid = false;
    printStatus("BMP data timeout; latest sample invalid.");
  }

  // Short polling interval; sensor output is configured for 10 Hz.
  delay(5);
}