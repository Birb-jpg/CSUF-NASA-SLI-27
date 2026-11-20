#pragma once

#include <Arduino.h>
#include <Wire.h>

// The caller starts the shared I2C bus before calling this.
bool initIMU(TwoWire& bus = Wire, uint8_t address = 0x6A);

// Polls at most once every 20 ms.
// Returns true when a reading passes the wrapper's checks.
bool updateIMU();

// False after a detected failure or 2 seconds without an accepted reading.
bool hasValidIMUReading();

// Cached measurements. Return NAN if the reading is invalid.
// Acceleration includes gravity.
float getAccelX_mps2();
float getAccelY_mps2();
float getAccelZ_mps2();

float getGyroX_radps();
float getGyroY_radps();
float getGyroZ_radps();

float getIMUTemperature_C();

// MCU acquisition time since startup, not UTC.
uint32_t getIMUTimestamp_ms();