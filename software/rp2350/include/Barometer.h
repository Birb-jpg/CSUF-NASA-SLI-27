#pragma once

#include <Arduino.h>
#include <Wire.h>

// Initialize and configure the BMP581 on an already-started I2C bus.
bool initBMP(TwoWire& bus = Wire, uint8_t address = 0x47);

// Establish a local altitude reference from ten new pressure samples.
// Keep the sensor stationary. Returns within approximately 3 seconds.
bool calibrateBMP();

// Retrieve a new pressure/temperature sample.
// Returns true only when a new, valid sample was acquired.
bool updateBMP();

// Check whether the cached reading is valid and less than 2 seconds old.
bool hasValidBMPReading();

// Getters return cached data; they do not trigger additional sensor reads.
// Measurement getters return NAN when their required data is invalid.
float getTemperature_C();
float getPressure_hPa();
float getAltitude_ft();
float getBaselinePressure_hPa();

// Milliseconds since startup when the last successful sample was read.
// This timestamp alone does not establish validity.
uint32_t getBMPTimestamp_ms();