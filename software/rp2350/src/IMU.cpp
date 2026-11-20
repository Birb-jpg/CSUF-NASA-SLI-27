#include "IMU.h"

#include <Adafruit_LSM6DSOX.h>
#include <math.h>

/************** PRIVATE SENSOR AND STATE **************/

static Adafruit_LSM6DSOX imu;

static TwoWire* imuBus = nullptr;
static uint8_t imuAddress = 0x6A;

static bool initialized = false;
static bool readingValid = false;

static float accelX = NAN;
static float accelY = NAN;
static float accelZ = NAN;

static float gyroX = NAN;
static float gyroY = NAN;
static float gyroZ = NAN;

static float temperature_C = NAN;

static uint32_t sampleTimestamp_ms = 0;
static uint32_t lastPoll_ms = 0;

static constexpr uint32_t POLL_INTERVAL_MS = 20;
static constexpr uint32_t STALE_TIMEOUT_MS = 2000;

/************** PRIVATE HELPERS **************/

static bool sensorResponds()
{
    if (imuBus == nullptr) {
        return false;
    }

    imuBus->beginTransmission(imuAddress);
    return imuBus->endTransmission() == 0;
}

static float validValue(float value)
{
    return hasValidIMUReading() ? value : NAN;
}

/************** INITIALIZATION **************/

bool initIMU(TwoWire& bus, uint8_t address)
{
    initialized = false;
    readingValid = false;

    accelX = accelY = accelZ = NAN;
    gyroX = gyroY = gyroZ = NAN;
    temperature_C = NAN;

    sampleTimestamp_ms = 0;
    lastPoll_ms = 0;

    imuBus = &bus;
    imuAddress = address;

    if (!imu.begin_I2C(address, &bus)) {
        return false;
    }

    // Initial bench settings, not finalized flight settings.
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);

    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

    // Configuration setters return void, so check the readback.
    if (!sensorResponds() ||
        imu.getAccelRange() != LSM6DS_ACCEL_RANGE_4_G ||
        imu.getGyroRange() != LSM6DS_GYRO_RANGE_500_DPS ||
        imu.getAccelDataRate() != LSM6DS_RATE_104_HZ ||
        imu.getGyroDataRate() != LSM6DS_RATE_104_HZ) {
        return false;
    }

    initialized = true;
    lastPoll_ms = millis();

    return true;
}

/************** DATA ACQUISITION **************/

bool updateIMU()
{
    if (!initialized) {
        readingValid = false;
        return false;
    }

    const uint32_t now = millis();

    if (static_cast<uint32_t>(now - lastPoll_ms) <
        POLL_INTERVAL_MS) {
        return false;
    }

    lastPoll_ms = now;

    // Detect a missing/unresponsive device before reading.
    if (!sensorResponds()) {
        readingValid = false;
        return false;
    }

    sensors_event_t acceleration = {};
    sensors_event_t rotation = {};
    sensors_event_t temperature = {};

    const bool eventAvailable =
        imu.getEvent(&acceleration, &rotation, &temperature);

    // The library does not fully propagate internal read errors.
    // These checks catch a missing device and nonfinite output,
    // but cannot guarantee detection of every transfer failure.
    if (!eventAvailable || !sensorResponds()) {
        readingValid = false;
        return false;
    }

    if (!isfinite(acceleration.acceleration.x) ||
        !isfinite(acceleration.acceleration.y) ||
        !isfinite(acceleration.acceleration.z) ||
        !isfinite(rotation.gyro.x) ||
        !isfinite(rotation.gyro.y) ||
        !isfinite(rotation.gyro.z) ||
        !isfinite(temperature.temperature)) {
        readingValid = false;
        return false;
    }

    accelX = acceleration.acceleration.x;
    accelY = acceleration.acceleration.y;
    accelZ = acceleration.acceleration.z;

    gyroX = rotation.gyro.x;
    gyroY = rotation.gyro.y;
    gyroZ = rotation.gyro.z;

    temperature_C = temperature.temperature;

    sampleTimestamp_ms = millis();
    readingValid = true;

    return true;
}

bool hasValidIMUReading()
{
    return initialized &&
           readingValid &&
           static_cast<uint32_t>(
               millis() - sampleTimestamp_ms) < STALE_TIMEOUT_MS;
}

/************** CACHED MEASUREMENT GETTERS **************/

float getAccelX_mps2() { return validValue(accelX); }
float getAccelY_mps2() { return validValue(accelY); }
float getAccelZ_mps2() { return validValue(accelZ); }

float getGyroX_radps() { return validValue(gyroX); }
float getGyroY_radps() { return validValue(gyroY); }
float getGyroZ_radps() { return validValue(gyroZ); }

float getIMUTemperature_C()
{
    return validValue(temperature_C);
}

uint32_t getIMUTimestamp_ms()
{
    return sampleTimestamp_ms;
}