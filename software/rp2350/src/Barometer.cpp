#include "Barometer.h"

#include <Adafruit_BMP5xx.h>
#include <math.h>

/************** PRIVATE SENSOR AND STATE **************/

static Adafruit_BMP5xx bmp;

static bool initialized = false;
static bool readingValid = false;
static bool baselineValid = false;

static float temperature_C = NAN;
static float pressure_hPa = NAN;
static float baselinePressure_hPa = NAN;

static uint32_t sampleTimestamp_ms = 0;

static constexpr uint32_t STALE_TIMEOUT_MS = 2000;
static constexpr uint32_t CALIBRATION_TIMEOUT_MS = 3000;
static constexpr uint8_t CALIBRATION_SAMPLES = 10;

/************** INITIALIZATION **************/

bool initBMP(TwoWire& bus, uint8_t address)
{
    initialized = false;
    readingValid = false;
    baselineValid = false;

    temperature_C = NAN;
    pressure_hPa = NAN;
    baselinePressure_hPa = NAN;
    sampleTimestamp_ms = 0;

    if (!bmp.begin(address, &bus)) {
        return false;
    }

    // Configure while measurements are stopped.
    if (!bmp.setPowerMode(BMP5XX_POWERMODE_STANDBY)) {
        return false;
    }

    // Initial bench settings: 10 Hz, temperature x2, pressure x16,
    // and no IIR filtering.
    if (!bmp.enablePressure(true) ||
        !bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X) ||
        !bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X) ||
        !bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_BYPASS) ||
        !bmp.setOutputDataRate(BMP5XX_ODR_10_HZ)) {
        return false;
    }

    // Enable the data-ready status. It is polled through I2C;
    // the physical INT pin does not need to be connected.
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

/************** DATA ACQUISITION **************/

bool updateBMP()
{
    if (!initialized) {
        readingValid = false;
        return false;
    }

    // False can mean no new sample or a failed status transaction.
    // hasValidBMPReading() also checks sample age to catch silence.
    if (!bmp.dataReady()) {
        return false;
    }

    if (!bmp.performReading()) {
        readingValid = false;
        return false;
    }

    const float newTemperature = bmp.temperature;
    const float newPressure = bmp.pressure;  // Library returns hPa.

    if (!isfinite(newTemperature) ||
        !isfinite(newPressure) ||
        newPressure <= 0.0f) {
        readingValid = false;
        return false;
    }

    temperature_C = newTemperature;
    pressure_hPa = newPressure;
    sampleTimestamp_ms = millis();
    readingValid = true;

    return true;
}

bool hasValidBMPReading()
{
    return initialized &&
           readingValid &&
           static_cast<uint32_t>(
               millis() - sampleTimestamp_ms) < STALE_TIMEOUT_MS;
}

/************** BASELINE CALIBRATION **************/

bool calibrateBMP()
{
    baselineValid = false;
    baselinePressure_hPa = NAN;

    if (!initialized) {
        return false;
    }

    double pressureSum = 0.0;
    uint8_t sampleCount = 0;
    const uint32_t startTime = millis();

    // Only count new samples reported by updateBMP().
    while (sampleCount < CALIBRATION_SAMPLES &&
           static_cast<uint32_t>(
               millis() - startTime) < CALIBRATION_TIMEOUT_MS) {

        if (updateBMP()) {
            pressureSum += pressure_hPa;
            ++sampleCount;
        }

        delay(5);
    }

    if (sampleCount != CALIBRATION_SAMPLES) {
        return false;
    }

    baselinePressure_hPa =
        static_cast<float>(pressureSum / CALIBRATION_SAMPLES);

    baselineValid = true;
    return true;
}

/************** CACHED MEASUREMENT GETTERS **************/

float getTemperature_C()
{
    return hasValidBMPReading() ? temperature_C : NAN;
}

float getPressure_hPa()
{
    return hasValidBMPReading() ? pressure_hPa : NAN;
}

float getAltitude_ft()
{
    if (!baselineValid || !hasValidBMPReading()) {
        return NAN;
    }

    // Approximate pressure altitude relative to the calibration location.
    // This uses the same model as the library's readAltitude(), but
    // uses our cached pressure so all outputs share one acquired sample.
    const float pressureRatio = pressure_hPa / baselinePressure_hPa;

    const float altitude_m =
        44330.0f * (1.0f - powf(pressureRatio, 0.1903f));

    return altitude_m * 3.28084f;
}

float getBaselinePressure_hPa()
{
    return baselineValid ? baselinePressure_hPa : NAN;
}

uint32_t getBMPTimestamp_ms()
{
    return sampleTimestamp_ms;
}