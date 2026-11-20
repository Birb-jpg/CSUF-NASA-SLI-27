#include <Arduino.h>
#include <Wire.h>

#include "Barometer.h"

static bool bmpReady = false;
static bool calibrated = false;
static bool headerPrinted = false;
static uint32_t lastStatus_ms = 0;

void setup()
{
    Serial.begin(115200);

    // Shared I2C bus: GP0 = SDA, GP1 = SCL.
    Wire.setSDA(0);
    Wire.setSCL(1);
    Wire.begin();
    Wire.setClock(100000);

    bmpReady = initBMP(Wire, 0x47);

    if (bmpReady) {
        // Keep the board stationary during startup calibration.
        calibrated = calibrateBMP();
    }
}

void loop()
{
    // Allow the header to be printed again after reconnecting USB serial.
    if (!Serial) {
        headerPrinted = false;
    }

    const bool newSample = bmpReady && updateBMP();

    if (Serial && !headerPrinted) {
        Serial.print("# BMP initialization: ");
        Serial.println(bmpReady ? "OK" : "FAILED");

        Serial.print("# Altitude baseline calibration: ");
        Serial.println(calibrated ? "OK" : "FAILED");

        Serial.println(
            "timestamp_ms,temperature_c,pressure_hpa,altitude_ft,valid");

        headerPrinted = true;
    }

    if (Serial && newSample) {
        Serial.print(getBMPTimestamp_ms());
        Serial.print(',');
        Serial.print(getTemperature_C(), 3);
        Serial.print(',');
        Serial.print(getPressure_hPa(), 3);
        Serial.print(',');
        Serial.print(getAltitude_ft(), 3);
        Serial.println(",1");
    }

    if (Serial &&
        static_cast<uint32_t>(
            millis() - lastStatus_ms) >= 1000) {

        lastStatus_ms = millis();

        if (!bmpReady) {
            Serial.println(
                "# Initialization failed: check wiring/address and restart.");
        } else if (!hasValidBMPReading()) {
            Serial.println("# No valid recent BMP reading.");
        }
    }

    delay(5);
}