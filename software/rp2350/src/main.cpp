#include <Arduino.h>
#include <Wire.h>

#include "IMU.h"

static bool imuReady = false;
static bool headerPrinted = false;

static uint32_t lastPrint_ms = 0;
static uint32_t lastStatus_ms = 0;

void setup()
{
    Serial.begin(115200);

    // Same shared I2C connections used for the BMP.
    Wire.setSDA(0);
    Wire.setSCL(1);
    Wire.begin();
    Wire.setClock(100000);

    imuReady = initIMU(Wire, 0x6A);
}

void loop()
{
    if (!Serial) {
        headerPrinted = false;
    }

    const bool updated = imuReady && updateIMU();
    const uint32_t now = millis();

    if (Serial && !headerPrinted) {
        Serial.print("# LSM6DSOX initialization: ");
        Serial.println(imuReady ? "OK" : "FAILED");

        Serial.println(
            "# Bench settings: +/-4 g, +/-500 deg/s, sensor ODR 104 Hz");
        Serial.println(
            "# Acquisition polling up to 50 Hz; CSV output about 10 Hz");

        Serial.println(
            "timestamp_ms,ax_mps2,ay_mps2,az_mps2,"
            "gx_radps,gy_radps,gz_radps,temperature_c,valid");

        headerPrinted = true;
    }

    // Print fewer rows than we acquire, keeping the monitor readable.
    if (Serial && updated &&
        static_cast<uint32_t>(now - lastPrint_ms) >= 100) {

        lastPrint_ms = now;

        Serial.print(getIMUTimestamp_ms());
        Serial.print(',');

        Serial.print(getAccelX_mps2(), 3);
        Serial.print(',');
        Serial.print(getAccelY_mps2(), 3);
        Serial.print(',');
        Serial.print(getAccelZ_mps2(), 3);
        Serial.print(',');

        Serial.print(getGyroX_radps(), 4);
        Serial.print(',');
        Serial.print(getGyroY_radps(), 4);
        Serial.print(',');
        Serial.print(getGyroZ_radps(), 4);
        Serial.print(',');

        Serial.print(getIMUTemperature_C(), 2);
        Serial.println(",1");
    }

    if (Serial &&
        static_cast<uint32_t>(now - lastStatus_ms) >= 1000) {

        lastStatus_ms = now;

        if (!imuReady) {
            Serial.println(
                "# IMU initialization failed: check wiring/model/address "
                "and restart.");
        } else if (!hasValidIMUReading()) {
            Serial.println("# No valid recent IMU reading.");
        }
    }

    delay(1);
}