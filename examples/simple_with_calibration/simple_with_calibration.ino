#include "MPU9250.h"

MPU9250 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    mpu.verbose(true);
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    mpu.verbose(false);
    mpu.printCalibration();
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 50) {
            mpu.printRollPitchYaw();
            prev_ms = millis();
        }
    }
}
