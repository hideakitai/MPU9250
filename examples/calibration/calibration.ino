#include "MPU9250.h"

MPU9250 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) { // change to your own address
        while(1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    delay(5000);

    // calibrate anytime you want to
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();

    mpu.printCalibration();
}

void loop() {
}
