#include "MPU9250.h"

MPU9250 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    mpu.setup(0x68);  // change to your own address

    delay(5000);

    // calibrate anytime you want to
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();

    mpu.printCalibration();
}

void loop() {
}
