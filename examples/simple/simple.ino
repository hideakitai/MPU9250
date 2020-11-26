#include "MPU9250.h"

MPU9250 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    mpu.setup(0x68);  // change to your own address
}

void loop() {
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 16) {
        mpu.update();
        mpu.printRollPitchYaw();
        prev_ms = millis();
    }
}
