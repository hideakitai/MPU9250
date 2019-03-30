#include "MPU9250.h"
#include "eeprom_utils.h"

MPU9250 mpu;

void setup()
{
    Serial.begin(115200);

    Wire.begin();

    delay(2000);
    mpu.setup(Wire);

    delay(5000);

    // calibrate when you want to
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();

    // save to eeprom
    saveCalibration();

    // load from eeprom
    loadCalibration();

    mpu.printCalibration();
}

void loop()
{
}
