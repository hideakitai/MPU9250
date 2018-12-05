#include "EEPROM.h"
#include "MPU9250.h"

const uint8_t EEPROM_SIZE = 1 + sizeof(float) * 3 * 4;
bool b_calibrated = false;
extern MPU9250 mpu;

enum EEP_ADDR
{
    EEP_CALIB_FLAG = 0x00,
    EEP_ACC_BIAS = 0x01,
    EEP_GYRO_BIAS = 0x0D,
    EEP_MAG_BIAS = 0x19,
    EEP_MAG_SCALE = 0x25
};

void saveCalibration(bool b_save = true)
{
    EEPROM.writeByte(EEP_CALIB_FLAG, 1);
    EEPROM.writeFloat(EEP_ACC_BIAS + 0, mpu.getAccBias(0));
    EEPROM.writeFloat(EEP_ACC_BIAS + 4, mpu.getAccBias(1));
    EEPROM.writeFloat(EEP_ACC_BIAS + 8, mpu.getAccBias(2));
    EEPROM.writeFloat(EEP_GYRO_BIAS + 0, mpu.getGyroBias(0));
    EEPROM.writeFloat(EEP_GYRO_BIAS + 4, mpu.getGyroBias(1));
    EEPROM.writeFloat(EEP_GYRO_BIAS + 8, mpu.getGyroBias(2));
    EEPROM.writeFloat(EEP_MAG_BIAS + 0, mpu.getMagBias(0));
    EEPROM.writeFloat(EEP_MAG_BIAS + 4, mpu.getMagBias(1));
    EEPROM.writeFloat(EEP_MAG_BIAS + 8, mpu.getMagBias(2));
    EEPROM.writeFloat(EEP_MAG_SCALE + 0, mpu.getMagScale(0));
    EEPROM.writeFloat(EEP_MAG_SCALE + 4, mpu.getMagScale(1));
    EEPROM.writeFloat(EEP_MAG_SCALE + 8, mpu.getMagScale(2));
    if (b_save) EEPROM.commit();
}

void loadCalibration()
{
    if (EEPROM.readByte(EEP_CALIB_FLAG) == 0x01)
    {
        Serial.println("calibrated? : YES");
        Serial.println("load calibrated values");
        mpu.setAccBias(0, EEPROM.readFloat(EEP_ACC_BIAS + 0));
        mpu.setAccBias(1, EEPROM.readFloat(EEP_ACC_BIAS + 4));
        mpu.setAccBias(2, EEPROM.readFloat(EEP_ACC_BIAS + 8));
        mpu.setGyroBias(0, EEPROM.readFloat(EEP_GYRO_BIAS + 0));
        mpu.setGyroBias(1, EEPROM.readFloat(EEP_GYRO_BIAS + 4));
        mpu.setGyroBias(2, EEPROM.readFloat(EEP_GYRO_BIAS + 8));
        mpu.setMagBias(0, EEPROM.readFloat(EEP_MAG_BIAS + 0));
        mpu.setMagBias(1, EEPROM.readFloat(EEP_MAG_BIAS + 4));
        mpu.setMagBias(2, EEPROM.readFloat(EEP_MAG_BIAS + 8));
        mpu.setMagScale(0, EEPROM.readFloat(EEP_MAG_SCALE + 0));
        mpu.setMagScale(1, EEPROM.readFloat(EEP_MAG_SCALE + 4));
        mpu.setMagScale(2, EEPROM.readFloat(EEP_MAG_SCALE + 8));
    }
    else
    {
        Serial.println("calibrated? : NO");
        Serial.println("load default values");
        mpu.setAccBias(0, +0.005);
        mpu.setAccBias(1, -0.008);
        mpu.setAccBias(2, -0.001);
        mpu.setGyroBias(0, +1.5);
        mpu.setGyroBias(1, -0.5);
        mpu.setGyroBias(2, +0.7);
        mpu.setMagBias(0, +186.41);
        mpu.setMagBias(1, -197.91);
        mpu.setMagBias(2, -425.55);
        mpu.setMagScale(0, +1.07);
        mpu.setMagScale(1, +0.95);
        mpu.setMagScale(2, +0.99);
    }
}

void clearCalibration()
{
    for (size_t i = 0; i < EEPROM_SIZE; ++i)
        EEPROM.writeByte(i, 0xFF);
    EEPROM.commit();
}

bool isCalibrated()
{
    return (EEPROM.readByte(EEP_CALIB_FLAG) == 0x01);
}

void printCalibration()
{
    Serial.println("< calibration parameters >");
    Serial.print("calibrated? : ");
    Serial.println(EEPROM.readByte(EEP_CALIB_FLAG) ? "YES" : "NO");
    Serial.print("acc bias x  : ");
    Serial.println(EEPROM.readFloat(EEP_ACC_BIAS + 0) * 1000.f);
    Serial.print("acc bias y  : ");
    Serial.println(EEPROM.readFloat(EEP_ACC_BIAS + 4) * 1000.f);
    Serial.print("acc bias z  : ");
    Serial.println(EEPROM.readFloat(EEP_ACC_BIAS + 8) * 1000.f);
    Serial.print("gyro bias x : ");
    Serial.println(EEPROM.readFloat(EEP_GYRO_BIAS + 0));
    Serial.print("gyro bias y : ");
    Serial.println(EEPROM.readFloat(EEP_GYRO_BIAS + 4));
    Serial.print("gyro bias z : ");
    Serial.println(EEPROM.readFloat(EEP_GYRO_BIAS + 8));
    Serial.print("mag bias x  : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_BIAS + 0));
    Serial.print("mag bias y  : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_BIAS + 4));
    Serial.print("mag bias z  : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_BIAS + 8));
    Serial.print("mag scale x : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_SCALE + 0));
    Serial.print("mag scale y : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_SCALE + 4));
    Serial.print("mag scale z : ");
    Serial.println(EEPROM.readFloat(EEP_MAG_SCALE + 8));
}

void printBytes()
{
    for (size_t i = 0; i < EEPROM_SIZE; ++i)
        Serial.println(EEPROM.readByte(i), HEX);
}

void setupEEPROM()
{
    Serial.println("EEPROM start");
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        Serial.println("EEPROM start failed");
    }

    b_calibrated = isCalibrated();
    if (!b_calibrated)
    {
        Serial.println("Need Calibration!!");
    }
    Serial.println("EEPROM calibration value is : ");
    printCalibration();
    Serial.println("Loaded calibration value is : ");
    loadCalibration();
}
