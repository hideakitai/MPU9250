# MPU9250
Arduino library for [MPU9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/) Nine-Axis (Gyro + Accelerometer + Compass) MEMS MotionTracking™ Device

This library is based on the [great work](https://github.com/kriswiner/MPU9250) by [kriswiner](https://github.com/kriswiner), and re-writen for the simple usage.

## Usage

### Simple Measurement

```
#include "MPU9250.h"

MPU9250 mpu;

void setup()
{
    Serial.begin(115200);

    Wire.begin();

    delay(2000);
    mpu.setup();
}

void loop()
{
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 16)
    {
        mpu.update();
        mpu.print();

        Serial.print("roll  (x-forward (north)) : ");
        Serial.println(mpu.getRoll());
        Serial.print("pitch (y-right (east))    : ");
        Serial.println(mpu.getPitch());
        Serial.print("yaw   (z-down (down))     : ");
        Serial.println(mpu.getYaw());

        prev_ms = millis();
    }
}
```

### Calibration

```
#include "MPU9250.h"

MPU9250 mpu;

void setup()
{
    Serial.begin(115200);

    Wire.begin();

    delay(2000);
    mpu.setup();

    delay(5000);

    // calibrate anytime you want to
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();

    mpu.printCalibration();
}

void loop()
{
}
```

## License

MIT

## Acknowledgments / Contributor

- [Yuta Asai](https://github.com/asaiyuta)
