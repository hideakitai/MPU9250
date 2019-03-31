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

- device should be stay still during accel/gyro calibration
- round device around during mag calibration

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

### 

## Other Settings

### Magnetic Declination

Magnetic declination should be set depending on where you are to get accurate data.
To set it, use this method.

```C++
mpu.setMagneticDeclination(value);
```

You can find magnetic declination in your city [here](http://www.magnetic-declination.com/).

For more details, see [wiki](https://en.wikipedia.org/wiki/Magnetic_declination).


### AFS, FGS, MFS

#### AFS

`enum class AFS { A2G, A4G, A8G, A16G };`

#### GFS

`enum class GFS { G250DPS, G500DPS, G1000DPS, G2000DPS };`

#### MFS

`enum class MFS { M14BITS, M16BITS }; // 0.6mG, 0.15mG per LSB`

#### How to change

MPU9250 class is defined as follows.

```C++
template <
	typename WireType, 
	AFS AFSSEL = AFS::A16G, 
	GFS GFSSEL = GFS::G2000DPS, 
	MFS MFSSEL = MFS::M16BITS
>
class MPU9250_;
```

So, please use like

```
class MPU9250_<TwoWire, AFS::A4G, GFS::G500DPS, MFS::M14BITS> mpu; // most of Arduino
class MPU9250_<i2c_t3, AFS::A4G, GFS::G500DPS, MFS::M14BITS> mpu; // Teensy
```


## License

MIT

## Acknowledgments / Contributor

- [Yuta Asai](https://github.com/asaiyuta)
