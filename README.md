# MPU9250
Arduino library for [MPU9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/) Nine-Axis (Gyro + Accelerometer + Compass) MEMS MotionTracking™ Device

This library is based on the [great work](https://github.com/kriswiner/MPU9250) by [kriswiner](https://github.com/kriswiner), and re-writen for the simple usage.

## Usage

### Simple Measurement

``` C++
#include "MPU9250.h"

MPU9250 mpu;
// MPU9255 mpu; // You can also use MPU9255

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    mpu.setup(0x68);  // change to your own address
}

void loop() {
    if (mpu.update()) {
        mpu.printRollPitchYaw();
    }
}
```

### Calibration

- device should be stay still during accel/gyro calibration
- round device around during mag calibration

``` C++
#include "MPU9250.h"

MPU9250 mpu;
// MPU9255 mpu; // You can also use MPU9255

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

void loop() { }
```


## Other Settings

### I2C Address

You must set your own address based on A0, A1, A2 setting as:

``` C++
mpu.setup(0x70);
```

### Other I2C library

You can use other I2C library e.g. [SoftWire](https://github.com/stevemarple/SoftWire).

``` C++
MPU9250_<SoftWire, MPU9250_WHOAMI_DEFAULT_VALUE> mpu;
SoftWire sw(SDA, SCL);

// in setup()
mpu.setup(0x70, sw);
```

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

`enum class MFS { M14BITS, M16BITS };

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

### MPU9255

To use MPU9255 instead of MPU9250, just declare MPU9255.

```C++
MPU9255 mpu;
```

## APIs

``` C++
bool setup(const uint8_t addr, WireType& w = Wire);
void verbose(const bool b);
void calibrateAccelGyro();
void calibrateMag();
bool isConnectedMPU9250();
bool isConnectedAK8963();
bool available();
bool update();

float getRoll() const;
float getPitch() const;
float getYaw() const;

float getQuaternion(uint8_t i) const;

float getQuaternionX() const;
float getQuaternionY() const;
float getQuaternionZ() const;
float getQuaternionW() const;

float getAcc(const uint8_t i) const;
float getGyro(const uint8_t i) const;
float getMag(const uint8_t i) const;

float getAccX() const;
float getAccY() const;
float getAccZ() const;
float getGyroX() const;
float getGyroY() const;
float getGyroZ() const;
float getMagX() const;
float getMagY() const;
float getMagZ() const;

float getAccBias(const uint8_t i) const;
float getGyroBias(const uint8_t i) const;
float getMagBias(const uint8_t i) const;
float getMagScale(const uint8_t i) const;

float getAccBiasX() const;
float getAccBiasY() const;
float getAccBiasZ() const;
float getGyroBiasX() const;
float getGyroBiasY() const;
float getGyroBiasZ() const;
float getMagBiasX() const;
float getMagBiasY() const;
float getMagBiasZ() const;
float getMagScaleX() const;
float getMagScaleY() const;
float getMagScaleZ() const;

float getTemperature() const;

void setAccBias(const uint8_t i, const float v);
void setGyroBias(const uint8_t i, const float v);
void setMagBias(const uint8_t i, const float v);
void setMagScale(const uint8_t i, const float v);

void setAccBiasX(const float v);
void setAccBiasY(const float v);
void setAccBiasZ(const float v);
void setGyroBiasX(const float v);
void setGyroBiasY(const float v);
void setGyroBiasZ(const float v);
void setMagBiasX(const float v);
void setMagBiasY(const float v);
void setMagBiasZ(const float v);
void setMagScaleX(const float v);
void setMagScaleY(const float v);
void setMagScaleZ(const float v);

void setMagneticDeclination(const float d);

void print() const;
void printRawData() const;
void printRollPitchYaw() const;
void printCalibration() const;
```

## License

MIT

## Acknowledgments / Contributor

- [Yuta Asai](https://github.com/asaiyuta)
