#pragma once
#ifndef MPU9250_H
#define MPU9250_H

#ifdef TEENSYDUINO
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

#include "MPU9250/MPU9250RegisterMap.h"
#include "MPU9250/QuaternionFilter.h"


enum class AFS { A2G, A4G, A8G, A16G };
enum class GFS { G250DPS, G500DPS, G1000DPS, G2000DPS };
enum class MFS { M14BITS, M16BITS }; // 0.6mG, 0.15mG per LSB


template <typename WireType, AFS AFSSEL = AFS::A16G, GFS GFSSEL = GFS::G2000DPS, MFS MFSSEL = MFS::M16BITS>
class MPU9250_
{
    const uint8_t MPU9250_ADDRESS {0x68};  // Device address when ADO = 0
    const uint8_t AK8963_ADDRESS {0x0C};   //  Address of magnetometer

    const uint8_t MPU9250_WHOAMI_DEFAULT_VALUE {0x71}; // 0x68????
    const uint8_t AK8963_WHOAMI_DEFAULT_VALUE {0x48};

    // Set initial input parameters
    // const uint8_t Mmode {0x02};        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    const uint8_t Mmode {0x06};        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    const float aRes {getAres()};      // scale resolutions per LSB for the sensors
    const float gRes {getGres()};      // scale resolutions per LSB for the sensors
    const float mRes {getMres()};      // scale resolutions per LSB for the sensors

    float magCalibration[3] = {0, 0, 0}; // factory mag calibration
    float magBias[3] = {0, 0, 0};
    float magScale[3]  = {1.0, 1.0, 1.0}; // Bias corrections for gyro and accelerometer

    float gyroBias[3] = {0, 0, 0}; // bias corrections
    float accelBias[3] = {0, 0, 0}; // bias corrections

    int16_t tempCount;      // temperature raw count output
    float temperature;    // Stores the real internal chip temperature in degrees Celsius
    float SelfTestResult[6];    // holds results of gyro and accelerometer self test

    float a[3], g[3], m[3];
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
    float pitch, yaw, roll;
    float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
    float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)

    QuaternionFilter qFilter;

    float magnetic_declination = -7.51; // Japan, 24th June

public:

    MPU9250_() : aRes(getAres()), gRes(getGres()), mRes(getMres()) {}

    void setup(WireType& w = Wire)
    {
        wire = &w;

        uint8_t m_whoami = 0x00;
        uint8_t a_whoami = 0x00;

        m_whoami = isConnectedMPU9250();
        if (m_whoami)
        {
            Serial.println("MPU9250 is online...");
            initMPU9250();

            a_whoami = isConnectedAK8963();
            if (a_whoami)
            {
                initAK8963(magCalibration);
            }
            else
            {
                Serial.print("Could not connect to AK8963: 0x");
                Serial.println(a_whoami);
            }
        }
        else
        {
            Serial.print("Could not connect to MPU9250: 0x");
            Serial.println(m_whoami);
        }
    }

    void calibrateAccelGyro()
    {
        calibrateMPU9250(gyroBias, accelBias);
    }

    void calibrateMag()
    {
        magcalMPU9250(magBias, magScale);
    }

    bool isConnectedMPU9250()
    {
        byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
        Serial.print("MPU9250 WHO AM I = ");
        Serial.println(c, HEX);
        return (c == MPU9250_WHOAMI_DEFAULT_VALUE);
    }

    bool isConnectedAK8963()
    {
        byte c = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
        Serial.print("AK8963  WHO AM I = ");
        Serial.println(c, HEX);
        return (c == AK8963_WHOAMI_DEFAULT_VALUE);
    }

    bool available()
    {
        return (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01);
    }

    void update()
    {
        if (available())
        {  // On interrupt, check if data ready interrupt
            updateAccelGyro();
            updateMag(); // TODO: set to 30fps?
        }

        // Madgwick function needs to be fed North, East, and Down direction like
        // (AN, AE, AD, GN, GE, GD, MN, ME, MD)
        // Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
        // Magneto direction is Right-Hand, Y-Forward, Z-Down
        // So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
        // we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
        // but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
        // because gravity is by convention positive down, we need to ivnert the accel data

        // get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
        // acc[mg], gyro[deg/s], mag [mG]
        // gyro will be convert from [deg/s] to [rad/s] inside of this function
        qFilter.update(-a[0], a[1], a[2], g[0], -g[1], -g[2], m[1], -m[0], m[2], q);

        if (!b_ahrs)
        {
            tempCount = readTempData();  // Read the adc values
            temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
        }
        else
        {
            // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
            // In this coordinate system, the positive z-axis is down toward Earth.
            // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
            // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
            // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
            // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
            // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
            // applied in the correct order which for this configuration is yaw, pitch, and then roll.
            // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
            updateRPY();
        }
    }


    void setI2CAddress(uint8_t addr) {} // TODO:

    // TODO: more efficient getter, const refrerence of struct??
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }

    float getQuaternion(uint8_t i) const { return (i < 4) ? q[i] : 0.f; }

    float getAcc(uint8_t i) const { return (i < 3) ? a[i] : 0.f; }
    float getGyro(uint8_t i) const { return (i < 3) ? g[i] : 0.f; }
    float getMag(uint8_t i) const { return (i < 3) ? m[i] : 0.f; }

    float getAccBias(uint8_t i) const { return (i < 3) ? accelBias[i] : 0.f; }
    float getGyroBias(uint8_t i) const { return (i < 3) ? gyroBias[i] : 0.f; }
    float getMagBias(uint8_t i) const { return (i < 3) ? magBias[i] : 0.f; }
    float getMagScale(uint8_t i) const { return (i < 3) ? magScale[i] : 0.f; }

    void setAccBias(uint8_t i, float v) { if (i < 3) accelBias[i] = v; }
    void setGyroBias(uint8_t i, float v) { if (i < 3) gyroBias[i] = v; }
    void setMagBias(uint8_t i, float v) { if (i < 3) magBias[i] = v; }
    void setMagScale(uint8_t i, float v) { if (i < 3) magScale[i] = v; }

    void setMagneticDeclination(const float d) { magnetic_declination = d; }

    void print() const
    {
        printRawData();
        printRollPitchYaw();
        printCalibration();
    }

    void printRawData() const
    {
        // Print acceleration values in milligs!
        Serial.print("ax = "); Serial.print((int)1000 * a[0]);
        Serial.print(" ay = "); Serial.print((int)1000 * a[1]);
        Serial.print(" az = "); Serial.print((int)1000 * a[2]); Serial.println(" mg");
        // Print gyro values in degree/sec
        Serial.print("gx = "); Serial.print(g[0], 2);
        Serial.print(" gy = "); Serial.print(g[1], 2);
        Serial.print(" gz = "); Serial.print(g[2], 2); Serial.println(" deg/s");
        // Print mag values in degree/sec
        Serial.print("mx = "); Serial.print((int)m[0]);
        Serial.print(" my = "); Serial.print((int)m[1]);
        Serial.print(" mz = "); Serial.print((int)m[2]); Serial.println(" mG");

        Serial.print("q0 = "); Serial.print(q[0]);
        Serial.print(" qx = "); Serial.print(q[1]);
        Serial.print(" qy = "); Serial.print(q[2]);
        Serial.print(" qz = "); Serial.println(q[3]);
    }

    void printRollPitchYaw() const
    {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(yaw, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(roll, 2);
    }

    void printCalibration() const
    {
        Serial.println("< calibration parameters >");
        Serial.println("accel bias [g]: ");
        Serial.print(accelBias[0] * 1000.f); Serial.print(", ");
        Serial.print(accelBias[1] * 1000.f); Serial.print(", ");
        Serial.print(accelBias[2] * 1000.f); Serial.println();
        Serial.println("gyro bias [deg/s]: ");
        Serial.print(gyroBias[0]); Serial.print(", ");
        Serial.print(gyroBias[1]); Serial.print(", ");
        Serial.print(gyroBias[2]); Serial.println();
        Serial.println("mag bias [mG]: ");
        Serial.print(magBias[0]); Serial.print(", ");
        Serial.print(magBias[1]); Serial.print(", ");
        Serial.print(magBias[2]); Serial.println();
        Serial.println("mag scale []: ");
        Serial.print(magScale[0]); Serial.print(", ");
        Serial.print(magScale[1]); Serial.print(", ");
        Serial.print(magScale[2]); Serial.println();
    }

private:

    float getAres() const
    {
        switch (AFSSEL)
        {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case AFS::A2G:  return 2.0 / 32768.0;
            case AFS::A4G:  return 4.0 / 32768.0;
            case AFS::A8G:  return 8.0 / 32768.0;
            case AFS::A16G: return 16.0 / 32768.0;
        }
    }

    float getGres() const
    {
        switch (GFSSEL)
        {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case GFS::G250DPS:  return 250.0 / 32768.0;
            case GFS::G500DPS:  return 500.0 / 32768.0;
            case GFS::G1000DPS: return 1000.0 / 32768.0;
            case GFS::G2000DPS: return 2000.0 / 32768.0;
        }
    }

    float getMres() const
    {
        switch (MFSSEL)
        {
            // Possible magnetometer scales (and their register bit settings) are:
            // 14 bit resolution (0) and 16 bit resolution (1)
            // Proper scale to return milliGauss
            case MFS::M14BITS: return 10. * 4912. / 8190.0;
            case MFS::M16BITS: return 10. * 4912. / 32760.0;
        }
    }


    void updateAccelGyro()
    {
        int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
        readMPU9250Data(MPU9250Data); // INT cleared on any read

        // Now we'll calculate the accleration value into actual g's
        a[0] = (float)MPU9250Data[0] * aRes - accelBias[0];  // get actual g value, this depends on scale being set
        a[1] = (float)MPU9250Data[1] * aRes - accelBias[1];
        a[2] = (float)MPU9250Data[2] * aRes - accelBias[2];

        // Calculate the gyro value into actual degrees per second
        g[0] = (float)MPU9250Data[4] * gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
        g[1] = (float)MPU9250Data[5] * gRes - gyroBias[1];
        g[2] = (float)MPU9250Data[6] * gRes - gyroBias[2];
    }

    void readMPU9250Data(int16_t * destination)
    {
        uint8_t rawData[14];  // x/y/z accel register data stored here
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
        destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
        destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
        destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
        destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
        destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
        destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
    }

    void updateMag()
    {
        int16_t magCount[3] = {0, 0, 0};    // Stores the 16-bit signed magnetometer sensor output
        readMagData(magCount);  // Read the x/y/z adc values
        // getMres();

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        m[0] = (float)(magCount[0] * mRes * magCalibration[0] - magBias[0]) * magScale[0];  // get actual magnetometer value, this depends on scale being set
        m[1] = (float)(magCount[1] * mRes * magCalibration[1] - magBias[1]) * magScale[1];
        m[2] = (float)(magCount[2] * mRes * magCalibration[2] - magBias[2]) * magScale[2];
    }

    void readMagData(int16_t * destination)
    {
        uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
        if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
            readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
            uint8_t c = rawData[6]; // End data read by reading ST2 register
            if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
                destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
                destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
                destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
            }
        }
    }

    void updateRPY()
    {
        a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
        a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
        a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
        a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
        pitch = -asinf(a32);
        roll  = atan2f(a31, a33);
        yaw   = atan2f(a12, a22);
        pitch *= 180.0f / PI;
        roll  *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        yaw   += magnetic_declination;
        if      (yaw >= +180.f) yaw -= 360.f;
        else if (yaw <  -180.f) yaw += 360.f;

        lin_ax = a[0] + a31;
        lin_ay = a[1] + a32;
        lin_az = a[2] - a33;
    }

    int16_t readTempData()
    {
        uint8_t rawData[2];  // x/y/z gyro register data stored here
        readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
        return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
    }

    void initAK8963(float * destination)
    {
        // First extract the factory calibration for each magnetometer axis
        uint8_t rawData[3];  // x/y/z gyro calibration data stored here
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
        delay(10);
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
        delay(10);
        readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
        destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
        destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
        destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
        delay(10);
        // Configure the magnetometer for continuous read and highest resolution
        // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
        // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
        writeByte(AK8963_ADDRESS, AK8963_CNTL, (uint8_t)MFSSEL << 4 | Mmode); // Set magnetometer data resolution and sample ODR
        delay(10);

        Serial.println("Calibration values: ");
        Serial.print("X-Axis sensitivity adjustment value "); Serial.println(destination[0], 2);
        Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(destination[1], 2);
        Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(destination[2], 2);
        Serial.print("X-Axis sensitivity offset value "); Serial.println(magBias[0], 2);
        Serial.print("Y-Axis sensitivity offset value "); Serial.println(magBias[1], 2);
        Serial.print("Z-Axis sensitivity offset value "); Serial.println(magBias[2], 2);
    }

    void magcalMPU9250(float * dest1, float * dest2)
    {
        uint16_t ii = 0, sample_count = 0;
        int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
        int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

        Serial.println("Mag Calibration: Wave device in a figure eight until done!");
        delay(4000);

        // shoot for ~fifteen seconds of mag data
        if      (Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
        else if (Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms

        for(ii = 0; ii < sample_count; ii++)
        {
            readMagData(mag_temp);  // Read the mag data
            for (int jj = 0; jj < 3; jj++)
            {
                if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
                if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
            }
            if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
            if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
        }

        Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
        Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
        Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

        // Get hard iron correction
        mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
        mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
        mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

        dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
        dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
        dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

        // Get soft iron correction estimate
        mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
        mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
        mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

        float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
        avg_rad /= 3.0;

        dest2[0] = avg_rad/((float)mag_scale[0]);
        dest2[1] = avg_rad/((float)mag_scale[1]);
        dest2[2] = avg_rad/((float)mag_scale[2]);

        Serial.println("Mag Calibration done!");

        Serial.println("AK8963 mag biases (mG)");
        Serial.print(magBias[0]); Serial.print(", ");
        Serial.print(magBias[1]); Serial.print(", ");
        Serial.print(magBias[2]); Serial.println();
        Serial.println("AK8963 mag scale (mG)");
        Serial.print(magScale[0]); Serial.print(", ");
        Serial.print(magScale[1]); Serial.print(", ");
        Serial.print(magScale[2]); Serial.println();
    }

    void initMPU9250()
    {
        // wake up device
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
        delay(100); // Wait for all registers to reset

        // get stable time source
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
        delay(200);

        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x03);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                            // determined inset in CONFIG above

        // Set gyroscope full scale range
        // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
        // c = c & ~0xE0; // Clear self-test bits [7:5]
        c = c & ~0x03; // Clear Fchoice bits [1:0]
        c = c & ~0x18; // Clear GFS bits [4:3]
        c = c | (uint8_t)GFSSEL << 3; // Set full scale range for the gyro
        // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

        // Set accelerometer full-scale range configuration
        c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
        // c = c & ~0xE0; // Clear self-test bits [7:5]
        c = c & ~0x18;  // Clear AFS bits [4:3]
        c = c | (uint8_t)AFSSEL << 3; // Set full scale range for the accelerometer
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
        c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
        c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
        writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
        delay(100);
    }


    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    void calibrateMPU9250(float * dest1, float * dest2)
    {
        uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
        uint16_t ii, packet_count, fifo_count;
        int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

        // reset device
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
        delay(100);

        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
        // else use the internal oscillator, bits 2:0 = 001
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
        writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
        delay(200);

        // Configure device for bias calculation
        writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
        writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
        delay(15);

        // Configure MPU6050 gyro and accelerometer for bias calculation
        writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

        uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
        uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

        // Configure FIFO to capture accelerometer and gyro data for bias calculation
        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
        delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

        // At end of sample accumulation, turn off FIFO sensor read
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
        readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
        fifo_count = ((uint16_t)data[0] << 8) | data[1];
        packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

        for (ii = 0; ii < packet_count; ii++)
        {
            int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
            readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
            accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
            accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
            accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
            gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
            gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
            gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

            accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            accel_bias[1] += (int32_t) accel_temp[1];
            accel_bias[2] += (int32_t) accel_temp[2];
            gyro_bias[0]  += (int32_t) gyro_temp[0];
            gyro_bias[1]  += (int32_t) gyro_temp[1];
            gyro_bias[2]  += (int32_t) gyro_temp[2];
        }
        accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
        accel_bias[1] /= (int32_t) packet_count;
        accel_bias[2] /= (int32_t) packet_count;
        gyro_bias[0]  /= (int32_t) packet_count;
        gyro_bias[1]  /= (int32_t) packet_count;
        gyro_bias[2]  /= (int32_t) packet_count;

        if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
        else {accel_bias[2] += (int32_t) accelsensitivity;}

        // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
        data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
        data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
        data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
        data[3] = (-gyro_bias[1] / 4)       & 0xFF;
        data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
        data[5] = (-gyro_bias[2] / 4)       & 0xFF;

        // Push gyro biases to hardware registers
        writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
        writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
        writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
        writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
        writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
        writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

        // Output scaled gyro biases for display in the main program
        dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
        dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
        dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

        // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
        // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
        // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
        // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
        // the accelerometer biases calculated above must be divided by 8.

        // int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
        // readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
        // accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
        // readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
        // accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
        // readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
        // accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

        // uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
        // uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

        // for(ii = 0; ii < 3; ii++) {
        //     if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
        // }

        // // Construct total accelerometer bias, including calculated average accelerometer bias from above
        // accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
        // accel_bias_reg[1] -= (accel_bias[1] / 8);
        // accel_bias_reg[2] -= (accel_bias[2] / 8);

        // data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
        // data[1] = (accel_bias_reg[0])      & 0xFF;
        // data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
        // data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
        // data[3] = (accel_bias_reg[1])      & 0xFF;
        // data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
        // data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
        // data[5] = (accel_bias_reg[2])      & 0xFF;
        // data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

        // Apparently this is not working for the acceleration biases in the MPU-9250
        // Are we handling the temperature correction bit properly?
        // Push accelerometer biases to hardware registers
        // writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
        // writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
        // writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
        // writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
        // writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
        // writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

        // Output scaled accelerometer biases for display in the main program
        dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
        dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
        dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;

        Serial.println("MPU9250 bias");
        Serial.println(" x   y   z  ");
        Serial.print((int)(1000 * accelBias[0])); Serial.print(" ");
        Serial.print((int)(1000 * accelBias[1])); Serial.print(" ");
        Serial.print((int)(1000 * accelBias[2])); Serial.print(" ");
        Serial.println("mg");
        Serial.print(gyroBias[0], 1); Serial.print(" ");
        Serial.print(gyroBias[1], 1); Serial.print(" ");
        Serial.print(gyroBias[2], 1); Serial.print(" ");
        Serial.println("o/s");

        delay(100);

        initMPU9250();

        delay(1000);
    }

    // Accelerometer and gyroscope self test; check calibration wrt factory settings
    void SelfTest() // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
    {
        uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
        uint8_t selfTest[6];
        int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
        float factoryTrim[6];
        uint8_t FS = 0;

        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
        writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3);  // Set full scale range for the gyro to 250 dps
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

        for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

            readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
            aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
            aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
            aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

            readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
            gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
            gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
            gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
        }

        for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
            aAvg[ii] /= 200;
            gAvg[ii] /= 200;
        }

        // Configure the accelerometer for self-test
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
        delay(25);  // Delay a while to let the device stabilize

        for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

            readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
            aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
            aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
            aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

            readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
            gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
            gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
            gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
        }

        for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
            aSTAvg[ii] /= 200;
            gSTAvg[ii] /= 200;
        }

        // Configure the gyro and accelerometer for normal operation
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
        delay(25);  // Delay a while to let the device stabilize

        // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
        selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
        selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
        selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
        selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
        selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
        selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

        // Retrieve factory self-test value from self-test code reads
        factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
        factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
        factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
        factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
        factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
        factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

        // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
        // To get percent, must multiply by 100
        for (int i = 0; i < 3; i++)
        {
            SelfTestResult[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;   // Report percent differences
            SelfTestResult[i+3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i+3] - 100.; // Report percent differences
        }

        Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTestResult[0], 1); Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTestResult[1], 1); Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTestResult[2], 1); Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTestResult[3], 1); Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTestResult[4], 1); Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTestResult[5], 1); Serial.println("% of factory value");
        delay(5000);
    }



    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
    {
        wire->beginTransmission(address);  // Initialize the Tx buffer
        wire->write(subAddress);           // Put slave register address in Tx buffer
        wire->write(data);                 // Put data in Tx buffer
        i2c_err_ = wire->endTransmission();           // Send the Tx buffer
        if (i2c_err_) pirntI2CError();
    }

    uint8_t readByte(uint8_t address, uint8_t subAddress)
    {
        uint8_t data = 0; // `data` will store the register data
        wire->beginTransmission(address);         // Initialize the Tx buffer
        wire->write(subAddress);	                 // Put slave register address in Tx buffer
        i2c_err_ = wire->endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
        if (i2c_err_) pirntI2CError();
        wire->requestFrom(address, (size_t)1);  // Read one byte from slave register address
        if (wire->available()) data = wire->read();                      // Fill Rx buffer with result
        return data;                             // Return data read from slave register
    }

    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
    {
        wire->beginTransmission(address);   // Initialize the Tx buffer
        wire->write(subAddress);            // Put slave register address in Tx buffer
        i2c_err_ = wire->endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
        if (i2c_err_) pirntI2CError();
        uint8_t i = 0;
        wire->requestFrom(address, count);  // Read bytes from slave register address
        while (wire->available())
        {
            dest[i++] = wire->read();
        } // Put read results in the Rx buffer
    }

    void pirntI2CError()
    {
        if (i2c_err_ == 7) return; // to avoid stickbreaker-i2c branch's error code
        Serial.print("I2C ERROR CODE : ");
        Serial.println(i2c_err_);
    }


    bool b_ahrs {true};

    WireType* wire;
    uint8_t i2c_err_;


};

#ifdef TEENSYDUINO
using MPU9250 = MPU9250_<i2c_t3>;
#else
using MPU9250 = MPU9250_<TwoWire>;
#endif

#endif // MPU9250_H
