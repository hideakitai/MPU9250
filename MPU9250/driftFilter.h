#pragma once
#ifndef DRIFTFILTER_H
#define DRIFTFILTER_H

#ifndef DRIFT_FILTER  
	#define DRIFT_FILTER 1
#endif
#ifndef MAGNETTIC_DECLINATION 
	#define MAGNETTIC_DECLINATION -7.51  // Japan, 24th June
#endif

class DriftFilter {
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
	float magnetic_declination = MAGNETTIC_DECLINATION;

// NO FILTER JUST INTEGRATE GYRO axes
#if DRIFT_FILTER == 0
public:
    void bind() {}
 	void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT ) {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // variable for readability
        q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz)*deltaT;
        q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy)*deltaT;
        q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx)*deltaT;
        q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx)*deltaT;
		float recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= recipNorm;
        q[1] *= recipNorm;
        q[2] *= recipNorm;
        q[3] *= recipNorm;
	}
	
	void getRPY(float *yaw,float *pitch, float *roll)
	{
		*yaw = atan2(2.0f * q[1] * q[2] - 2.0f * q[0] * q[3], 
					2.0f *  q[0] * q[0] + 2.0f * q[1] * q[1] - 1)*180.0f / PI;
		*pitch = -asin(2 * (q[0] * q[2] + q[3] * q[1]))*180.0f / PI;
		*roll = atan2(2.0f * q[2] * q[3] - 2.0f * q[0] * q[1], 
					2.0f *  q[0] * q[0] + 2.0f * q[3] * q[3] - 1)*180.0f / PI;
	}
#endif
// Madgwick 1
#if DRIFT_FILTER == 1  // https://github.com/arduino-libraries/MadgwickAHRS/blob/master/src/MadgwickAHRS.cpp
    	
    //float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 40 deg/s)
    //float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
	float beta = 0.1f;
public:
    void bind() {}
    // QuaternionUpdate
	// a. MPU accelleration, g. gyro rad/s, m. magnetometer, dt in seconds
    void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT ) {

        double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // variable for readability
        double recipNorm;
        double s0, s1, s2, s3;
        double qDot1, qDot2, qDot3, qDot4;
        double hx, hy;
        double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Normalise accelerometer measurement
        double a_norm = ax * ax + ay * ay + az * az;
        if (a_norm == 0.) return;  // handle NaN
        recipNorm = 1.0 / sqrt(a_norm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        double m_norm = mx * mx + my * my + mz * mz;
        if (m_norm == 0.) return;  // handle NaN
        recipNorm = 1.0 / sqrt(m_norm);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        
		recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * deltaT;
        q1 += qDot2 * deltaT;
        q2 += qDot3 * deltaT;
        q3 += qDot4 * deltaT;

        // Normalise quaternion
        recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
    }
	
	void getRPY(float *yaw,float *pitch, float *roll)
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
        // @hideakitai: replaced by https://github.com/openframeworks/openFrameworks/blob/fe591d17e95218569cc2426d1d8f4f646f75fa00/libs/openFrameworks/math/ofQuaternion.cpp#L276

#if 0
		float test = q[1] * q[2] + q[3] * q[0];
		
        if (test > 0.499) {  // singularity at north pole
            *yaw = 2 * atan2(q[1], q[0]);
            *pitch = PI / 2;
            *roll = 0;
        } else if (test < -0.499) {  // singularity at south pole
            *yaw = -2 * atan2(q[1], q[0]);
            *pitch = -PI / 2;
            *roll = 0;
        } else {
            float sqx = q[1] * q[1];
            float sqy = q[2] * q[2];
            float sqz = q[3] * q[3];
            *yaw = atan2(2.0f * q[2] * q[0] - 2.0f * q[1] * q[3], 1.0f - 2.0f * sqy - 2.0f * sqz);
            *pitch = asin(2 * test);
            *roll = atan2(2.0f * q[1] * q[0] - 2.0f * q[2] * q[3], 1.0f - 2.0f * sqx - 2.0f * sqz);
        }
        *pitch *= 180.0f / PI;
        *roll *= 180.0f / PI;
        *yaw *= 180.0f / PI;
        *yaw += magnetic_declination;
        if (*yaw >= +180.f)
            *yaw -= 360.f;
        else if (*yaw < -180.f)
            *yaw += 360.f;		

#else 

		*roll = atan2f( q[0] * q[1] + q[2] * q[3], 0.5f - q[0] * q[0] - q[2] * q[2])*180.0f / PI;
		*pitch = asinf(-2.0f * (q[3] * q[1] - q[0] * q[2]))*180.0f / PI;
		*yaw = atan2f(q[1] * q[2] - q[0] * q[3], 0.5f - q[2] * q[2] - q[3] * q[3])*180.0f / PI;
 
#endif				
	}
#endif

// Mahony gyro accel
#if DRIFT_FILTER == 2 
	// Mahony scheme uses proportional and integral filtering on
	// the error between estimated reference vector (gravity) and measured one.
	// Madgwick's implementation of Mayhony's AHRS algorithm.
	// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
	// Free parameters in the Mahony filter and fusion scheme,
	// Kp for proportional feedback, Ki for integral
	float Kp = 30.0;
	float Ki = 0.0;
	// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
	// with MPU-6050, some instability observed at Kp=100 Now set to 30.
public:
	void bind() {}
    void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT ) {
		float recipNorm;
		float vx, vy, vz;
		float ex, ey, ez;  //error terms
		float qa, qb, qc;
		static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
		float tmp;

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		tmp = ax * ax + ay * ay + az * az;
		if (tmp > 0.0)
		{

			// Normalise accelerometer (assumed to measure the direction of gravity in body frame)
			recipNorm = 1.0 / sqrt(tmp);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Estimated direction of gravity in the body frame (factor of two divided out)
			vx = q[1] * q[3] - q[0] * q[2];
			vy = q[0] * q[1] + q[2] * q[3];
			vz = q[0] * q[0] - 0.5f + q[3] * q[3];

			// Error is cross product between estimated and measured direction of gravity in body frame
			// (half the actual magnitude)
			ex = (ay * vz - az * vy);
			ey = (az * vx - ax * vz);
			ez = (ax * vy - ay * vx);

			// Compute and apply to gyro term the integral feedback, if enabled
			if (Ki > 0.0f) {
			  ix += Ki * ex * deltaT;  // integral error scaled by Ki
			  iy += Ki * ey * deltaT;
			  iz += Ki * ez * deltaT;
			  gx += ix;  // apply integral feedback
			  gy += iy;
			  gz += iz;
			}

			// Apply proportional feedback to gyro term
			gx += Kp * ex;
			gy += Kp * ey;
			gz += Kp * ez;
		}

		// Integrate rate of change of quaternion, q cross gyro term
		deltaT = 0.5 * deltaT;
		gx *= deltaT;   // pre-multiply common factors
		gy *= deltaT;
		gz *= deltaT;
		qa = q[0];
		qb = q[1];
		qc = q[2];
		q[0] += (-qb * gx - qc * gy - q[3] * gz);
		q[1] += (qa * gx + qc * gz - q[3] * gy);
		q[2] += (qa * gy - qb * gz + q[3] * gx);
		q[3] += (qa * gz + qb * gy - qc * gx);

		// renormalise quaternion
		recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		q[0] = q[0] * recipNorm;
		q[1] = q[1] * recipNorm;
		q[2] = q[2] * recipNorm;
		q[3] = q[3] * recipNorm;
			
	}
	
	void getRPY(float *yaw,float *pitch, float *roll)
	{
		float test = q[0] * q[2] + q[3] * q[1];
		
        float sqw = q[0] * q[0];
		float sqx = q[1] * q[1];
		float sqy = q[2] * q[2];
		float sqz = q[3] * q[3];
		*yaw = atan2(2.0f * q[1] * q[2] - 2.0f * q[0] * q[3], 2.0f * sqw + 2.0f * sqx - 1)*180.0f / PI;
		*pitch = -asin(2 * test)*180.0f / PI;
		*roll = atan2(2.0f * q[2] * q[3] - 2.0f * q[0] * q[1], 2.0f * sqw + 2.0f * sqz - 1)*180.0f / PI;
	}
#endif
public:
	void setMagneticDeclination(const float d) { magnetic_declination = d; }

    float getQuaternionW() const { return q[0]; }
    float getQuaternionX() const { return q[1]; }
    float getQuaternionY() const { return q[2]; }
    float getQuaternionZ() const { return q[3]; }
	float getQuaternion(const uint8_t i) const { return (i < 4) ? q[i] : 0.f; }
    
    void printQuaternionHead() const {
		Serial.print("\tq0\tqx\tqy\tqz");
    }
    void printQuaternion() const {
		Serial.print("\t");
        Serial.print(q[0]);
        Serial.print("\t");
        Serial.print(q[1]);
        Serial.print("\t");
        Serial.print(q[2]);
        Serial.print("\t");
        Serial.print(q[3]);
    }
};

#endif  // DRIFTFILTER_H
