#include "imu.h"

IMU::IMU(uint8_t sensorAddress): i2cDevice(I2CDevice()), mpu(MPU9250(sensorAddress, i2cDevice)) {
    mpu.initialize();
    uint8_t accelFullScale = MPU9250_ACCEL_FS_8;
    mpu.setFullScaleAccelRange(accelFullScale);
    uint8_t gyroFullScale = MPU9250_GYRO_FS_500;
    mpu.setFullScaleGyroRange(gyroFullScale);

    aRes = ((float)(accelFullScale+1))/16384.0f;
    gRes = (((float)(gyroFullScale+1))/131.0f)*degToRad;

    calibrateGyro();
}

IMU::IMU(const IMU& other): i2cDevice(other.i2cDevice), mpu(other.mpu) {

}

IMU::~IMU() {

}

void IMU::calibrateGyro() {
    LOG.info() << "Starting calibrations..." << LOG.flush;
    for(int i = 0; i < 500; i++){
        mpu.getRotation(&gXRaw,&gYRaw,&gZRaw);
        gXOffset += gXRaw;
        gYOffset += gYRaw;
        gZOffset += gZRaw;
        HAL_Delay(3);
    }
    LOG.info() << "Calibrations Finished" << LOG.flush;
    gXOffset /= 500;
    gYOffset /= 500;
    gZOffset /= 500;
}

void IMU::loadValues() {
    mpu.getMotion9(&aXRaw,&aYRaw,&aZRaw,&gXRaw,&gYRaw,&gZRaw,&mXRaw,&mYRaw,&mZRaw);

    aX = (float)(aXRaw*aRes);
    aY = (float)(aYRaw*aRes); 
    aZ = (float)(aZRaw*aRes);

    gX = (float)(gXRaw-gXOffset)*gRes;
    gY = (float)(gYRaw-gYOffset)*gRes;  
    gZ = (float)(gZRaw-gZOffset)*gRes;

}

void IMU::getAngles(float *pitch, float *roll, float *yaw) {
	*roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*radToDeg;
	*pitch = asinf(-2.0f * (q1*q3 - q0*q2))*radToDeg;
	*yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*radToDeg;
}

void IMU::printAngles() {
    float roll,pitch,yaw;
    getAngles(&pitch, &roll, &yaw);

    LOG.info() << "Angle: " << roll << " ,\t " << pitch << " ,\t " << yaw << LOG.flush;
}

void IMU::printQuaternions() {
    LOG.info() << "Quaternion: " << q0 << " ,\t " << q1 << " ,\t " << q2 << " ,\t " << q3 << LOG.flush;
}

void IMU::update6Dof() {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gX - q2 * gY - q3 * gZ);
	qDot2 = 0.5f * (q0 * gX + q2 * gZ - q3 * gY);
	qDot3 = 0.5f * (q0 * gY - q1 * gZ + q3 * gX);
	qDot4 = 0.5f * (q0 * gZ + q1 * gY - q2 * gX);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((aX == 0.0f) && (aY == 0.0f) && (aZ == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(aX * aX + aY * aY + aZ * aZ);
		aX *= recipNorm;
		aY *= recipNorm;
		aZ *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * aX + _4q0 * q1q1 - _2q1 * aY;
		s1 = _4q1 * q3q3 - _2q3 * aX + 4.0f * q0q0 * q1 - _2q0 * aY - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * aZ;
		s2 = 4.0f * q0q0 * q2 + _2q0 * aX + _4q2 * q3q3 - _2q3 * aY - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * aZ;
		s3 = 4.0f * q1q1 * q3 - _2q1 * aX + 4.0f * q2q2 * q3 - _2q2 * aY;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void IMU::update() {
    loadValues();
	for (int i = 0; i < 10; i++) {
        update6Dof();
    }
}

float IMU::invSqrt(float x) {
	union {
		float f;
		int32_t i;
	} y;
	float halfx = 0.5f * x;

	y.f = x;
	y.i = 0x5f375a86 - (y.i >> 1);
	y.f = y.f * (1.5f - (halfx * y.f * y.f));
	y.f = y.f * (1.5f - (halfx * y.f * y.f));
	y.f = y.f * (1.5f - (halfx * y.f * y.f));
	return y.f;
}