#include "imu.h"

#include "fccomm.h"

IMU::IMU(uint8_t sensorAddress): i2cDevice(I2CDevice()), mpu(MPU9250(sensorAddress, i2cDevice)) {
    mpu.initialize();
    uint8_t accelFullScale = MPU9250_ACCEL_FS_8;
    mpu.setFullScaleAccelRange(accelFullScale);
    uint8_t gyroFullScale = MPU9250_GYRO_FS_1000;
    mpu.setFullScaleGyroRange(gyroFullScale);
	mMode = AK8963_SAMP_100;
	uint8_t mBitWith = AK8963_16_BIT;
	mpu.initAK8963(mBitWith, mMode);

    aRes = ((float)(accelFullScale+1))/16384.0f;
    gRes = (((float)(gyroFullScale+1))/131.0f)*degToRad;
	if (mBitWith == AK8963_16_BIT) {
		mRes = 10.*4912./32760.0;
	} else {
    	mRes = 10.*4912./8190.; // Proper scale to return milliGauss 14bit
	}

    calibrateGyro();

	beta = 5.0f;
    uint64_t tLast = HAL_GetTick();
    for(int i = 0; i < 40; i++) {
        update();
        while(HAL_GetTick() - tLast < samplePeriod);
        tLast = HAL_GetTick();
    }
	beta = 0.1f;

    xTaskCreate(this->startTaskImpl, "IMU", 1024, this, configMAX_PRIORITIES-1, &taskHandle);
}

IMU::IMU(const IMU& other): i2cDevice(other.i2cDevice), mpu(other.mpu) {

}

IMU::~IMU() {

}

void IMU::startTaskImpl(void* _this){
    ((IMU*)_this)->task();
}

void IMU::task() {

    if (false) {
        vTaskDelay(100);
        calibrateMagnetometer();
        vTaskDelay(100000000);
    }

 	TickType_t xLastWakeTime;
 	const TickType_t xFrequency = samplePeriod;

	xLastWakeTime = xTaskGetTickCount();
	while (1) {
        update();
        printAngles();
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

void IMU::calibrateMagnetometer() {
	for (int i = 0; i < 1000; i++) {
		mpu.getMagnetometerReading(&mXRaw, &mYRaw, &mZRaw);
		FCComm("MAGCAL") << (float)mXRaw*mRes << " " << (float)mYRaw*mRes << " " << (float)mZRaw*mRes;
		if(mMode == AK8963_SAMP_8)   vTaskDelay(125);  // at 8 Hz ODR, new mag data is available every 125 ms
		if(mMode == AK8963_SAMP_100) vTaskDelay(10);   // at 100 Hz ODR, new mag data is available every 10 ms   
	}
}

void IMU::calibrateGyro() {
    LOG.info() << "Starting calibrations..." << LOG.flush;
    for(int i = 0; i < 500; i++){
        mpu.getRotation(&gXRaw,&gYRaw,&gZRaw);
        gXOffset += gXRaw;
        gYOffset += gYRaw;
        gZOffset += gZRaw;
        vTaskDelay(3);
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

	float mXPreCal = (float)mXRaw*mRes - magBias[0];
	float mYPreCal = (float)mYRaw*mRes - magBias[1];
	float mZPreCal = (float)mZRaw*mRes - magBias[2];

	mX = mXPreCal*magCalibration[0][0] + mYPreCal*magCalibration[0][1] + mZPreCal*magCalibration[0][2];
	mY = mXPreCal*magCalibration[1][0] + mYPreCal*magCalibration[1][1] + mZPreCal*magCalibration[1][2];
	mZ = mXPreCal*magCalibration[2][0] + mYPreCal*magCalibration[2][1] + mZPreCal*magCalibration[2][2];
	mZ *=-1; //Invert since MPU magnetometer Z axis is inverted from aZ
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
	q0 += qDot1 * samplePeriodMadgwick;
	q1 += qDot2 * samplePeriodMadgwick;
	q2 += qDot3 * samplePeriodMadgwick;
	q3 += qDot4 * samplePeriodMadgwick;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void IMU::update9Dof() {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mY == 0.0f) && (mX == 0.0f) && (mZ == 0.0f)) {
		update6Dof();
		return;
	}

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

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mY * mY + mX * mX + mZ * mZ);
		mY *= recipNorm;
		mX *= recipNorm;
		mZ *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mY;
		_2q0my = 2.0f * q0 * mX;
		_2q0mz = 2.0f * q0 * mZ;
		_2q1mx = 2.0f * q1 * mY;
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
		hx = mY * q0q0 - _2q0my * q3 + _2q0mz * q2 + mY * q1q1 + _2q1 * mX * q2 + _2q1 * mZ * q3 - mY * q2q2 - mY * q3q3;
		hy = _2q0mx * q3 + mX * q0q0 - _2q0mz * q1 + _2q1mx * q2 - mX * q1q1 + mX * q2q2 + _2q2 * mZ * q3 - mX * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mZ * q0q0 + _2q1mx * q3 - mZ * q1q1 + _2q2 * mX * q3 - mZ * q2q2 + mZ * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - aX) + _2q1 * (2.0f * q0q1 + _2q2q3 - aY) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mY) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mX) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mZ);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - aX) + _2q0 * (2.0f * q0q1 + _2q2q3 - aY) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - aZ) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mY) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mX) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mZ);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - aX) + _2q3 * (2.0f * q0q1 + _2q2q3 - aY) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - aZ) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mY) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mX) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mZ);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - aX) + _2q2 * (2.0f * q0q1 + _2q2q3 - aY) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mY) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mX) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mZ);
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
	q0 += qDot1 * samplePeriodMadgwick;
	q1 += qDot2 * samplePeriodMadgwick;
	q2 += qDot3 * samplePeriodMadgwick;
	q3 += qDot4 * samplePeriodMadgwick;

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
        update9Dof();
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