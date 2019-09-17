#pragma once
#include "mpu9250.h"
#include "logger.h"

#define PI 3.141592f

class IMU
{
public:
	IMU(uint8_t sensorAddress);
    IMU(const IMU& other);
	~IMU();
	void update6Dof();
	void update();
    void getAngles(float *pitch, float *roll, float *yaw);
    void printAngles();
    void printQuaternions();
    void calibrateMagnetometer();
private:
    void calibrateGyro();
    void loadValues();
    float invSqrt(float x);
    void mahonyInit();

    I2CDevice i2cDevice;
    MPU9250 mpu;
    float aRes, gRes, mRes;
    float magCalibration[3] = {0, 0, 0}, magScale[3] = {0.82, 1.03, 1.21}, magBias[3] = {133.44, 50.97, -19.49};
    uint8_t mMode;

    int16_t aXRaw,aYRaw,aZRaw,gXRaw,gYRaw,gZRaw,mXRaw,mYRaw,mZRaw;

    int64_t gXOffset = 0, gYOffset = 0, gZOffset = 0;
    float aXOffset = 0.82f, aYOffset = 0.05f, aZOffset = -0.21;

    float aX,aY,aZ,gX,gY,gZ,mX,mY,mZ;

    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float invSampleFreq = 1.0f/200.0f;
    float beta = 0.01f;

    float degToRad = PI/180.0f;
    float radToDeg = 180.0f/PI;

    Logger LOG = Logger("IMU");
};

