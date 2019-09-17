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
	void update9Dof();
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
    float magBias[3] = {-7.53898708, 143.70979295, -363.5544908};
    
    float magCalibration[3][3] = {{2.57983916e-03, -1.08889381e-04, 1.12930132e-05},
                                {-1.08889381e-04, 2.36526444e-03, -5.24037004e-05},
                                { 1.12930132e-05, -5.24037004e-05, 2.58332164e-03}};
    uint8_t mMode;

    int16_t aXRaw,aYRaw,aZRaw,gXRaw,gYRaw,gZRaw,mXRaw,mYRaw,mZRaw;

    int64_t gXOffset = 0, gYOffset = 0, gZOffset = 0;
    float aXOffset = 0.82f, aYOffset = 0.05f, aZOffset = -0.21;

    float aX,aY,aZ,gX,gY,gZ,mX,mY,mZ;

    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float invSampleFreq = 1.0f/200.0f;
    float beta = 0.1f;

    float degToRad = PI/180.0f;
    float radToDeg = 180.0f/PI;

    Logger LOG = Logger("IMU");
};

