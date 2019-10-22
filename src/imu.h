#pragma once
#include "mpu9250.h"
#include "logger.h"

#include "FreeRTOS.h"
#include "task.h"

#define PI 3.141592f

class IMU
{
public:
	IMU(uint8_t sensorAddress);
    IMU(const IMU& other);
	~IMU();
    void getAngles(float *pitch, float *roll, float *yaw);
    void setMagnetometerCalibration(float magB0, float magB1, float magB2, float magC00, float magC01, float magC02, float magC10, float magC11, float magC12,  float magC20, float magC21, float magC22);
    void calibrateMagnetometer();
    void calibrateGyro();
private:
    static void startTaskImpl(void* _this);
    void task();
	void update6Dof();
	void update9Dof();
	void update();
    void printAngles();
    void printQuaternions();
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

    int16_t gXOffset = 0, gYOffset = 0, gZOffset = 0;
    float aXOffset = 0.82f, aYOffset = 0.05f, aZOffset = -0.21;

    float aX,aY,aZ,gX,gY,gZ,mX,mY,mZ;

    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float samplePeriod = 10.0f;
    float samplePeriodMadgwick = samplePeriod/10000.0f; //Convert to seconds and divide by 10 since we do 10 filter updates per sample
    float beta = 0.1f;

    float degToRad = PI/180.0f;
    float radToDeg = 180.0f/PI;

    Logger LOG = Logger("IMU");

    TaskHandle_t taskHandle;
};

