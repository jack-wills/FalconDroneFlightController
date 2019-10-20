#pragma once
#include <stm32f401xe.h>
#include <stm32f4xx_hal.h>

#include "logger.h"
#include "imu.h"

class MotorController
{
public:
	MotorController(IMU &imu);
	~MotorController();
    MotorController(const MotorController& other);
    void setThrottle(uint16_t throttle);
    
private:
    void calculatePID(float pitch, float roll, float yaw);
    static void startTaskImpl(void* _this);
    void task();

    uint16_t throttle = 0;
    uint16_t pitchInput = 0, rollInput = 0, yawInput = 0;
    
    float pidPitchError, pidPitchIntegral, pidPitchValue, pidPitchOld;

    float pidRollError, pidRollIntegral, pidRollValue, pidRollOld;

    float pidYawError, pidYawIntegral, pidYawValue, pidYawOld;

    float pidGainPP = 1;
    float pidGainPR = 1;
    float pidGainPY = 1;

    float pidGainIP = 0;
    float pidGainIR = 0;
    float pidGainIY = 0;

    float pidGainDP = 0;
    float pidGainDR = 0;
    float pidGainDY = 0;

    IMU *imu;

    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_HandleTypeDef PWM1Handle;
    TIM_HandleTypeDef PWM2Handle;

    Logger LOG = Logger("MotorController");

    TaskHandle_t taskHandle;
};

