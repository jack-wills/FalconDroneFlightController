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
    static void startTaskImpl(void* _this);
    void task();

    uint16_t throttle = 0;

    IMU imu;

    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_HandleTypeDef PWM1Handle;
    TIM_HandleTypeDef PWM2Handle;

    Logger LOG = Logger("MotorController");

    TaskHandle_t taskHandle;
};

