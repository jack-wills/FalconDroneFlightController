#pragma once
#include <stm32f401xe.h>
#include <stm32f4xx_hal.h>
#include <string.h>

#include <string>

#include "FreeRTOS.h"
#include "task.h"

#include "logger.h"
#include "motorcontroller.h"
#include "imu.h"

class PCSerialReciever
{
public:
	PCSerialReciever(MotorController& motorController, IMU &imu);
	~PCSerialReciever();
	PCSerialReciever(const PCSerialReciever& other);
private:
    static void startTaskImpl(void* _this);
    void task();

	MotorController *motorController;
    IMU *imu;

    Logger LOG = Logger("SerialRX");

	TaskHandle_t taskHandle;
};

