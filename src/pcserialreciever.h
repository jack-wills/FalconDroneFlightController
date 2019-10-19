#pragma once
#include <stm32f401xe.h>
#include <stm32f4xx_hal.h>
#include <string.h>

#include <string>

#include "FreeRTOS.h"
#include "task.h"

#include "logger.h"

class PCSerialReciever
{
public:
	PCSerialReciever();
	~PCSerialReciever();
	PCSerialReciever(const PCSerialReciever& other);
private:
    static void startTaskImpl(void* _this);
    void task();

    Logger LOG = Logger("SerialRX");

	TaskHandle_t taskHandle;
};

