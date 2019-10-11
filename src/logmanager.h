#pragma once
#include "FreeRTOS.h"
#include "queue.h"

#include "pcserial.h"

extern QueueHandle_t loggerQueue;

class LogManager
{
public:
	LogManager();
    void flush();
private:
    static void startTaskImpl(void* _this);

    void task();

    PCSerial pcSerial; 

    TaskHandle_t taskHandle;
};