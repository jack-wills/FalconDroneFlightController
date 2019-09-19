#include "logmanager.h"

QueueHandle_t loggerQueue;

LogManager::LogManager() : pcSerial(PCSerial(115200)) {
    loggerQueue = xQueueCreate(10, 100*sizeof(char));

    xTaskCreate(this->startTaskImpl, "LOG", 1024, this, tskIDLE_PRIORITY, &taskHandle);
} 

void LogManager::startTaskImpl(void* _this){
    ((LogManager*)_this)->task();
}

void LogManager::task() {
    while (1) {
        while (uxQueueMessagesWaiting(loggerQueue) > 0) {
            char logMessage[100];
            if( xQueueReceive( loggerQueue, ( void * ) logMessage, 0 ) )
            {
                pcSerial.println(logMessage);
            }
        }
        vTaskSuspend(taskHandle);
    }
}

void LogManager::flush() {
    vTaskResume(taskHandle);
}