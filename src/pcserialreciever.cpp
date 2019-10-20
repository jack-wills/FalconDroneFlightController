#include "pcserialreciever.h"

extern QueueHandle_t serialRXQueue;

PCSerialReciever::PCSerialReciever(MotorController& motorController) {
    serialRXQueue = xQueueCreate(10, 64*sizeof(char));
    this->motorController = &motorController;
    xTaskCreate(this->startTaskImpl, "SerialRX", 1024, this, configMAX_PRIORITIES-1, &taskHandle);
}

PCSerialReciever::~PCSerialReciever() {

}

PCSerialReciever::PCSerialReciever(const PCSerialReciever& other): motorController(other.motorController) {

}

void PCSerialReciever::startTaskImpl(void* _this) {
    ((PCSerialReciever*)_this)->task();
}

void PCSerialReciever::task() {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50;

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        while (uxQueueMessagesWaiting(serialRXQueue) > 0) {
            char rxMessage[64];
            if( xQueueReceive(serialRXQueue, (void*)rxMessage, 0)) {
                std::string temp = rxMessage;
                if (temp.find("\n") != std::string::npos) {
                    std::string s = temp.substr(0, temp.find("\n"));
                    if (s.find(" ") != std::string::npos) {
                        std::string command = s.substr(0, s.find(" "));
                        std::string value = s.substr(s.find(" "), s.length());
                        if (!command.compare("C")) {
                            motorController->setThrottle(std::stoul(value,nullptr,0));
                        } else if (!command.compare("TEST")) {      
                            LOG.info() << s << LOG.flush;
                        } else {
                            LOG.error() << "Invalid command recieved from PC: " << command << LOG.flush;
                        }
                    } else {
                        LOG.error() << "Invalid data recieved from PC: " << s << LOG.flush;
                    }
                }
            }
            
        }
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}