#include "pcserialreciever.h"

#define UART_BUFFER_SIZE            256
extern uint8_t UART_Buffer[UART_BUFFER_SIZE];
extern bool dmaDataAvailable;

PCSerialReciever::PCSerialReciever(MotorController& motorController) {
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
    const TickType_t xFrequency = 500;

    xLastWakeTime = xTaskGetTickCount();
    int start = 0;
    while (1) {
        if (dmaDataAvailable) {
            dmaDataAvailable = false;
            int end = start;
            while (1) {
                if (UART_Buffer[++end] == '\n') {
                    break;
                } else if (end == UART_BUFFER_SIZE-1) {
                    end = 0;
                } else if (end == start) {
                    LOG.error() << "No new line in UART recieve buffer\n" << LOG.flush;
                }
            }
            std::string s;
            if (end > start) {
                s = std::string(UART_Buffer + start, UART_Buffer + end);
            } else {
                s = std::string(UART_Buffer + start, UART_Buffer + UART_BUFFER_SIZE);
                s.append(std::string(UART_Buffer, UART_Buffer + end));
            }
            start = end + 1;
            if (s.find(" ") != std::string::npos) {
                std::string command = s.substr(0, s.find(" "));
                std::string value = s.substr(s.find(" "), s.length());
                if (!command.compare("THROTTLE")) {
                    motorController->setThrottle(std::stoul(value,nullptr,0));
                } else if (!command.compare("TEST")) {
                    LOG.info() << value << LOG.flush;
                } else {
                    LOG.error() << "Invalid command recieved from PC" << LOG.flush;
                }
            } else {
                LOG.error() << "Invalid command recieved from PC" << LOG.flush;
            }
            
        }
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}