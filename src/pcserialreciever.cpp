#include "pcserialreciever.h"

#define UART_BUFFER_SIZE            256
extern uint8_t UART_Buffer[UART_BUFFER_SIZE];
extern bool dmaDataAvailable;

PCSerialReciever::PCSerialReciever() {
    xTaskCreate(this->startTaskImpl, "SerialRX", 1024, this, configMAX_PRIORITIES-1, &taskHandle);
}

PCSerialReciever::~PCSerialReciever() {

}

PCSerialReciever::PCSerialReciever(const PCSerialReciever& other) {
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
            LOG.info() << s << LOG.flush;
            start = end + 1;
        }
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}