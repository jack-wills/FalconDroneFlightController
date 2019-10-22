#include "pcserialreciever.h"

#include <sstream>
#include <vector>
#include <iterator>

#define DMA_RX_BUFFER_SIZE          256

extern QueueHandle_t serialRXQueue;

PCSerialReciever::PCSerialReciever(MotorController& motorController, IMU &imu) {
    serialRXQueue = xQueueCreate(10, DMA_RX_BUFFER_SIZE*sizeof(char));
    this->imu = &imu; 
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
            char rxMessage[DMA_RX_BUFFER_SIZE];
            if( xQueueReceive(serialRXQueue, (void*)rxMessage, 0)) {
                std::string temp = rxMessage;
                if (temp.find("\n") != std::string::npos) {
                    std::string s = temp.substr(0, temp.find("\n"));
                    if (s.find(" ") != std::string::npos) {
                        std::string command = s.substr(0, s.find(" "));
                        std::string value = s.substr(s.find(" "), s.length());
                        if (!command.compare("C")) {
                            motorController->setThrottle(std::stoul(value,nullptr,0));
                        } else if (!command.compare("MAGCALINIT")) {      
                            imu->calibrateMagnetometer();
                        } else if (!command.compare("GYROCALINIT")) {      
                            imu->calibrateGyro();
                        } else if (!command.compare("MAGCALVAL")) {
                            std::istringstream iss(value);
                            std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                                            std::istream_iterator<std::string>{}};
                            if (tokens.size() != 12) {
                                LOG.error() << "Invalid amount of magnetometer calibration values" << LOG.flush;
                                continue;
                            }
                            float values[12];
                            for (unsigned i=0; i < 12; i++) {
                                values[i] = std::stof(tokens[i], nullptr);
                            }
                            imu->setMagnetometerCalibration(values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7],values[8],values[9],values[10],values[11]);
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