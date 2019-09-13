#pragma once
#include <sstream>
#include <iomanip>

#include "pcserial.h"

class Logger
{
public:
	Logger(std::string className) : pcSerial(PCSerial(115200)) {
        std::string nameTemp = "[" + className + "] ";
        for (int i = strlen(nameTemp.c_str()); i < 16; i++) {
            nameTemp = nameTemp + " ";
        }
        this->className = nameTemp;
    }
    void info(std::string msg) {
        print(msg, "INFO ");
    }
    void warn(std::string msg) {
        print(msg, "WARN ");
    }
    void error(std::string msg) {
        print(msg, "ERROR");
    }
private:
    PCSerial pcSerial; 
    std::string className;
	void print(std::string msg, std::string level) {
        std::stringstream ss;
        int time = HAL_GetTick();
        int milli = time%1000;
        int sec = (time%60000)/1000;
        int min = time/60000;
        ss << "(" << std::setw(3) << std::setfill('0') << min 
           << ":" << std::setw(2) << std::setfill('0') << sec 
           << "." << std::setw(3) << std::setfill('0') << milli 
           << ") " << className << level << ": " << msg;
        pcSerial.println(ss.str());
    }
};

