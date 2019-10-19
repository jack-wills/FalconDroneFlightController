#pragma once
#include <stm32f401xe.h>
#include <stm32f4xx_hal.h>
#include <string.h>

#include <string>

class PCSerial
{
public:
	PCSerial(int baudRate);
	~PCSerial();
	void print(std::string string);
	void println(std::string string);
private:
    GPIO_InitTypeDef GPIO_InitStruct; 
};

