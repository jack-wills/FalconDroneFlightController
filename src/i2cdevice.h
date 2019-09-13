#pragma once
#include <stm32f401xe.h>
#include <stm32f4xx_hal.h>
#include <string.h>

#include <string>

#include "logger.h"

class I2CDevice
{
public:
	I2CDevice();
	~I2CDevice();
	void write(char addr, uint8_t* data, int dataSize);
	void write(char addr, uint8_t data);
	void read(char addr, uint8_t* rXData, int dataSize);
	void writeReg(char addr, uint8_t reg, uint8_t data);
	void readReg(char addr, uint8_t* rXData, int dataSize, uint8_t reg);
	void scan();
private:
	void I2C_ClearBusyFlagErratum();
    GPIO_InitTypeDef GPIO_InitStruct; 
	I2C_HandleTypeDef I2CHandle;
    Logger LOG = Logger("I2CDevice");
};

