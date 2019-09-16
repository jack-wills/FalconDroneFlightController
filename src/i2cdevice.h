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
	I2CDevice(const I2CDevice& i2cDevice);
	bool write(uint8_t addr, uint8_t* data, int dataSize);
	bool write(uint8_t addr, uint8_t data);
	bool writeReg(uint8_t addr, uint8_t reg, uint8_t* data, int dataSize);
	bool writeReg(uint8_t addr, uint8_t reg, int dataSize, uint8_t* data);
	bool writeWord(uint8_t addr, uint8_t reg, uint16_t data);
	bool writeByte(uint8_t addr, uint8_t reg, uint8_t data);
	bool writeBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data);
	bool writeBits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);
	bool read(uint8_t addr, uint8_t* rXData, int dataSize);
	bool readReg(uint8_t addr, uint8_t reg, uint8_t* rXData, int dataSize);
	bool readReg(uint8_t addr, uint8_t reg, int dataSize, uint8_t* rXData);
	bool readByte(uint8_t addr, uint8_t reg, uint8_t* rXData);
	bool readBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t *data);
	bool readBits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t *data);
	void scan();
private:
	void I2C_ClearBusyFlagErratum();
    GPIO_InitTypeDef GPIO_InitStruct; 
	I2C_HandleTypeDef I2CHandle;
    Logger LOG = Logger("I2CDevice");
};

