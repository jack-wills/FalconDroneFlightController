#include "i2cdevice.h"

I2CDevice::I2CDevice() {
    __I2C1_CLK_ENABLE(); 

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    I2CHandle.Instance = I2C1;
    I2CHandle.Init.ClockSpeed = 400000;
    I2CHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2CHandle.Init.OwnAddress1 = 0;
    I2CHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2CHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2CHandle.Init.OwnAddress2 = 0;
    I2CHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2CHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    I2C_ClearBusyFlagErratum();
}

I2CDevice::~I2CDevice() {

}

I2CDevice::I2CDevice(const I2CDevice& i2cDevice) {
    GPIO_InitStruct = i2cDevice.GPIO_InitStruct;
    I2CHandle = i2cDevice.I2CHandle;
}

void I2CDevice::I2C_ClearBusyFlagErratum() {
    uint16_t            sdaPin = GPIO_PIN_9;
    GPIO_TypeDef*       sdaPort = GPIOB;
    uint16_t            sclPin = GPIO_PIN_8;
    GPIO_TypeDef*       sclPort = GPIOB;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. Clear PE bit.
    I2CHandle.Instance->CR1 &= ~(0x0001);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Alternate    = GPIO_AF4_I2C1;
    GPIO_InitStructure.Pull         = GPIO_PULLUP;
    GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStructure.Pin          = sclPin;
    HAL_GPIO_Init(sclPort, &GPIO_InitStructure);
    HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);

    GPIO_InitStructure.Pin          = sdaPin;
    HAL_GPIO_Init(sdaPort, &GPIO_InitStructure);
    HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_SET);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sclPort, sclPin))
    {
        asm("nop");
    }

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sdaPort, sdaPin))
    {
        asm("nop");
    }

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_RESET);

    //  5. Check SDA Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(sdaPort, sdaPin))
    {
        asm("nop");
    }

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_RESET);

    //  7. Check SCL Low level in GPIOx_IDR.
    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(sclPort, sclPin))
    {
        asm("nop");
    }

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sclPort, sclPin))
    {
        asm("nop");
    }

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sdaPort, sdaPin))
    {
        asm("nop");
    }

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode         = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Alternate    = GPIO_AF4_I2C1;

    GPIO_InitStructure.Pin          = sclPin;
    HAL_GPIO_Init(sclPort, &GPIO_InitStructure);

    GPIO_InitStructure.Pin          = sdaPin;
    HAL_GPIO_Init(sdaPort, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    I2CHandle.Instance->CR1 |= 0x8000;

    asm("nop");

    // 14. Clear SWRST bit in I2Cx_CR1 register.
    I2CHandle.Instance->CR1 &= ~0x8000;

    asm("nop");

    // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    I2CHandle.Instance->CR1 |= 0x0001;

    if (HAL_I2C_Init(&I2CHandle) != HAL_OK) {
        LOG.error() << "I2C Failed to Initialise" << LOG.flush;
    }
}

bool I2CDevice::write(uint8_t addr, uint8_t* data, int dataSize) {
    HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(&I2CHandle, addr<<1, data, dataSize, 10000);
    if(result == HAL_OK) {
        return true;
    } else {
        if(result == HAL_ERROR) {
            LOG.error() << "I2C request failed due to HAL_ERROR" << LOG.flush;
        }
        if(result == HAL_BUSY) {
            LOG.error() << "I2C request failed due to HAL_BUSY" << LOG.flush;
            /*I2C_ClearBusyFlagErratum();
            while(HAL_I2C_Master_Transmit(&I2CHandle, addr<<1, data, dataSize, 10000) != HAL_OK) {
                HAL_Delay(100);
            }*/
        }
        if(result == HAL_TIMEOUT) {
            LOG.error() << "I2C request failed due to HAL_TIMEOUT" << LOG.flush;
        }
        return false;
    }
}

bool I2CDevice::write(uint8_t addr, uint8_t data) {
    return write(addr, &data, 1);
}

bool I2CDevice::writeReg(uint8_t addr, uint8_t reg, uint8_t* data, int dataSize) {
    uint8_t* data_ = (uint8_t*)malloc(sizeof(uint8_t) * (dataSize + 2));
    data_[0] = reg;
    memmove(data_+1, data, dataSize);
    return write(addr, data_, dataSize+1);
}

bool I2CDevice::writeReg(uint8_t addr, uint8_t reg, int dataSize, uint8_t* data) {
    return writeReg(addr, reg, data, dataSize);
}

bool I2CDevice::writeWord(uint8_t addr, uint8_t reg, uint16_t data) {
    uint8_t data_[3] = {reg, (uint8_t)(data >> 8), (uint8_t)data};
    return write(addr, data_, 3);
}

bool I2CDevice::writeByte(uint8_t addr, uint8_t reg, uint8_t data) {
    return writeReg(addr, reg, &data, 1);
}

bool I2CDevice::writeBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(addr, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(addr, reg, b);
}

bool I2CDevice::writeBits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(addr, reg, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(addr, reg, b);
    } else {
        return false;
    }
}

bool I2CDevice::read(uint8_t addr, uint8_t* rXData, int dataSize) { 
    HAL_StatusTypeDef result = HAL_I2C_Master_Receive(&I2CHandle, addr<<1, rXData, dataSize, 10000);  
    if(result == HAL_OK) {
        return true;
    } else {
        if(result == HAL_ERROR) {
            LOG.error() << "I2C request failed due to HAL_ERROR" << LOG.flush;
        }
        if(result == HAL_BUSY) {
            LOG.error() << "I2C request failed due to HAL_BUSY" << LOG.flush;
        }
        if(result == HAL_TIMEOUT) {
            LOG.error() << "I2C request failed due to HAL_TIMEOUT" << LOG.flush;
        }
        return false;
    }
}

bool I2CDevice::readReg(uint8_t addr, uint8_t reg, uint8_t* rXData, int dataSize) { 
    bool writeResult = write(addr, reg);
    bool readResult = read(addr, rXData, dataSize);
    return writeResult && readResult;
}

bool I2CDevice::readReg(uint8_t addr, uint8_t reg, int dataSize, uint8_t* rXData) { 
    return readReg(addr, reg, rXData, dataSize);
}

bool I2CDevice::readByte(uint8_t addr, uint8_t reg, uint8_t* rXData) {
    return readReg(addr, reg, rXData, 1);
}

bool I2CDevice::readBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    bool count = readByte(addr, reg, &b);
    *data = b & (1 << bitNum);
    return count;
}

bool I2CDevice::readBits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t b;
    bool result = readByte(addr, reg, &b);
    if (result) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return result;
}

void I2CDevice::scan() {
    HAL_StatusTypeDef result;
 	int i;
    std::stringstream ss;
 	for (i=1; i<128; i++)
 	{
        result = HAL_I2C_IsDeviceReady(&I2CHandle, (uint16_t)(i<<1), 2, 200);
        if (result == HAL_OK)
        {
            ss << "0x" << std::hex << i << ", ";
        }
 	}
    LOG.info() << ss.str() << LOG.flush; // Received an ACK at that address
}