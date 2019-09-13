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
    if (HAL_I2C_Init(&I2CHandle) != HAL_OK) {
        LOG.error("I2C Failed to Initialise");
    }
}

I2CDevice::~I2CDevice() {

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
}

void I2CDevice::write(char addr, uint8_t* data, int dataSize) {
    HAL_I2C_Master_Transmit(&I2CHandle, addr<<1, data, dataSize, 10000);
}

void I2CDevice::write(char addr, uint8_t data) {
    write(addr, &data, 1);
}

void I2CDevice::read(char addr, uint8_t* rXData, int dataSize) { 
    HAL_I2C_Master_Receive(&I2CHandle, addr<<1, rXData, dataSize, 10000);  
}

void I2CDevice::writeReg(char addr, uint8_t reg, uint8_t data) {
    uint8_t data_[2] = {reg, data};
    write(addr, data_, 2);
}

void I2CDevice::readReg(char addr, uint8_t* rXData, int dataSize, uint8_t reg) { 
    write(addr, reg);
    read(addr, rXData, dataSize);
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
    LOG.info(ss.str()); // Received an ACK at that address
}