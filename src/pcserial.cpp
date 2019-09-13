#include "pcserial.h"

PCSerial::PCSerial(int baudRate) {
    __GPIOA_CLK_ENABLE(); 
    __USART2_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    UARTHandle = UART_HandleTypeDef();
    UARTHandle.Instance        = USART2;
    UARTHandle.Init.BaudRate   = baudRate;
    UARTHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UARTHandle.Init.StopBits   = UART_STOPBITS_1;
    UARTHandle.Init.Parity     = UART_PARITY_NONE;
    UARTHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UARTHandle.Init.Mode       = UART_MODE_TX_RX;
    
    if (HAL_UART_Init(&UARTHandle) != HAL_OK) 
        asm("bkpt 255");
}

PCSerial::~PCSerial() {

}

void PCSerial::print(std::string string) {
    const char *buf = string.c_str();
    HAL_UART_Transmit(&UARTHandle, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

void PCSerial::println(std::string string) {
    print(string + "\n");
}