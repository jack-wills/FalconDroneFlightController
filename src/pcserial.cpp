#include "pcserial.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

QueueHandle_t serialRXQueue = 0;

SemaphoreHandle_t uartMutex = 0;

UART_HandleTypeDef UARTHandle;
DMA_HandleTypeDef DMAHandle;

#define DMA_RX_BUFFER_SIZE          64
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
extern "C" void DMA1_Stream5_IRQHandler(void)
{
	typedef struct
	{
		__IO uint32_t ISR;   /*!< DMA interrupt status register */
		__IO uint32_t Reserved0;
		__IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
	} DMA_Base_Registers;

	DMA_Base_Registers *regs = (DMA_Base_Registers *)DMAHandle.StreamBaseAddress;
	
	if(__HAL_DMA_GET_IT_SOURCE(&DMAHandle, DMA_IT_TC) != RESET)   // if the source is TC
	{
        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
		/* Clear the transfer complete flag */
		regs->IFCR = DMA_FLAG_TCIF1_5 << DMAHandle.StreamIndex;

        if (serialRXQueue) {
            char queueBuffer[64];
            sprintf(queueBuffer, (char*)DMA_RX_Buffer);
            xQueueSendFromISR(serialRXQueue, (void*)queueBuffer, &xHigherPriorityTaskWoken);
        }

		/* Prepare DMA for next transfer */
        /* Important! DMA stream won't start if all flags are not cleared first */
 
        regs->IFCR = 0x3FU << DMAHandle.StreamIndex; // clear all interrupts
		DMAHandle.Instance->M0AR = (uint32_t)DMA_RX_Buffer;   /* Set memory address for DMA again */
        DMAHandle.Instance->NDTR = DMA_RX_BUFFER_SIZE;    /* Set number of bytes to receive */
        DMAHandle.Instance->CR |= DMA_SxCR_EN;            /* Start DMA transfer */
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

/**
  * @brief This function handles USART2 global interrupt.
  */
extern "C" void USART2_IRQHandler(void)
{
	if (UARTHandle.Instance->SR & UART_FLAG_IDLE)           /* if Idle flag is set */
	{
		volatile uint32_t tmp;                  /* Must be volatile to prevent optimizations */
        tmp = UARTHandle.Instance->SR;                       /* Read status register */
        tmp = UARTHandle.Instance->DR;                       /* Read data register */
		tmp++;
		DMAHandle.Instance->CR &= ~DMA_SxCR_EN;       /* Disabling DMA will force transfer complete interrupt if enabled */       
	}
}

PCSerial::PCSerial(int baudRate) {
    if (uartMutex == 0) {
        uartMutex = xSemaphoreCreateMutex();

        __GPIOA_CLK_ENABLE(); 
        __USART2_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

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

        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);

        DMAHandle.Instance = DMA1_Stream5;
        DMAHandle.Init.Channel = DMA_CHANNEL_4;
        DMAHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
        DMAHandle.Init.PeriphInc = DMA_PINC_DISABLE;
        DMAHandle.Init.MemInc = DMA_MINC_ENABLE;
        DMAHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        DMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        DMAHandle.Init.Mode = DMA_NORMAL;
        DMAHandle.Init.Priority = DMA_PRIORITY_LOW;
        DMAHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&DMAHandle) != HAL_OK)
        {
            asm("bkpt 255");
        }

        __HAL_LINKDMA(&UARTHandle,hdmarx,DMAHandle);

        HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 5);
        HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
        __HAL_UART_ENABLE_IT(&UARTHandle, UART_IT_IDLE);   // enable idle line interrupt
        __HAL_DMA_ENABLE_IT (&DMAHandle, DMA_IT_TC);  // enable DMA Tx cplt interrupt

        HAL_UART_Receive_DMA (&UARTHandle, DMA_RX_Buffer, DMA_RX_BUFFER_SIZE);

        DMAHandle.Instance->CR &= ~DMA_SxCR_HTIE;  // disable uart half tx interrupt
    }
}

PCSerial::~PCSerial() {

}

void PCSerial::print(std::string string) {
    const char *buf = string.c_str();
    if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
        HAL_UART_Transmit(&UARTHandle, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
        xSemaphoreGive(uartMutex);
    }
}

void PCSerial::println(std::string string) {
    print(string + "\n");
}