
#include <stm32f401xe.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include <sstream>
#include <stdio.h>

#include "logger.h"
#include "i2cdevice.h"

/*----------------------------------------------------------------------------
 * SystemCoreClockConfigure: configure SystemCoreClock using HSI
                             (HSE is not populated on Nucleo board)
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure(void) {
    RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable HSI
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Wait for HSI Ready

    RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSI is system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

    FLASH->ACR  = FLASH_ACR_PRFTEN;                          // Enable Prefetch Buffer
    FLASH->ACR |= FLASH_ACR_ICEN;                            // Instruction cache enable
    FLASH->ACR |= FLASH_ACR_DCEN;                            // Data cache enable
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;                     // Flash 5 wait state

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;                        // APB1 = HCLK/4
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;                        // APB2 = HCLK/2

    RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL

    // PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P
    RCC->PLLCFGR = ( 16ul                   |                // PLL_M =  16
                    (384ul <<  6)            |                // PLL_N = 384
                    (  3ul << 16)            |                // PLL_P =   8
                    (RCC_PLLCFGR_PLLSRC_HSI) |                // PLL_SRC = HSI
                    (  8ul << 24)             );              // PLL_Q =   8

    RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
    while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Wait till PLL is ready

    RCC->CFGR &= ~RCC_CFGR_SW;                               // Select PLL as system clock source
    RCC->CFGR |=  RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait till PLL is system clock src
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

void* __dso_handle;

int main(void) {

    HAL_Init();
    SystemCoreClockConfigure();                              // configure System Clock
    SystemCoreClockUpdate();
    /*
     * Turn on the GPIOA unit,
     * -> see section 6.3.9 in the manual
     */
    __GPIOA_CLK_ENABLE(); 
    __GPIOB_CLK_ENABLE(); 
    __GPIOC_CLK_ENABLE(); 
    __GPIOD_CLK_ENABLE(); 
    
    Logger LOG = Logger("main");
    LOG.info("Falcon start up!");

    I2CDevice i2cDevice = I2CDevice();

    GPIO_InitTypeDef GPIO_InitStruct; 

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // digital Input
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    i2cDevice.writeReg(0x68, 0x6B, 0x00);
	
    while(1) {
        uint8_t aRxBuffer[14]; 
        i2cDevice.readReg(0x68, aRxBuffer, 14, 0x3B);
        int16_t accelX = (int16_t)(aRxBuffer[0]<<8|aRxBuffer[1]);
        int16_t accelY = (int16_t)(aRxBuffer[2]<<8|aRxBuffer[3]);
        int16_t accelZ = (int16_t)(aRxBuffer[4]<<8|aRxBuffer[5]);
        int16_t temp = (int16_t)(aRxBuffer[6]<<8|aRxBuffer[7]);
        int16_t gyroX = (int16_t)(aRxBuffer[8]<<8|aRxBuffer[9]);
        int16_t gyroY = (int16_t)(aRxBuffer[10]<<8|aRxBuffer[11]);
        int16_t gyroZ = (int16_t)(aRxBuffer[12]<<8|aRxBuffer[13]);
        std::stringstream ss;
        ss << "results: ";
        for(int i=0; i<7; ++i) {
            ss << (int16_t)(aRxBuffer[2*i]<<8|aRxBuffer[(2*i)+1]) << ",";
        }
        LOG.info(ss.str());
        HAL_Delay(1000);
    }
}

