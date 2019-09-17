
#include <stm32f401xe.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include <sstream>
#include <stdio.h>

#include "logger.h"
#include "imu.h"
#include "eeprom.h"
#include "fccomm.h"

//__attribute__((__section__(".user_data"))) const uint32_t userConfig[64] = {0};

/*----------------------------------------------------------------------------
 * SystemCoreClockConfigure: configure SystemCoreClock using HSI
                             (HSE is not populated on Nucleo board)
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /** Configure the main internal regulator output voltage 
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    /** Initializes the CPU, AHB and APB busses clocks 
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /** Initializes the CPU, AHB and APB busses clocks 
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

void* __dso_handle;

int main(void) {

    HAL_Init();
    SystemCoreClockConfigure();
    SystemCoreClockUpdate();

    __GPIOA_CLK_ENABLE(); 
    __GPIOB_CLK_ENABLE(); 
    __GPIOC_CLK_ENABLE(); 
    __GPIOD_CLK_ENABLE(); 

    GPIO_InitTypeDef GPIO_InitStruct; 

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // digital Input
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    Logger LOG = Logger("main");
    LOG.info() << "Falcon start up!" << LOG.flush;
    
    IMU imu = IMU(0x69);

    if (false) {
        HAL_Delay(100);
        imu.calibrateMagnetometer();
        HAL_Delay(100000000);
    }
	
    uint64_t tLast = HAL_GetTick();
    while(1) {
        imu.update();
        imu.printAngles();
        while(HAL_GetTick() - tLast < 50);
        tLast = HAL_GetTick();
    }
}

