
#include <stm32f401xe.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include <sstream>
#include <stdio.h>

#include "logger.h"
#include "i2cdevice.h"
#include "mpu9250.h"

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

    GPIO_InitTypeDef GPIO_InitStruct; 

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // digital Input
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    Logger LOG = Logger("main");
    LOG.info("Falcon start up!");

    I2CDevice i2cDevice = I2CDevice();

    MPU9250 mpu = MPU9250(0x69, i2cDevice);

    uint8_t accelFullScale = MPU9250_ACCEL_FS_8;
    mpu.setFullScaleAccelRange(accelFullScale);
    float aRes = ((float)(accelFullScale+1))/16384.0f;

    uint8_t gyroFullScale = MPU9250_GYRO_FS_500;
    mpu.setFullScaleGyroRange(gyroFullScale);
    float gRes = ((float)(gyroFullScale+1))/131.0f;

    int16_t aXRaw,aYRaw,aZRaw,gXRaw,gYRaw,gZRaw,mXRaw,mYRaw,mZRaw;

    int64_t gXOffset = 0, gYOffset = 0, gZOffset = 0;
    float aXOffset = 0.82f, aYOffset = 0.05f, aZOffset = -0.21;

    float aX,aY,aZ,gX,gY,gZ;
    float pitch,roll,yaw;
    float aPitch,aRoll;

    //Calc starting heading from just acceleration values;
    mpu.getAcceleration(&aXRaw, &aYRaw, &aZRaw);
    aX = (float)aXRaw*aRes;
    aY = (float)aYRaw*aRes;   
    aZ = (float)aZRaw*aRes; 
    pitch = atan2((float)aY, (float)aZ) * 57.296f;
    roll = atan2((float)aX, (float)aZ) * -57.296f;
    yaw = 0;

    LOG.info("Starting calibrations...");
    for(int i = 0; i < 500; i++){
        mpu.getRotation(&gXRaw,&gYRaw,&gZRaw);
        gXOffset += gXRaw;
        gYOffset += gYRaw;
        gZOffset += gZRaw;
        HAL_Delay(3);
    }

    LOG.info("Calibrations Finished");
    gXOffset /= 500;
    gYOffset /= 500;
    gZOffset /= 500;

    uint64_t tLast = HAL_GetTick();
    int count = 0;
	
    while(1) {
        mpu.getMotion9(&aXRaw,&aYRaw,&aZRaw,&gXRaw,&gYRaw,&gZRaw,&mXRaw,&mYRaw,&mZRaw);

        aX = (float)(aXRaw*aRes)-aXOffset;
        aY = (float)(aYRaw*aRes)-aYOffset;   
        aZ = (float)(aZRaw*aRes)-aZOffset; 

        aPitch = atan2((float)aY, (float)aZ) * 57.296f;
        aRoll = atan2((float)aX, (float)aZ) * -57.296f;

        gX = (float)(gXRaw-gXOffset)*gRes;
        gY = (float)(gYRaw-gYOffset)*gRes;  
        gZ = (float)(gZRaw-gZOffset)*gRes; 
        if (count != 0) {
            pitch += gX * 0.005;
            roll += gY * 0.005;
            yaw += gZ * 0.005;
        } else {
            // Print cycle
            pitch += gX * 0.05;
            roll += gY * 0.05;
            yaw += gZ * 0.05;
        }

        pitch += roll * sin(gZ * 0.01745f * 0.05);
        roll -= pitch * sin(gZ * 0.01745f * 0.05);

        pitch = pitch * 0.95f + aPitch * 0.05f;
        roll = roll * 0.95f + aRoll * 0.05f;
        
        while(HAL_GetTick() - tLast < 5);
        tLast = HAL_GetTick();

        if (count++ < 3) {
            while(HAL_GetTick() - tLast < 5);
            tLast = HAL_GetTick();
        } else {
            // Print cycle
            count = 0;

            std::stringstream ss;
            ss << "Angle: " << LOG.ftoa(roll, 2) << " ,\t " << LOG.ftoa(pitch, 2) << " ,\t " << LOG.ftoa(yaw, 2);
            LOG.info(ss.str());
            
            while(HAL_GetTick() - tLast < 50);
            tLast = HAL_GetTick();
        }
    }
}

