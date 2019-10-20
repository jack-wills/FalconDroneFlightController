#include "motorcontroller.h"

#define TIM2_PSC_SET 	16
#define TIM2_ARR_SET 	19999

MotorController::MotorController(IMU &imu) {
    this->imu = &imu; 
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    PWM1Handle.Instance = TIM2;
    PWM1Handle.Init.Prescaler = 1000;
    PWM1Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    PWM1Handle.Init.Period = 1700;
    PWM1Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    PWM2Handle.Instance = TIM3;
    PWM2Handle.Init.Prescaler = 1000;
    PWM2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    PWM2Handle.Init.Period = 1700;
    PWM2Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    if (HAL_TIM_Base_Init(&PWM1Handle) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }
    if (HAL_TIM_Base_Init(&PWM2Handle) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }


    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&PWM1Handle, &sClockSourceConfig) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }
    if (HAL_TIM_ConfigClockSource(&PWM2Handle, &sClockSourceConfig) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }

    if (HAL_TIM_PWM_Init(&PWM1Handle) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }
    if (HAL_TIM_PWM_Init(&PWM2Handle) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&PWM1Handle, &sMasterConfig) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }
    if (HAL_TIMEx_MasterConfigSynchronization(&PWM2Handle, &sMasterConfig) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&PWM1Handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }
    
    if (HAL_TIM_PWM_ConfigChannel(&PWM1Handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }

    if (HAL_TIM_PWM_ConfigChannel(&PWM2Handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }

    if (HAL_TIM_PWM_ConfigChannel(&PWM2Handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        LOG.error() << "Failed to initialise Motor control" << LOG.flush;
    }

    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    
    
    HAL_TIM_PWM_Start (&PWM1Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start (&PWM1Handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start (&PWM2Handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start (&PWM2Handle, TIM_CHANNEL_2);

    PWM1Handle.Instance->CCR1 = 0;   // D13 FL
    PWM1Handle.Instance->CCR2 = 0;   // A1 BR
    PWM2Handle.Instance->CCR1 = 0;   // D5 BL
    PWM2Handle.Instance->CCR2 = 0;   // D9 FR
    
    xTaskCreate(this->startTaskImpl, "MotorController", 1024, this, configMAX_PRIORITIES-1, &taskHandle);
}

MotorController::~MotorController() {

}

MotorController::MotorController(const MotorController& other): imu(other.imu) {
    PWM1Handle = other.PWM1Handle;
    PWM2Handle = other.PWM2Handle;
    GPIO_InitStruct = other.GPIO_InitStruct;
}

void MotorController::setThrottle(uint16_t throttle) {
    if (throttle > 100) {
        this->throttle = 100;
    } else {
        this->throttle = throttle;
    }
}


void MotorController::calculatePID(float pitch, float roll, float yaw) {
    //PID Pitch
    pidPitchError = pitch - pitchInput;
    pidPitchIntegral += pidPitchError;
    pidPitchValue = (pidGainPP * pidPitchError) + (pidGainIP * pidPitchIntegral) + (pidGainDP * (pidPitchError - pidPitchOld));
    pidPitchOld = pidPitchError;
    //PID Roll
    pidRollError = roll - rollInput;
    pidRollIntegral += pidRollError;
    pidRollValue = (pidGainPR * pidRollError) + (pidGainIR * pidRollIntegral) + (pidGainDR * (pidRollError - pidRollOld));
    pidRollOld = pidRollError;
    //PID Yaw
    pidYawError = yaw - yawInput;
    pidYawIntegral += pidYawError;
    //pidYawValue = (pidGainPY * pidYawError) + (pidGainIY * pidYawIntegral) + (pidGainDY * (pidYawError - pidYawOld));
    pidYawOld = pidYawError;
}


float map(float in, float inMin, float inMax, float outMin, float outMax) {
    // check it's within the range
    if (inMin<inMax) { 
        if (in <= inMin) 
            return outMin;
        if (in >= inMax)
            return outMax;
    } else { 
        if (in >= inMin) 
            return outMin;
        if (in <= inMax)
            return outMax;
    }
    return outMin + ((in-inMin)/(inMax-inMin))*(outMax-outMin);
}

void MotorController::startTaskImpl(void* _this) {
    ((MotorController*)_this)->task();
}

void MotorController::task() {

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50;

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        float roll,pitch,yaw;
        imu->getAngles(&pitch, &roll, &yaw);
        calculatePID(pitch, roll, yaw);
        
        float throttleFL = throttle + pidPitchValue - pidRollValue + pidYawValue;
        float throttleFR = throttle + pidPitchValue + pidRollValue - pidYawValue;
        float throttleBL = throttle - pidPitchValue - pidRollValue - pidYawValue;
        float throttleBR = throttle - pidPitchValue + pidRollValue + pidYawValue;
        
        PWM1Handle.Instance->CCR1 = map(throttleFR, 0, 120, 85.f, 170.f);   // D13
        PWM1Handle.Instance->CCR2 = map(throttleFL, 0, 120, 85.f, 170.f);   // A1
        PWM2Handle.Instance->CCR1 = map(throttleBL, 0, 120, 85.f, 170.f);   // D5
        PWM2Handle.Instance->CCR2 = map(throttleBR, 0, 120, 85.f, 170.f);   // D9
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
	  }
}