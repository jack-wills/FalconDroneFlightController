#pragma once
#include "mpu9250.h"
#include "logger.h"

#include "FreeRTOS.h"
#include "task.h"

#define BMP180_ADDR 0x77

class BMP180
{
public:
	BMP180();
    BMP180(const BMP180& other);
	~BMP180();
private:
    static void startTaskImpl(void* _this);
    void task();
    void correctPressure();
    void correctTemperature();
    void loadValues();
    void getAltitude();
    int32_t computeB5(int32_t UT);

    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;

    int32_t B5;

    uint32_t pressureRaw;
    uint16_t tempRaw;
    int16_t temp;
    int32_t pressure;
    float altitude;
    const unsigned char OSS = 3;  // Oversampling Setting
    /* blz 12 Datasheet
    OSS=0 ultra Low Power Setting, 1 sample, 4.5 ms 3uA
    OSS=1 Standard Power Setting, 2 samples, 7.5 ms 5uA
    OSS=2 High Resolution,              4 samples, 13.5 ms 7uA
    OSS=3 Ultra High Resolution,    2 samples, 25.5 ms 12uA
    */

    uint8_t buffer[22];

    I2CDevice i2cDevice;

    Logger LOG = Logger("BMP180");

    TaskHandle_t taskHandle;
};

