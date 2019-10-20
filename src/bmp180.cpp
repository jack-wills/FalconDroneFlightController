#include "bmp180.h"

BMP180::BMP180(): i2cDevice(I2CDevice()) {
    i2cDevice.readReg(BMP180_ADDR, 0xAA, 22, buffer);
    ac1 = (((int16_t)buffer[0]) << 8) | buffer[1];
    ac2 = (((int16_t)buffer[2]) << 8) | buffer[3];
    ac3 = (((int16_t)buffer[4]) << 8) | buffer[5];
    ac4 = (((uint16_t)buffer[6]) << 8) | buffer[7];
    ac5 = (((uint16_t)buffer[8]) << 8) | buffer[9];
    ac6 = (((uint16_t)buffer[10]) << 8) | buffer[11];
    b1 = (((int16_t)buffer[12]) << 8) | buffer[13];
    b2 = (((int16_t)buffer[14]) << 8) | buffer[15];
    mb = (((int16_t)buffer[16]) << 8) | buffer[17];
    mc = (((int16_t)buffer[18]) << 8) | buffer[19];
    md = (((int16_t)buffer[20]) << 8) | buffer[21];

    xTaskCreate(this->startTaskImpl, "BMP180", 1024, this, configMAX_PRIORITIES-2, &taskHandle);
}

BMP180::BMP180(const BMP180& other): i2cDevice(other.i2cDevice) {

}

BMP180::~BMP180() {

}

void BMP180::startTaskImpl(void* _this){
    ((BMP180*)_this)->task();
}

void BMP180::task() {

 	TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 500;

	xLastWakeTime = xTaskGetTickCount();
	while (1) {
      loadValues();
      correctTemperature();
      correctPressure();
      getAltitude();
      //LOG.info() << "Altitude = " << altitude << " Temp = " << (float)temp << " Pressure = " << pressure << LOG.flush;
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

void BMP180::loadValues() {
    i2cDevice.writeByte(BMP180_ADDR, 0xF4, 0x2E);
    vTaskDelay(5);
    i2cDevice.readReg(BMP180_ADDR, 0xF6, 2, buffer);
    tempRaw = (((int16_t)buffer[0]) << 8) | buffer[1];
    
    // Write 0x34+(OSS<<6) into register 0xF4
    // Request a pressure reading w/ oversampling setting
    i2cDevice.writeByte(BMP180_ADDR, 0xF4, 0x34 + (OSS<<6));
    
    // Wait for conversion, delay time dependent on OSS
    vTaskDelay(5 + (5*OSS));
    
    // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
    i2cDevice.readReg(BMP180_ADDR, 0xF6, 3, buffer);
    
    pressureRaw = (((uint32_t) buffer[0] << 16) | ((uint32_t) buffer[1] << 8) | (uint32_t) buffer[2]) >> (8-OSS);
}

void BMP180::getAltitude() {
    altitude = 145366.45f*(1.0f-pow(((float)pressure)/101300.25f, 0.190284f));
}

int32_t BMP180::computeB5(int32_t UT) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}

void BMP180::correctTemperature() {
  B5 = computeB5(tempRaw);
  temp = (B5+8) >> 4;
  temp /= 10;
}

void BMP180::correctPressure() { 
  int32_t B3, B6, X1, X2, X3;
  uint32_t B4, B7;

  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << OSS) + 2) / 4;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)pressureRaw - B3) * (uint32_t)( 50000UL >> OSS );

  if (B7 < 0x80000000) {
    pressure = (B7 * 2) / B4;
  } else {
    pressure = (B7 / B4) * 2;
  }
  X1 = (pressure >> 8) * (pressure >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * pressure) >> 16;
  pressure += ((X1 + X2 + (int32_t)3791)>>4);
}
