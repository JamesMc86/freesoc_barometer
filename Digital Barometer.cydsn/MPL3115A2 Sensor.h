/* ========================================
 * Library for I2C comms with the MPL3115A2
 * Altitude/Pressure/Temperature Sensor by
 * Hobbytronics and Sparkfun.
 *
 * Copyright James McNally, 2013
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <I2C.h>

#define TEMP_ADDR 0x60

//Pressure modes.
#define MPL3115A2_BAR_MODE 0
#define MPL3115A2_ALT_MODE 1


uint8 MPL3115A2Start(uint8 mode);
void MPL3115A2SetStandby();

//Low level register access
uint8 MPL3115A2ReadReg(uint8 reg_addr);
uint8 MPL3115A2WriteReg(uint8 regAddr, uint8 regValue);

//Data Access
float MPL3115A2ReadTemp();
float MPL3115A2ReadPressure();

//[] END OF FILE
