/* Software I2C (bit-banging) simple API for MPU use */
#ifndef __I2C_SOFT_H__
#define __I2C_SOFT_H__

#include "main.h"
#include <stdint.h>

// Initialize software I2C using specified GPIO ports and pins
void I2C_Soft_Init(GPIO_TypeDef* sclPort, uint16_t sclPin, GPIO_TypeDef* sdaPort, uint16_t sdaPin);

// Write multiple bytes to a device register. Returns 0 on success.
uint8_t I2C_Soft_WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t len);

// Read multiple bytes starting at a device register. Returns 0 on success.
uint8_t I2C_Soft_ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t* buf, uint16_t len);

#endif // __I2C_SOFT_H__
