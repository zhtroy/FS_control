#ifndef __MPU9250_DRV_H__
#define __MPU9250_DRV_H__

#include "common.h"

#define EEPROM_SLV_ADDR (0x50)
#define EEPROM_ADDR_EPC	  (0)
#define EEPROM_LEN_EPC	  (12)

/* 函数定义 */
void mpu9250I2CInit(void);
int32_t mpu9250WriteBytes(uint8_t slvAddr, uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr);
int32_t mpu9250WriteBytesBlocking(uint8_t slvAddr,uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr);
int32_t mpu9250ReadBytes(uint8_t slvAddr,uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr);
int32_t mpu9250ReadBytesBlocking(uint8_t slvAddr,uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr);
int32_t mpu9250WriteBytesFreeLength(uint8_t slvAddr, uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr);
int32_t mpu9250WriteBytesFreeLengthBlocking(uint8_t slvAddr, uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr);


#endif
