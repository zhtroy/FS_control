/*
 * system_param.h
 *
 *  Created on: 2019-6-30
 *      Author: DELL
 */

#ifndef SYSTEM_PARAM_H_
#define SYSTEM_PARAM_H_

#pragma pack(1)
typedef struct{
    uint16_t carID;
    uint8_t brakeDirection;
    uint8_t brakeOffset;
    uint8_t maxtThrottle;
    uint8_t brakeReverseRatio;
    uint32_t wheelPerimeter;  //车轮周长(mm)
    uint32_t encoderWheelPerimeter; //编码器测距轮周长(0.1mm)
    uint8_t changeRailDirection;  //变轨方向
}systemParameter_t;
#pragma pack()

#define EEPROM_PHY_ADDR (0x50)

extern systemParameter_t sysParam;

void ParmInit();

#endif /* SYSTEM_PARAM_H_ */
