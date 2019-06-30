/*
 * system_param.h
 *
 *  Created on: 2019-6-30
 *      Author: DELL
 */

#ifndef SYSTEM_PARAM_H_
#define SYSTEM_PARAM_H_

typedef struct{
    uint16_t carID;
    uint8_t brakeDirection;
    uint8_t brakeOffset;
    uint8_t maxtThrottle;
}systemParameter_t;

#define EEPROM_PHY_ADDR (0x50)

extern systemParameter_t sysParam;

void ParmInit();

#endif /* SYSTEM_PARAM_H_ */
