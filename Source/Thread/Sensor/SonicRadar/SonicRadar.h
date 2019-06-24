/*
 * SonicRadar.h
 *
 *  Created on: 2019-6-24
 *      Author: zhtro
 */

#ifndef SONICRADAR_H_
#define SONICRADAR_H_

#include "stdint.h"
#include <xdc/std.h>

extern uint16_t SonicGetDistance();
extern Void taskSonicRadar(UArg a0, UArg a1);

#endif /* SONICRADAR_H_ */
