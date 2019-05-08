/*
 * Parameter.h
 *
 *  Created on: 2019-4-8
 *      Author: zhtro
 */

#ifndef PARAMETER_H_
#define PARAMETER_H_

#include "stdint.h"
#include "Decision/CarState.h"

typedef struct parameter_tag{
	float KI;
	float KP;
	float KU;

	/*是否允许变轨*/
    uint8_t EnableChangeRail;
    uint16_t StateRPM[car_state_None];

    float KSP;
    float KSI;
    uint16_t REAR_CAR_ADDR;


}parameter_t;

extern parameter_t* ParamInstance();



#endif /* PARAMETER_H_ */
