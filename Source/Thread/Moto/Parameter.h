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
#include "Sensor/RFID/EPCdef.h"

/*
 * 只包含需要从上位机配置的参数
 */
#pragma pack(1)
typedef struct parameter_tag{
	float KI;
	float KP;
	float KU;

	/*路线是否循环*/
    uint8_t cycleRoute;
    uint16_t StateRPM[EPC_FEAT_END];

    float KSP;
    float KSI;
    uint16_t station_addr;   //站台zigbee地址


}parameter_t;


extern parameter_t g_param;



#endif /* PARAMETER_H_ */
