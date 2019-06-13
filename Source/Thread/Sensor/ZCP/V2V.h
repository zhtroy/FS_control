/*
 * V2V.h
 *
 *  Created on: 2019-6-4
 *      Author: zhtro
 */

#ifndef V2V_H_
#define V2V_H_

#define V2V_ID_NONE (0xFFFF)

typedef struct
{
	uint16_t frontId;
	uint16_t backId;
}v2v_param_t;

/*
 * API
 */
void V2VInit();
void V2VHandShakeFrontCar(uint16_t frontid);

#endif /* V2V_H_ */
