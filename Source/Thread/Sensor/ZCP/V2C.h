/*
 * V2C.h
 *
 *  Created on: 2019-6-3
 *      Author: zhtro
 */

#ifndef V2C_H_
#define V2C_H_


#include "stdint.h"
#include <ti/sysbios/knl/Task.h>

/*
 * 请求===================================
 */
/*
 * 请求前车ID
 */
typedef struct
{
	uint8_t rfid[12];
	uint8_t railpos;
}v2c_req_forward_id_t;
/*
 * ===================================
 */

/*
 * 响应===================================
 */
/*
 * 响应前车ID
 */
typedef struct
{
	uint8_t status;
	uint16_t forwardid;
}v2c_resp_forward_id_t;
/*
 * ===================================
 */

/*
 * API
 */
extern void StartIfNotRunning(Task_Handle task,Task_FuncPtr func, int pri );
extern void V2CInit();
extern void V2CAskFrontID();


#endif /* V2C_H_ */
