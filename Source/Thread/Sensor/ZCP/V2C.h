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
 * 上报状态
 */
typedef struct
{
	uint8_t rfid[12];
	uint16_t rpm;
	uint8_t status;
}v2c_req_carstatus_t;

/*
 * 申请进站
 */
typedef struct
{
	uint8_t roadId[5];
}v2c_req_enterstation_t;

/*
 * 申请出站
 */
typedef struct
{
	uint8_t roadId[5];
}v2c_req_leavestation_t;

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
 * 进站响应
 */
typedef struct
{
	uint8_t status;
	uint8_t rfid[12];
}v2c_resp_enterstation_t;

/*
 * 开门响应
 */
typedef struct
{
	uint8_t operation;
	uint8_t status;
}v2c_resp_opendoor_t;
/*
 *
 * ===================================
 */

/*
 * API
 */
extern void StartIfNotRunning(Task_Handle * task,Task_FuncPtr func, int pri );
extern void V2CInit();
extern void V2CAskFrontID();
extern void V2CEnterStation();
extern void V2CLeaveStation();
extern void V2COpenDoor(uint8_t openOrClose);


#endif /* V2C_H_ */
