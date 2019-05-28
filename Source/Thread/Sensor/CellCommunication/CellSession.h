/*
 * CellSession.h
 *
 *  Created on: 2019-5-16
 *      Author: zhtro
 */

#ifndef CELLSESSION_H_
#define CELLSESSION_H_

#include "stdint.h"
#include "Sensor/CellCommunication/CellDriver.h"
#include <ti/sysbios/knl/Clock.h>

typedef void (*DenyHandler)(cell_packet_t* p);
typedef void (*AllowHandler)(cell_packet_t * p);
typedef void (*TimeoutHandler)(void);

typedef struct {
	//是否开启状态
	uint8_t isOpen;
	//定时时钟
	Clock_Handle clock;
	//报文指令
	uint16_t cmd;
	//和通讯报文头部的reqid一致
	uint16_t reqid;
	//超时timeout后，关闭这个session
	uint32_t timeout;
	//拒绝后的回调函数
	DenyHandler denyhandler;
	//同意后的回调函数
	AllowHandler allowhandler;
	//超时后的回调函数，可用于超时重发
	TimeoutHandler timeouthandler;
}cell_session_t;

/*
 * API
 */

void CellSessionInit();
uint8_t CellSessionOpen(//报文指令
					uint16_t cmd,
					//和通讯报文头部的reqid一致
					uint16_t reqid,
					//超时timeout后，关闭这个session
					uint32_t timeout,
					//拒绝后的回调函数
					DenyHandler denyhandler,
					//同意后的回调函数
					AllowHandler allowhandler,
					TimeoutHandler timeouthandler);


#endif /* CELLSESSION_H_ */
