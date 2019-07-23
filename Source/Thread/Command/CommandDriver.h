/*
 * Command.h
 *
 *  Created on: 2019-7-4
 *      Author: zhtro
 *
 *  	用于接收串口发来的操作命令
 */

#ifndef COMMAND_H_
#define COMMANDDRIVER_H_

#include "stdint.h"
#include <xdc/std.h>

#define COMMAND_UART_NUM (5)

#define COMMAND_PACKET_HEAD_0 (0xFF)
#define COMMAND_PACKET_HEAD_1 (0xAA)

//1.     设置起始点
#define COMMAND_TYPE_SET_START_POINT  (1)
//2.     更新路线开始
#define COMMAND_TYPE_ROUTE_START  (2)
//3.     路线节点
#define COMMAND_TYPE_ROUTE_NODE  (3)
//4.     更新路线结束
#define COMMAND_TYPE_ROUTE_END  (4)
//5.     更新距离校准点开始
#define COMMAND_TYPE_CALIB_START  (5)
//6.     距离校准点节点
#define COMMAND_TYPE_CALIB_NODE  (6)
//7.     更新距离校准点结束
#define COMMAND_TYPE_CALIB_END  (7)

//10.  启动
#define COMMAND_TYPE_GO  (10)


#define COMMAND_PACKET_MAX_LEN (144)

typedef struct{
	uint8_t head[2];
	uint8_t len;  //从len开始计算的长度，包含len
	uint8_t type;
	uint8_t data[COMMAND_PACKET_MAX_LEN];
}command_packet_t;

extern void CommandDriverInit();
Bool CommandRecv(command_packet_t * packet,UInt timeout);


#endif /* COMMAND_H_ */
