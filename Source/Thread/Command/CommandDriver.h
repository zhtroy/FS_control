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
#define COMMAND_TYPE_SET_START_POINT_RESPONSE  (0x11)
//2.     更新路线开始
#define COMMAND_TYPE_ROUTE_START  (2)
//3.     路线节点
#define COMMAND_TYPE_ROUTE_NODE  (3)
//4.     更新路线结束
#define COMMAND_TYPE_ROUTE_END  (4)
#define COMMAND_TYPE_ROUTE_RESPONSE (0x14)
//5.     更新距离校准点开始
#define COMMAND_TYPE_CALIB_START  (5)
//6.     距离校准点节点
#define COMMAND_TYPE_CALIB_NODE  (6)
//7.     更新距离校准点结束
#define COMMAND_TYPE_CALIB_END  (7)
#define COMMAND_TYPE_CALIB_RESPONSE (0x17)

//10.  启动
#define COMMAND_TYPE_GO  (0x0A)
#define COMMAND_TYPE_GO_RESPONSE   (0x1A)

//设置循环路线
#define COMMAND_TYPE_SETLOOP   (0x0B)
#define COMMAND_TYPE_SETLOOP_RESPONSE (0x1B)

//车门动作
#define COMMAND_TYPE_DOOR     (0x0C)
#define COMMAND_TYPE_DOOR_RESPONSE     (0x1C)

//修改路线
#define COMMAND_TYPE_CHANGE_ROUTE_END       (0x0D)
#define COMMAND_TYPE_CHANGE_ROUTE_RESPONSE  (0x1D)

//设置当前的RFID
#define COMMAND_TYPE_SET_RFID    		(0x0E)
#define COMMAND_TYPE_SET_RFID_RESPONSE	(0x1E)

//车辆信息回传
#define COMMAND_TYPE_CAR_INFO (0x80)
//路线运行结束
#define COMMAND_TYPE_ROUTE_FINISH (0x81)
//临停结束
#define COMMAND_TYPE_TEMPSTOP_FINISH (0x83)

#define COMMAND_PACKET_MAX_LEN (144)

typedef struct{
	uint8_t head[2];
	uint8_t len;  //从len开始计算的长度，包含len
	uint8_t type;
	uint8_t data[COMMAND_PACKET_MAX_LEN];
}command_packet_t;

extern void CommandDriverInit();
extern Bool CommandRecv(command_packet_t * packet,UInt timeout);
extern void CommandSend(char data[], int datalen, uint8_t type);


#endif /* COMMAND_H_ */
