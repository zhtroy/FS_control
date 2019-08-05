/*
 * messageQueue.h
 *
 *  Created on: 2018-12-2
 *      Author: zhtro
 */

#ifndef MESSAGE_H_
#define MESSAGE_H_

#include <ti/sysbios/knl/Queue.h>
#include "Message/InternalEvtCode.h"

#define NUMMSGS 256 /* number of messages */
#define MSGSIZE (128)

typedef enum{
	rfid,
	mmradar,
	uart,
	timer,
	sonicradar,
	cell,               //4G数据
	photon,				//对管
	changerail, 		//变轨
	motor,				//动力电机
	brake,    			//刹车
	error,				//错误
	zigbee,             //Zigbee
	internal,           //内部模块通信用的事件
	Empty               //缺省类型
}msg_type_t;

typedef struct MsgObj {
	Queue_Elem elem; /* first field for Queue */
	msg_type_t type; /*msg type */
	uint8_t data[MSGSIZE]; /* message value */
	uint32_t dataLen;      //数据长度
} msg_t, *p_msg_t;


//API=========================
void Message_init();
p_msg_t Message_getEmpty();
void Message_recycle(p_msg_t msg);
p_msg_t Message_pend();
void Message_post(p_msg_t);
char* Message_getNameByType(msg_type_t t);
void Message_postError(unsigned char errorcode);
void Message_postEvent(msg_type_t type, unsigned char code);


#endif /* MESSAGE_H_ */
