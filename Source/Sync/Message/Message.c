/*
 * testqueue.c
 *
 *  Created on: 2018-12-1
 *      Author: zhtro
 */
//TODO:
#include <xdc/std.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <xdc/runtime/System.h>

#include "stdint.h"
#include "Message/Message.h"


static Semaphore_Handle sem_msg;
static Queue_Handle msgQueue;
static Queue_Handle freeQueue;

static char * typeToName[]= {
		"rfid",
		"mmradar",
		"uart",
		"timer",
		"sonicradar",
		"cell",
		"Empty"
};


void Message_init(){


	Int i;
	p_msg_t msg;
	Error_Block eb;

    // 创建一个信号量
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_COUNTING;
    sem_msg = Semaphore_create(0, &semParams, NULL);

    msgQueue = Queue_create(NULL,NULL);
    freeQueue = Queue_create(NULL,NULL);


    //分配若干条消息的内存
	Error_init(&eb);
	msg = (msg_t *) Memory_alloc(NULL, NUMMSGS * sizeof(msg_t), 0, &eb);
	if (msg == NULL) {
		System_abort("Memory allocation failed");
	}


	/* Put all messages on freeQueue */
	for (i = 0; i < NUMMSGS; msg++, i++) {
		Queue_put(freeQueue, (Queue_Elem *) msg);
	}


}

//从freeQueue中获取一条空的消息
p_msg_t Message_getEmpty()
{
	p_msg_t msg;

	msg = Queue_get(freeQueue);
	/* fill in value */
	msg->type = Empty;

	return msg;
}

void Message_recycle(p_msg_t msg)
{
	/* put message */
	Queue_put(freeQueue, (Queue_Elem *) msg);
}

//等待一条消息
p_msg_t Message_pend()
{
	p_msg_t msg;

	Semaphore_pend(sem_msg, BIOS_WAIT_FOREVER);
	/* get message */
	msg = Queue_get(msgQueue);

	return msg;
}

//post 一条消息
void Message_post(p_msg_t msg)
{
	/* put message */
	Queue_put(msgQueue, (Queue_Elem *) msg);
	/* post semaphore */
	Semaphore_post(sem_msg);
}

//输入 msg_type_t 返回消息类型字符串
char* Message_getNameByType(msg_type_t t)
{
	return typeToName[(int) t];
}

void Message_postError(unsigned char errorcode)
{
	p_msg_t msg;

	msg = Message_getEmpty();
	msg->type = error;
	msg->data[0] = errorcode;
	msg->dataLen = 1;
	Message_post(msg);
}
