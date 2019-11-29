/*
 * testStateMachine.c
 *
 *  Created on: 2018-12-2
 *      Author: zhtro
 */



#include "Message/Message.h"
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include "uartStdio.h"
#include "watch.h"
#include "uart.h"
#include "soc_C6748.h"


Void DecisionTask(UArg a0, UArg a1)
{
	p_msg_t msg;
	enum WatchEvents e;

	Watch_init();
	while(1){
		msg= Message_pend();

		//将msg (数据消息) 进行处理，映射到HSM的消息上去

		if(msg->type == uart){
			switch (msg->data[0]){
				case's':
					e = Watch_SET_EVT;
					break;
				case 'm':
					e = Watch_MODE_EVT;
					break;
				default:
					continue;
			}
		}

		else if(msg->type == timer){
			e = Watch_TICK_EVT;
		}

		else{
			continue;
		}

		//hsm input

		Watch_Event(e);
	}
}

void UARTInputTask(UArg a0, UArg a1)
{
	uint8_t c;
	p_msg_t msg;
	while(1)
	{


		c = UARTCharGetNonBlocking(SOC_UART_1_REGS);

		msg = Message_getEmpty();

		msg ->type = uart;

		msg ->data[0] = c;

		Message_post(msg);

		Task_sleep(20);

	}
}

void rfidTask(UArg a0, UArg a1)
{
//	uint8_t c;
	p_msg_t msg;
	uint8_t rfidmsg[] = {"rfid epc field"};
	while(1)
	{

		msg = Message_getEmpty();

		msg ->type = rfid;

		memcpy(msg->data, rfidmsg, sizeof(rfidmsg));

		Message_post(msg);

		Task_sleep(20);

	}
}

void mmRadarTask(UArg a0, UArg a1)
{
//	uint8_t c;
	p_msg_t msg;
	uint8_t rfidmsg[] = {"mm radar field"};
	while(1)
	{

		msg = Message_getEmpty();

		msg ->type = mmradar;

		memcpy(msg->data, rfidmsg, sizeof(rfidmsg));

		Message_post(msg);

		Task_sleep(10);

	}
}

void TimerTask(UArg a0, UArg a1)
{
	p_msg_t msg;

	while(1)
	{
		msg = Message_getEmpty();
		msg->type = timer;
		Message_post(msg);
		Task_sleep(1000);
	}
}

void testStateMachine_init()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;


	Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(DecisionTask, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//UART
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(UARTInputTask, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//timer
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(TimerTask, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//rfid
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(rfidTask, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//mm radar
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(mmRadarTask, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

}
