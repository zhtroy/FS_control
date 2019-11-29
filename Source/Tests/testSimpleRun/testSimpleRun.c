/*
 *  模拟机车上电启动，运行过程
 *
 *  测试4G， RFID， 毫米波雷达的数据
 *  验证程序框架
 *
 *  Created on: 2018-12-2
 *      Author: zhtro
 */


#include <string.h>
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

extern Void taskRFID(UArg a0, UArg a1);
extern Void taskCellCommunication(UArg a0, UArg a1);

typedef enum{
	init,
	waitRFID,
	speed60
}car_state_t;


//var ----------------------
static float car_speed = 0;


Void taskSimpleRun(UArg a0, UArg a1)
{
	p_msg_t msg;

	car_state_t state;

	state = init;

	while(1){
		msg= Message_pend();

		//将msg (数据消息) 进行处理，映射到HSM的消息上去

		//switch...case 状态机
		switch(state){
		case init:
			if(msg->type == cell){
				if(strcmp((const char *)msg->data, (const char *)"go") == 0)  //命令go
				{
					UARTPuts("state: into waitRFID\n", -1);
					state = waitRFID;
				}
			}
			break;
		case waitRFID:
			if(msg->type == rfid){
				if(msg->data[0] == 0x5)    //5号id是起点
				{
					UARTPuts("state: into speed60\n",-1);
					state = speed60;
					car_speed = 60;
				}
			}
			break;
		case speed60:
			break;
		}


	}
}


void taskTheMotor(UArg a0, UArg a1)
{
//	uint8_t c;
//	p_msg_t msg;
	char str[50];

	memset(str,1,50);
	while(1)
	{
		sprintf(str,"car running at speed: %f\n",car_speed);
		UARTPuts(str,-1);

		Task_sleep(20);

	}
}





void testSimpleRun_init()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;


	//决策
	Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(taskSimpleRun, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//RFID
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(taskRFID, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//4G
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(taskCellCommunication, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//motor
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(taskTheMotor, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

}
