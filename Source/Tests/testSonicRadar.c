/*
 * testSonicRadar.c
 *
 *  Created on: 2018-12-7
 *      Author: zhtro
 */
#include "Message/Message.h"
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include "stdio.h"

extern Void taskSonicRadar(UArg a0, UArg a1);

Void taskSonicRadarMain(UArg a0, UArg a1)
{
	p_msg_t msg;
//	float distance;
	while(1){
		msg= Message_pend();
		if(msg->type==sonicradar){
			float distance =(float) *((uint16_t *) (&(msg->data[0])));


			printf("msg type = %s\t distance = %f mm \n", Message_getNameByType(msg->type),distance );
		}

	}
}

void testSonicRadar_init()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;


	Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(taskSonicRadarMain, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//Sonic radar
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(taskSonicRadar, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}
}
