/*
 * testCellCommunication.c
 *
 *  Created on: 2018-12-8
 *      Author: zhtro
 */


#include "Message/Message.h"
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include "stdio.h"
#include "DSP_Uart/dsp_uart2.h"

extern Void taskCellCommunication(UArg a0, UArg a1);

static Void taskCellComMain(UArg a0, UArg a1)
{
	p_msg_t msg;
//	float distance;
	while(1){
		msg= Message_pend();
		if(msg->type==cell){

			System_printf("recv 4G command %s \n", msg->data  );
			UART2Puts("recved command from server", -2);
		}

	}
}

void testCellCom_init()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;


	Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(taskCellComMain, &taskParams, &eb);
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
}
