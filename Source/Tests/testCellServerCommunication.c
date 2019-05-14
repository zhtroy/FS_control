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
#include "uartStdio.h"
#include "Sensor/CellCommunication/CellDriver.h"


static Void taskCellComMain(UArg a0, UArg a1)
{
	p_msg_t msg;
	float distance;
	char string[100];
	cell_packet_t packet;
	int count = 0;

	while(1){
		CellRecvPacket(&packet, BIOS_WAIT_FOREVER);


		count ++;
	}
}

void testServerCom_init()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;

	CellDriverInit();

	Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(taskCellComMain, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}



}
