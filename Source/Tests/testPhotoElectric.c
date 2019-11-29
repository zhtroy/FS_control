/*
 * testPhotoElectric.c
 *
 *  Created on: 2018-12-18
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
#include "Sensor/PhotoElectric/PhotoElectric.h"

static void testPhotoTask()
{
	while(1)
	{
		//跑马灯

		PhotoEleSetLight(0x512, PHOTO_LIGHT_0_ON |PHOTO_LIGHT_1_ON );
		Task_sleep(1000);
		PhotoEleSetLight(0x512, PHOTO_LIGHT_1_ON |PHOTO_LIGHT_2_ON );
		Task_sleep(1000);
		PhotoEleSetLight(0x512, PHOTO_LIGHT_2_ON |PHOTO_LIGHT_0_ON );
		Task_sleep(1000);
	}
}

void testPhotoElectric_init()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;

	Error_init(&eb);
    Task_Params_init(&taskParams);


	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create((ti_sysbios_knl_Task_FuncPtr)taskPhotoElectric, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	task = Task_create((ti_sysbios_knl_Task_FuncPtr)testPhotoTask, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}


}
