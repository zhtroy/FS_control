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


typedef enum{
	free,
	ordered,
	arrived_start,
	loading,
	shipping,
	arrived_end,
	unloading

}orderstate_t;

/*
 * 一个测试流程，其中维护一个车辆预定状态机
 */
static Void taskCellMock(UArg a0, UArg a1)
{
	cell_packet_t recvPacket, sendPacket;
	orderstate_t state = free;

	while(1)
	{
		CellRecvPacket(&recvPacket, BIOS_WAIT_FOREVER);

		/*
		 * 对type == request的packet做出回应，不改变状态
		 */
		if(CellPacketGetType(&recvPacket) == CELL_TYPE_REQ)
		{
			switch(recvPacket.cmd)
			{
				case CELL_CMD_ORDERCAB:
				{
					if(state != free)
						CellPacketBuildResponse(&recvPacket,&sendPacket,0,NULL,0);
					else
						CellPacketBuildResponse(&recvPacket,&sendPacket,1,NULL,0);

					CellSendPacket(&sendPacket);
					break;
				}

				case CELL_CMD_UPDATEROUTE:
				{
					if(state == free ||
					   state == ordered ||
					   state == shipping)
					{
						CellPacketBuildResponse(&recvPacket,&sendPacket,0,NULL,0);
					}
					CellPacketBuildResponse
				}
			}
		}

		/*
		 * 状态机
		 */

	}
}

void testServerCom_init()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;

	CellSessionInit();

	Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(taskCellMock, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}



}
