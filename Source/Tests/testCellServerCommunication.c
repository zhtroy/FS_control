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
#include "Sensor/CellCommunication/Packet/PacketBuilder.h"
#include "Lib/bitop.h"


typedef enum{
	unknown,
	freestate,
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
	orderstate_t state = freestate;
	orderstate_t laststate = freestate;
	packet_cabstatechange_t cabstatechange;
	Bool recved = 0;
	const int RECV_TIMEOUT = 1000;
	const int SESSIONID = 0x100;

	while(1)
	{
		/*
		 * test
		 */
		Task_sleep(500);
		PacketBuildCabPulse(&sendPacket,SESSIONID ,CELL_ADDR_TESTCAR , CELL_ADDR_VRC_BACKUP);
		CellSendPacket(&sendPacket);

		recved = CellRecvPacket(&recvPacket, RECV_TIMEOUT);

		continue;
		/*
		 * recved为0时，表示超时，保证状态机可以继续运行
		 * recved为1时，表示收到了包
		 */
		recved = 0;
		recved = CellRecvPacket(&recvPacket, RECV_TIMEOUT);

		/*
		 * 对type == CELL_TYPE_REQ的packet做出回应，不改变状态
		 */
		if (recved  && (CellPacketGetType(&recvPacket) == CELL_TYPE_REQ))
		{
			switch(recvPacket.cmd)
			{
				case CELL_CMD_ORDERCAB:
				{
					if(state != freestate)
						CellPacketBuildResponse(&recvPacket,&sendPacket,0,NULL,0);
					else
						CellPacketBuildResponse(&recvPacket,&sendPacket,1,NULL,0);

					CellSendPacket(&sendPacket);
					break;
				}

				case CELL_CMD_UPDATEROUTE:
				{
					if(state == freestate ||
					   state == ordered ||
					   state == shipping)
					{
						CellPacketBuildResponse(&recvPacket,&sendPacket,0,NULL,0);
					}
					else
					{
						CellPacketBuildResponse(&recvPacket, &sendPacket, 1, NULL, 0);
					}
					CellSendPacket(&sendPacket);
					break;
				}
			}
		}



		/*
		 * 状态机
		 */
		switch(state)
		{
			case freestate:
			{
				if(recved && recvPacket.cmd == CELL_CMD_ORDERCAB)
				{
					state = ordered;
				}
				break;
			}

			case ordered:
			{
				if(recved && recvPacket.cmd == CELL_CMD_UPDATEROUTE)  //一收到路径更新，就认为车辆到达目的地
				{
					state = arrived_start;
				}
				break;
			}

			case arrived_start:
			{
				state = loading;
				break;
			}

			case loading:
			{
				state = shipping;
				break;
			}

			case shipping:
			{
				if(recved && recvPacket.cmd == CELL_CMD_UPDATEROUTE)
				{
					state = arrived_end;
				}
				break;
			}

			case arrived_end:
			{
				state = unloading;
				break;
			}

			case unloading:
			{
				state = free;
				break;
			}


		}  //	switch(state)

		/*
		 * 订单状态改变
		 * 发送cabstatechange报文
		 */
		if(state != laststate)
		{

			cabstatechange.nid = 0x1;
			cabstatechange.cs = 0;
			BF_SET(cabstatechange.cs,state,0,4);
			cabstatechange.ps = 0;
			BF_SET(cabstatechange.ps,laststate,0,4);

			PacketBuildCabStateChange(&sendPacket,recvPacket.reqid,recvPacket.dstid,recvPacket.srcid,cabstatechange);
			CellSendPacket(&sendPacket);

		}
		laststate = state;

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
	taskParams.stackSize = 4096;
	task = Task_create(taskCellMock, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}



}
