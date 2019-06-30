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
#include <ti/sysbios/knl/Mailbox.h>
#include "stdio.h"
#include "DSP_Uart/dsp_uart2.h"
#include "uartStdio.h"
#include "Sensor/CellCommunication/CellDriver.h"
#include "Sensor/CellCommunication/Packet/Packet.h"
#include "Lib/bitop.h"
#include "Lib/vector.h"
#include "Decision/CarHsm.h"
#include "Sensor/RFID/RFID_task.h"
#include "Moto/task_moto.h"
#include "Sensor/CellCommunication/CellCommunication.h"
#include "logLib.h"
#include "Moto/task_moto.h"

#define TEST 0

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

static char* stateStrings[] =
{
		"unknown",
		"freestate",
		"ordered",
		"arrived_start",
		"loading",
		"shipping",
		"arrived_end",
		"unloading"
};

static Mailbox_Handle m_cellMbox = 0;
//订单状态
static orderstate_t m_orderstate = freestate;

/*
 * 定时发送线程
 */
static Void taskCellSend(UArg a0, UArg a1)
{
	cell_packet_t sendPacket;
	packet_cabstatus_t cabstatus;
	epc_t myepc;

	const int SEND_INTERVAL = 1000;


	while(1)
	{
		Task_sleep(SEND_INTERVAL);

		/*
		 * 发送cabstatus
		 */
		cabstatus.acc = 0;
		//上一次读到的RFID
		myepc = RFIDGetEpc();
		cabstatus.nid = EPCgetShortID(&myepc);


		//离上一个RFID节点的偏移
		cabstatus.offset = ((double)MotoGetCarDistance() - (double)(myepc.distance))/10.0;
		//速度
		cabstatus.speed = (double) MotoGetSpeed();
		//订单状态
		BF_SET(cabstatus.state, (uint8_t)m_orderstate, 0, 4);
		//是否运动
		BF_SET(cabstatus.state, (MotoGetRpm()==0 ? 1 : 0),4,1);
		//是否在站点
		BF_SET(cabstatus.state, CarHsmIsInStation(),5,1);
		//时间
		cabstatus.ts = 0 ;
		PacketBuildCabStatus(&sendPacket, SESSIONID, CELL_ADDR_TESTCAR,CELL_ADDR_VRC_BACKUP,cabstatus);
        CellSendPacket(&sendPacket);
	}

}

/*
 * 接收线程
 */
static Void taskCellRecv(UArg a0, UArg a1)
{
	cell_packet_t recvPacket, sendPacket;
	packet_routenode_t * v_route;


	while(1)
	{

		CellRecvPacket(&recvPacket, BIOS_WAIT_FOREVER);

		/*
		 * 对type == CELL_TYPE_REQ的packet做出回应，不改变状态
		 */
		if (CellPacketGetType(&recvPacket) == CELL_TYPE_REQ)
		{
			switch(recvPacket.cmd)
			{
				case CELL_CMD_ORDERCAB:
				{
					CellHsmPost(cell_ordercab);
					if(m_orderstate != freestate)
						CellPacketBuildResponse(&recvPacket,&sendPacket,0,NULL,0);
					else
						CellPacketBuildResponse(&recvPacket,&sendPacket,1,NULL,0);

					CellSendPacket(&sendPacket);
					break;
				}

				case CELL_CMD_UPDATEROUTE:
				{

					if(m_orderstate == ordered ||
					   m_orderstate == shipping)
					{
						v_route = PacketResolveUpdateRoute(&recvPacket);
						RouteUpdate(v_route);

						/*
						 * 判断如果车已经在路径终点，则不启动寻路流程
						 */
						if(RouteGetDestination().nid == RFIDGetNID())
						{
							CellHsmPost(carhsm_arrived);
						}
						else
						{
							Message_postEvent(cell,CELL_MSG_ENTERAUTOMODE);
							Message_postEvent(cell,CELL_MSG_STARTRUN);
						}


						CellPacketBuildResponse(&recvPacket,&sendPacket,1,NULL,0);
					}
					else
					{
						CellPacketBuildResponse(&recvPacket, &sendPacket, 0, NULL, 0);
					}
					CellSendPacket(&sendPacket);
					break;
				}
			}
		}
	}
}

/*
 * 维护一个车辆预定状态机
 */
static Void taskCellHsm(UArg a0, UArg a1)
{
	cellhsm_msg_t msg;
	cellhsm_msg_type_t type;
	orderstate_t laststate = m_orderstate;
	packet_cabstatechange_t cabstatechange;
	epc_t myepc;
	cell_packet_t sendPacket;

	while(1)
	{
		if(Mailbox_pend(m_cellMbox,(Ptr *) &msg, 100))
		{
			type = msg.type;
		}
		else
		{
			type = cell_msg_tick;
		}

		if(force_to_freestate == type)
		{
			m_orderstate = freestate;
		}

		/*
		 * 状态机
		 */
		switch(m_orderstate)
		{
			case freestate:
			{
				if(cell_ordercab == type)
				{
					m_orderstate = ordered;
				}
				break;
			}

			case ordered:
			{
				if(carhsm_arrived == type)  //收到车辆到达目的地
				{
					m_orderstate = arrived_start;
				}
				break;
			}

			case arrived_start:
			{
				m_orderstate = loading;
				break;
			}

			case loading:
			{
				m_orderstate = shipping;
				break;
			}

			case shipping:
			{
				if(carhsm_arrived == type)  //收到车辆到达目的地
				{
					m_orderstate = arrived_end;
				}
				break;
			}

			case arrived_end:
			{
				m_orderstate = unloading;
				break;
			}

			case unloading:
			{
				m_orderstate = freestate;
				break;
			}


		}  //	switch(state)

		/*
		 * 订单状态改变
		 * 发送cabstatechange报文
		 */
		if(m_orderstate != laststate)
		{
			LogMsg("CellHsm state:");
			LogMsg("%s->%s\n",stateStrings[laststate],stateStrings[m_orderstate]);
			myepc = RFIDGetEpc();
			cabstatechange.nid = EPCgetShortID(&myepc);
			BF_SET(cabstatechange.cs,m_orderstate,0,4);
			BF_SET(cabstatechange.ps,laststate,0,4);

			PacketBuildCabStateChange(&sendPacket,SESSIONID, CELL_ADDR_TESTCAR,CELL_ADDR_VRC_BACKUP,cabstatechange);
			CellSendPacket(&sendPacket);

		}
		laststate = m_orderstate;

		g_fbData.orderState = m_orderstate;
	}
}

void CellCommunicationInit()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;

	CellDriverInit();

	m_cellMbox = Mailbox_create (sizeof (cellhsm_msg_t),CELL_HSM_MAILBOX_DEPTH, NULL, NULL);

	Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 4096;
	task = Task_create(taskCellRecv, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	task = Task_create(taskCellSend, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	task = Task_create(taskCellHsm, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

}

void CellHsmPost(cellhsm_msg_type_t type)
{
	cellhsm_msg_t msg;

	if(m_cellMbox != 0)
	{
		msg.type = type;
		Mailbox_post(m_cellMbox, (Ptr *)&msg, BIOS_NO_WAIT);
	}
}
