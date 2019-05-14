/*
 * CellSession.c
 *
 * 负责从串口接收网络数据包，并维护一个session list
 *
 *  Created on: 2019-5-13
 *      Author: zhtro
 */


#include <xdc/std.h>
#include "Sensor/CellCommunication/CellDriver.h"
#include <ti/sysbios/BIOS.h>

static Void taskSessionManage(UArg a0, UArg a1)
{
	cell_packet_t packet;
	//初始化
	CellDriverInit();

	while(1)
	{
		CellRecvPacket(&packet, BIOS_WAIT_FOREVER);

		switch(CellPacketGetType(&packet))
		{
			case CELL_TYPE_NONE:
			{

			}
			case CELL_TYPE_REQ:
			{

			}
			case CELL_TYPE_ALLOW:
			case CELL_TYPE_DENY:
			{

			}
		}
	}
}
