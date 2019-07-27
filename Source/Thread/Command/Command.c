/*
 * Command.c
 *
 *  Created on: 2019-7-4
 *      Author: zhtro
 */


#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>
#include "uartns550.h"
#include "Command/CommandDriver.h"
#include "Sensor/RFID/EPCdef.h"
#include "Lib/vector.h"
#include "Moto/task_moto.h"
#include "CellCommunication/CellCommunication.h"
#include "Message/Message.h"
#include "Decision/route/Route.h"



static void ShowRFIDPointList(rfidPoint_t * list)
{
	LogMsg("\n==========\n size=%d\n",vector_size(list));

	if(vector_size(list) != 0)
	{
		size_t i;
		for(i=0; i<vector_size(list); i++)
		{
			LogMsg("pos:0x%02x%02x%02x\t isPath:%d\t flag:0x%02x\n",
					list[i].byte[8],
					list[i].byte[9],
					list[i].byte[10],
					list[i].byte[12],
					list[i].byte[13]);

		}

	}
	else
	{
		LogMsg("EMPTY\n");
	}
	LogMsg("============\n");
}

static void CommandHandleTask(UArg arg0, UArg arg1)
{
	command_packet_t packet;
	rfidPoint_t * vmap = 0;
	packet_routenode_t * vroute = 0;
	rfidPoint_t * vcalib = 0;
	rfidPoint_t  rfidPoint;

	packet_routenode_t routePoint;
	epc_t routeEPC;

	vector_grow(vmap, 256);
	vector_grow(vcalib, 256);
	vector_grow(vroute, 256);



	while(1)
	{
		CommandRecv(&packet, BIOS_WAIT_FOREVER);

		switch(packet.type)
		{
			case COMMAND_TYPE_ROUTE_START:
			{
				LogMsg("COMMAND_TYPE_ROUTE_START\n");

				vector_set_size(vmap, 0);
				vector_set_size(vroute, 0);

				break;
			}
			case COMMAND_TYPE_ROUTE_NODE:
			{
				memcpy(rfidPoint.byte, packet.data,RFID_POINT_LEN);
				vector_push_back(vmap, rfidPoint);

				if(rfidPoint.byte[12] == 1) //是路径点
				{
					EPCfromByteArray(&routeEPC, rfidPoint.byte);
					routePoint.nid = EPCgetShortID(&routeEPC);
					routePoint.length = 0;
					routePoint.speedlimit = 0;
					routePoint.flag = rfidPoint.byte[13];
					vector_push_back(vroute , routePoint);
				}
				break;
			}

			case COMMAND_TYPE_ROUTE_END:
			{
				uint16_t routeLen;
				routeLen = packet.data[0] * 256 + packet.data[1];

				if(routeLen ==  vector_size(vmap))
				{
					CommandSend("route_recved");
					RFIDUpdateQueue(vmap);
					LogMsg("map List\n");
					ShowRFIDPointList(vmap);

					RouteUpdateCopy(vroute);
				}
				else
				{
					LogMsg("Update map error recved :%d should be %d\n",vector_size(vmap),routeLen );
				}

				break;
			}

			case COMMAND_TYPE_CALIB_START:
			{
				LogMsg("COMMAND_TYPE_CALIB_START\n");

				vector_set_size(vcalib, 0);

				break;
			}

			case COMMAND_TYPE_CALIB_NODE:
			{
				memcpy(rfidPoint.byte, packet.data,RFID_POINT_LEN);
				vector_push_back(vcalib, rfidPoint);
				break;
			}

			case COMMAND_TYPE_CALIB_END:
			{
				uint16_t calibLen;
				calibLen = packet.data[0] * 256 + packet.data[1];

				if(calibLen ==  vector_size(vcalib))
				{
					CommandSend("calib_recved");
					MotoUpdateCalibrationPoint(vcalib);
					LogMsg("calib List\n");
					ShowRFIDPointList(vcalib);
				}
				else
				{
					LogMsg("Update calib error recved :%d should be %d\n",vector_size(vcalib),calibLen );
				}

				break;
			}

			case COMMAND_TYPE_GO:
			{
				CommandSend("GO_recved");
				LogMsg("COMMMAD: go\n");
				Message_postEvent(cell,CELL_MSG_ENTERAUTOMODE);
				Message_postEvent(cell,CELL_MSG_STARTRUN);

				break;
			}

			case COMMAND_TYPE_SET_START_POINT:
			{
				uint32_t startDis;


				startDis = (packet.data[0]<<24) \
						+  (packet.data[1]<<16) \
						+  (packet.data[2]<<8) \
						+  (packet.data[3]);

				LogMsg("COMMAND_TYPE_SET_START_POINT: %d\n",startDis);

				MotoSetCarDistance(startDis);

				break;
			}
			default:break;
		}
	}
}


/*
 * API
 */

void CommandInit()
{
	Task_Handle task;
	Task_Params taskParams;


	CommandDriverInit();

	//task
	Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(CommandHandleTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}
}
