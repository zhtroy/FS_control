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
#include "Lib/bitop.h"
#include "Moto/task_moto.h"
#include "Moto/task_brake_servo.h"
#include "Sensor/RFID/RFID_task.h"
#include "Thread/Moto/Parameter.h"



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

static void CommandSendTask(UArg arg0, UArg arg1)
{
	const int SEND_INTERVAL = 500;
	int packetLen = 0;
	char packetBuff[256];

	uint32_t distance;
	float speed;
	uint8_t railstate;
	uint8_t errorcode;
	uint8_t throttle;
	uint8_t brake;
	uint8_t mode;
	uint8_t * rfidArray;

	while(1)
	{
		Task_sleep(SEND_INTERVAL);

		packetLen = 0;
		//车辆位置
		distance = htonl(MotoGetCarDistance());
		memcpy(&packetBuff[packetLen],&distance,sizeof(uint32_t));
		packetLen += sizeof(uint32_t);

		//车辆速度
		speed = htonf(SpeedfromRPM(MotoGetRealRPM()) * 3.6f );
		memcpy(&packetBuff[packetLen],&speed,sizeof(float));
		packetLen += sizeof(float);

		//轨道状态
		railstate = RailGetRailState();
		memcpy(&packetBuff[packetLen],&railstate,sizeof(uint8_t));
		packetLen += sizeof(uint8_t);

		//当前RFID
		rfidArray = RFIDGetRaw();
		memcpy(&packetBuff[packetLen],rfidArray,EPC_SIZE);
		packetLen += EPC_SIZE;

		//故障代码
		errorcode = g_fbData.ErrorCode;
		memcpy(&packetBuff[packetLen],&errorcode, sizeof(uint8_t));
		packetLen += sizeof(uint8_t);

		//油门
		throttle = MotoGetThrottle();
		memcpy(&packetBuff[packetLen],&throttle, sizeof(uint8_t));
		packetLen += sizeof(uint8_t);

		//刹车
		brake = BrakeGetBrake();
		memcpy(&packetBuff[packetLen],&brake, sizeof(uint8_t));
		packetLen += sizeof(uint8_t);

		//模式
		mode = g_fbData.mode;
		memcpy(&packetBuff[packetLen],&mode, sizeof(uint8_t));
		packetLen += sizeof(uint8_t);

		CommandSend(packetBuff, packetLen, COMMAND_TYPE_CAR_INFO );

	}
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
	uint8_t routeReady = 0;   //路径是否准备好

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
				routeReady = 0;

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
				uint8_t success;
				routeLen = packet.data[0] * 256 + packet.data[1];
				uint8_t * rawRFID =0;
				epc_t vmapFirstEPC;
				int vmapFirstPosDiff;    //vmap第一个点与当前位置的差值

				if(routeLen ==  vector_size(vmap))
				{
					rawRFID = RFIDGetRaw();

					EPCfromByteArray(&vmapFirstEPC, vmap[0].byte);

					vmapFirstPosDiff = (int) vmapFirstEPC.distance - (int) MotoGetCarDistance();

					if(vmapFirstPosDiff <0)
					{
						vmapFirstPosDiff += TOTAL_DISTANCE;
					}

					//如果路线中第一个点在当前点前方36m处，认为路线正确，否则都认为发错路线
					if( vmapFirstPosDiff>=0 && vmapFirstPosDiff< 360)
					{
						success = 1;

						RFIDUpdateQueue(vmap);
						LogMsg("map List\n");
						ShowRFIDPointList(vmap);

						RouteUpdateCopy(vroute);

						routeReady = 1;
					}

					else
					{
						success = 0;
					}


				}
				else
				{
					success = 0;
					LogMsg("Update map error recved :%d should be %d\n",vector_size(vmap),routeLen );
				}

				CommandSend(&success, sizeof(uint8_t), COMMAND_TYPE_ROUTE_RESPONSE);
				break;
			}

			case COMMAND_TYPE_CHANGE_ROUTE_END:
			{
                uint16_t routeLen;
                uint8_t success;
                routeLen = packet.data[0] * 256 + packet.data[1];

                if(routeLen ==  vector_size(vmap))
                {

                    success = RFIDAppendQueue(vmap);

                    if(success)
                    {
                    	routeReady = 1;
                        LogMsg("append List\n");
                        ShowRFIDPointList(vmap);
                        RouteUpdateCopy(vroute);
                    }
                }
                else
                {
                    success = 0;
                    LogMsg("Update map error recved :%d should be %d\n",vector_size(vmap),routeLen );
                }
			    CommandSend(&success, sizeof(uint8_t), COMMAND_TYPE_CHANGE_ROUTE_RESPONSE);
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
				uint8_t success;
				calibLen = packet.data[0] * 256 + packet.data[1];

				if(calibLen ==  vector_size(vcalib))
				{
					success = 1;
					MotoUpdateCalibrationPoint(vcalib);
					LogMsg("calib List\n");
					ShowRFIDPointList(vcalib);
				}
				else
				{
					success = 0;
					LogMsg("Update calib error recved :%d should be %d\n",vector_size(vcalib),calibLen );
				}

				CommandSend(&success, sizeof(uint8_t), COMMAND_TYPE_CALIB_RESPONSE);

				break;
			}

			case COMMAND_TYPE_GO:
			{
				uint8_t success = 1;

				if(routeReady)
				{
					routeReady = 0;
					success = 1;
					Message_postEvent(cell,CELL_MSG_ENTERAUTOMODE);
					Message_postEvent(cell,CELL_MSG_STARTRUN);
				}
				else
				{
					success = 0;
				}

				LogMsg("COMMMAD: GO\t%d\n",success);

				CommandSend(&success, sizeof(uint8_t), COMMAND_TYPE_GO_RESPONSE);

				break;
			}

			case COMMAND_TYPE_SET_START_POINT:
			{
				uint32_t startDis;
				uint8_t success = 1;


				startDis = (packet.data[0]<<24) \
						+  (packet.data[1]<<16) \
						+  (packet.data[2]<<8) \
						+  (packet.data[3]);

				LogMsg("COMMAND_TYPE_SET_START_POINT: %d\n",startDis);

				MotoSetCarDistance(startDis);

				CommandSend(&success, sizeof(uint8_t), COMMAND_TYPE_SET_START_POINT_RESPONSE);


				break;
			}

			case COMMAND_TYPE_SET_RFID:
			{
				uint8_t success = 1;

				RFIDSetRaw(packet.data);

				MotoSetCarDistance(RFIDGetEpc().distance);

				CommandSend(&success, sizeof(uint8_t), COMMAND_TYPE_SET_RFID_RESPONSE);

				LogMsg("COMMAND_TYPE_SET_RFID\n");

				break;
			}

			case COMMAND_TYPE_SETLOOP:
			{
				uint8_t success = 1;

				g_param.cycleRoute = packet.data[0];
				CommandSend(&success, sizeof(uint8_t), COMMAND_TYPE_SETLOOP_RESPONSE);
				break;
			}

			case COMMAND_TYPE_DOOR:
			{
				uint8_t success = 0;

				CommandSend(&success, sizeof(uint8_t), COMMAND_TYPE_DOOR_RESPONSE);
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

	task = Task_create(CommandSendTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}
}
