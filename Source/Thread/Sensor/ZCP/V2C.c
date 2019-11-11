/*
 * v2c.c
 *
 * 车辆到区域控制器的通信
 *
 *  Created on: 2019-6-3
 *      Author: zhtro
 */

#include "ZCP/zcp_driver.h"
#include "ZCP/V2V.h"
#include "Lib/bitop.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include "ZCP/V2C.h"
#include "Moto/task_brake_servo.h"
#include "Sensor/RFID/RFID_task.h"
#include "Sensor/RFID/EPCdef.h"
#include "Message/Message.h"
#include "Moto/task_moto.h"
#include "CarState.h"
#include "route/Route.h"
#include "Lib/bitop.h"



#define V2C_ZCP_UART_DEV_NUM    (2)
#define V2C_ZCP_DEV_NUM (1)
/*
 * 确保接收task优先级比较低
 */
#define V2C_RECV_TASK_PRIO 		(5)
#define V2C_SESSION_TASK_PRIO   (V2C_RECV_TASK_PRIO + 1)

#define TIMEOUT (200)  //发送间隔
#define RETRY_NUM (10)  //重发次数

static ZCPInstance_t v2cInst;

static uint8_t m_openOrCloseDoor = 0;

/*
 * sems
 */
static Semaphore_Handle m_sem_askfrontid = NULL;
static Semaphore_Handle m_sem_enterstation = NULL;
static Semaphore_Handle m_sem_leavestation = NULL;
static Semaphore_Handle m_sem_openDoor = NULL;

/*
 * mailbox
 */
static Mailbox_Handle m_mb_forcarid;
static Mailbox_Handle m_mb_enterstation;
static Mailbox_Handle m_mb_opendoor;
static Mailbox_Handle m_mb_leavestation;



/*
 * tasks
 */
static Task_Handle m_task_askForwardId = NULL;
static Task_Handle m_task_enterStation = NULL;

static uint8_t m_isAskingFrontCar = 0;

/*
 * 启动一个Task，如果task正在运行，则不启动Task
 */
void StartIfNotRunning(Task_Handle * ptask,Task_FuncPtr func, int pri )
{
	Task_Params taskParams;

	//保证同一时刻只有一个task在运行
	if(*ptask != NULL && Task_getMode(*ptask) != Task_Mode_TERMINATED)
	{
		return;
	}

	if(*ptask != NULL && Task_getMode(*ptask) == Task_Mode_TERMINATED )
	{
		Task_delete(ptask);
	}

	Task_Params_init(&taskParams);
	taskParams.priority = pri;
	*ptask = Task_create(func, &taskParams, NULL);

}

static void V2CRecvTask(UArg arg0, UArg arg1)
{
    int32_t timestamp;
    ZCPUserPacket_t recvPacket;

	while(1)
	{
		ZCPRecvPacket(&v2cInst, &recvPacket, &timestamp, BIOS_WAIT_FOREVER);

		switch(recvPacket.type)
		{
			case ZCP_TYPE_V2C_RESP_STATION_FRONTCARID:
			{
				Mailbox_post(m_mb_forcarid,(Ptr)&recvPacket,BIOS_WAIT_FOREVER);
				break;
			}
			case ZCP_TYPE_V2C_RESP_STATION_ENTERSTATION:
			{
				Mailbox_post(m_mb_enterstation,(Ptr)&recvPacket,BIOS_WAIT_FOREVER);
				break;
			}
			case ZCP_TYPE_V2C_RESP_STATION_DOOR:
			{
				Mailbox_post(m_mb_opendoor,(Ptr)&recvPacket,BIOS_WAIT_FOREVER);
				break;
			}
			case ZCP_TYPE_V2C_RESP_STATION_LEAVESTATION:
			{
				Mailbox_post(m_mb_leavestation,(Ptr)&recvPacket,BIOS_WAIT_FOREVER);
				break;
			}
		}
	}
}

static void V2CSendTask(UArg arg0, UArg arg1)
{
	ZCPUserPacket_t sendPacket;
	v2c_req_carstatus_t carstatus;

	const int INTERVAL = 100;
	while(1)
	{
		Task_sleep(INTERVAL);

		//如果正在请求前车，则不发送状态
		if(m_isAskingFrontCar)
		{
			continue;
		}
		/*
		 * send status
		 */
		carstatus.rpm = MotoGetRealRPM();
		if(MotoGetCarMode() == ForceBrake)
		{
			carstatus.status = 2;
		}
		else
		{
			if(MotoGetRealRPM() == 0 && g_fbData.FSMstate == idle && MotoGetCarMode() == Auto )
				carstatus.status = 0;
			else
				carstatus.status = 1;
		}
		carstatus.distance = MotoGetCarDistance();
		carstatus.carmode = MotoGetCarMode();
		carstatus.railstate = RailGetRailState();
		memcpy(carstatus.rfid, RFIDGetRaw(), EPC_SIZE);
		memcpy(sendPacket.data, &carstatus, sizeof(carstatus));
		sendPacket.addr = g_sysParam.station_addr;
		sendPacket.type = ZCP_TYPE_V2C_REQ_CAR_CARSTATUS;
		sendPacket.len = sizeof(carstatus);

		ZCPSendPacket(&v2cInst, &sendPacket, NULL, BIOS_NO_WAIT);
	}
}

static void V2COpenDoorTask(UArg arg0, UArg arg1)
{
    ZCPUserPacket_t sendPacket, recvPacket;
    v2c_resp_opendoor_t resp;
    Bool pendResult;
    int retryNum;


    while(1)
    {
    	Semaphore_pend(m_sem_openDoor,BIOS_WAIT_FOREVER);

		retryNum = RETRY_NUM;
		do
		{
			/*
			 * 构造请求报文
			 */
			sendPacket.addr = g_sysParam.station_addr;
			sendPacket.type = ZCP_TYPE_V2C_REQ_CAR_DOOR;
			sendPacket.data[0] = m_openOrCloseDoor;
			sendPacket.len = 1;

			/*
			 * 发送请求
			 */
			ZCPSendPacket(&v2cInst, &sendPacket, NULL, BIOS_NO_WAIT);

			pendResult = Mailbox_pend(m_mb_opendoor, (Ptr)&recvPacket, TIMEOUT);
			retryNum --;
		}
		while(retryNum>0 && pendResult==FALSE);

		if(pendResult == FALSE) //发送后没收到响应
		{
//			Message_postError(ERROR_V2C);

		}
		else
		{
			memcpy(&resp, recvPacket.data, sizeof(resp));
			if(resp.status == 0)  //成功
			{
				//TODO
			}
			else
			{

			}
		}

    }
}

static void V2CAskFrontIdTask(UArg arg0, UArg arg1)
{
    ZCPUserPacket_t sendPacket, recvPacket;
    v2c_req_forward_id_t req;
    v2c_resp_forward_id_t resp;
    Bool pendResult;
    int retryNum;
    int i;

    while(1)
    {
    	Semaphore_pend(m_sem_askfrontid,BIOS_WAIT_FOREVER);

		/*
		 * 先清除邮箱
		 */
		do
		{
			Mailbox_pend(m_mb_forcarid, (Ptr)&recvPacket, BIOS_NO_WAIT);
		}
		while(Mailbox_getNumPendingMsgs(m_mb_forcarid)!=0 );

		retryNum = RETRY_NUM;

		m_isAskingFrontCar = 1;
		do
		{
			/*
			 * 构造请求报文
			 */

			req.carmode = MotoGetCarMode();
			req.railpos = RailGetRailState();
			req.distance = RFIDGetEpc().distance;
			memcpy(req.rfid, RFIDGetRaw(), EPC_SIZE);
			memcpy(sendPacket.data, &req, sizeof(req));
			sendPacket.addr = g_sysParam.station_addr;
			sendPacket.type = ZCP_TYPE_V2C_REQ_CAR_FRONTCARID;
			sendPacket.len = sizeof(req);


			/*
			 * 发送请求
			 */
			ZCPSendPacket(&v2cInst, &sendPacket, NULL, BIOS_NO_WAIT);

			pendResult = Mailbox_pend(m_mb_forcarid, (Ptr)&recvPacket, TIMEOUT);
			retryNum --;
		}
		while(retryNum>0 && pendResult==FALSE);

		//请求结束
		m_isAskingFrontCar = 0;

		if(pendResult == FALSE) //发送后没收到响应
		{
			Message_postError(ERROR_V2C);

		}
		else
		{
			memcpy(&resp, recvPacket.data, sizeof(resp));
			if(resp.status == 1)  //有前车
			{
				if(V2VGetFrontCarId() != resp.forwardid)   //只有在申请到的前车ID变化时才改变ID
				{

					//设置新的前车ID
					V2VSetFrontCarId(resp.forwardid);
					V2VSetLeftRoadID(resp.leftRoadID);
				}
				V2VHandShakeFrontCar();
			}
			else
			{
				V2VSetFrontCarId(V2V_ID_NONE);
			}
		}

    }
}

static void V2CEnterStationTask(UArg arg0, UArg arg1)
{
    ZCPUserPacket_t sendPacket, recvPacket;
    v2c_req_enterstation_t req;
    v2c_resp_enterstation_t resp;
    Bool pendResult;
    int retryNum;
    packet_routenode_t routeNode;
    int i;
    epc_t epc;

    while(1)
    {
    	Semaphore_pend(m_sem_enterstation,BIOS_WAIT_FOREVER);


		/*
		 * 先清除邮箱
		 */
		do
		{
			Mailbox_pend(m_mb_enterstation, (Ptr)&recvPacket, BIOS_NO_WAIT);
		}
		while(Mailbox_getNumPendingMsgs(m_mb_enterstation)!=0 );


		retryNum = RETRY_NUM;
		do
		{
			/*
			 * 构造请求报文
			 */
			//如果没有路径，不发送请求
			if(!RouteHasOngoing())
				return;
			routeNode = RouteGetDestination();
			req.roadId[0] = BF_GET(routeNode.nid,56,8);
			req.roadId[1] = BF_GET(routeNode.nid,48,8);
			req.roadId[2] = BF_GET(routeNode.nid,40,8);
			req.roadId[3] = BF_GET(routeNode.nid,32,8);
			req.roadId[4] = BF_GET(routeNode.nid,24,8);
			memcpy(sendPacket.data, &req, sizeof(req));
			sendPacket.addr = g_sysParam.station_addr;
			sendPacket.type = ZCP_TYPE_V2C_REQ_CAR_ENTERSTATION;
			sendPacket.len = sizeof(req);


			/*
			 * 发送请求
			 */
			ZCPSendPacket(&v2cInst, &sendPacket, NULL, BIOS_NO_WAIT);

			pendResult = Mailbox_pend(m_mb_enterstation, (Ptr)&recvPacket, TIMEOUT);
			retryNum --;
		}
		while(retryNum>0 && pendResult==FALSE);

		if(pendResult == FALSE) //发送后没收到响应
		{
//			Message_postError(ERROR_V2C);

		}
		else
		{
			memcpy(&resp, recvPacket.data, sizeof(resp));
			if(resp.status == 1)  //分配了停站点
			{
				//更新路径终点
				routeNode = RouteGetDestination();

				EPCfromByteArray(&epc, resp.rfid);
				routeNode.nid = EPCgetShortID(&epc);

				RouteChangeDestination(routeNode);
			}
			else
			{
				//do nothing
			}
		}

    }

}

static void V2CLeaveStationTask(UArg arg0, UArg arg1)
{
    ZCPUserPacket_t sendPacket, recvPacket;
    v2c_req_leavestation_t req;
    Bool pendResult;
    int retryNum;
    packet_routenode_t routeNode;
    int i;

    while(1)
    {
    	Semaphore_pend(m_sem_leavestation,BIOS_WAIT_FOREVER);

		/*
		 * 先清除邮箱
		 */
		do
		{
			Mailbox_pend(m_mb_leavestation, (Ptr)&recvPacket, BIOS_NO_WAIT);
		}
		while(Mailbox_getNumPendingMsgs(m_mb_leavestation)!=0 );

		retryNum = RETRY_NUM;
		do
		{
			/*
			 * 构造请求报文
			 */
			//如果没有路径，不发送请求
			if(!RouteHasOngoing())
				return;
			routeNode = RouteGetDestination();
			req.roadId[0] = BF_GET(routeNode.nid,56,8);
			req.roadId[1] = BF_GET(routeNode.nid,48,8);
			req.roadId[2] = BF_GET(routeNode.nid,40,8);
			req.roadId[3] = BF_GET(routeNode.nid,32,8);
			req.roadId[4] = BF_GET(routeNode.nid,24,8);
			memcpy(sendPacket.data, &req, sizeof(req));
			sendPacket.addr = g_sysParam.station_addr;
			sendPacket.type = ZCP_TYPE_V2C_REQ_CAR_LEAVESTATION;
			sendPacket.len = sizeof(req);


			/*
			 * 发送请求
			 */
			ZCPSendPacket(&v2cInst, &sendPacket, NULL, BIOS_NO_WAIT);

			pendResult = Mailbox_pend(m_mb_leavestation, (Ptr)&recvPacket, TIMEOUT);
			retryNum --;
		}
		while(retryNum>0 && pendResult==FALSE);

		if(pendResult == FALSE) //发送后没收到响应
		{
//			Message_postError(ERROR_V2C);

		}
		else
		{
			//TODO
		}

    }

}
/*
 * API
 */



void V2CStartUpTask()
{
    Task_Handle task;
    Task_Params taskParams;
	Semaphore_Params semParams;

	ZCPInit(&v2cInst, V2C_ZCP_DEV_NUM,V2C_ZCP_UART_DEV_NUM ,  g_sysParam.carID );


	/*
	 * 邮箱
	 */
	m_mb_forcarid = Mailbox_create(sizeof(ZCPUserPacket_t),4,NULL,NULL);
	m_mb_enterstation = Mailbox_create(sizeof(ZCPUserPacket_t),4,NULL,NULL);
	m_mb_opendoor= Mailbox_create(sizeof(ZCPUserPacket_t),4,NULL,NULL);
	m_mb_leavestation = Mailbox_create(sizeof(ZCPUserPacket_t),4,NULL,NULL);

	/*
	 * sem
	 */
	Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    m_sem_askfrontid = Semaphore_create(0, &semParams, NULL);
    m_sem_enterstation = Semaphore_create(0, &semParams, NULL);
    m_sem_openDoor= Semaphore_create(0, &semParams, NULL);
	m_sem_leavestation =  Semaphore_create(0, &semParams, NULL);


    Task_Params_init(&taskParams);

    taskParams.stackSize = 2048;
    taskParams.priority = V2C_RECV_TASK_PRIO;

	task = Task_create((Task_FuncPtr)V2CRecvTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	task = Task_create((Task_FuncPtr)V2CSendTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	taskParams.priority = V2C_SESSION_TASK_PRIO;
	task = Task_create((Task_FuncPtr)V2CAskFrontIdTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	task = Task_create((Task_FuncPtr)V2CEnterStationTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	task = Task_create((Task_FuncPtr)V2CLeaveStationTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	task = Task_create((Task_FuncPtr)V2COpenDoorTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}



}

void V2CInit()
{
	Task_Handle task;
	Task_Params taskParams;

	Task_Params_init(&taskParams);
	taskParams.stackSize = 2048;
	taskParams.priority = 5;

	task = Task_create((Task_FuncPtr)V2CStartUpTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}
}

/*
 * 向站台申请前车ID
 */
void V2CAskFrontID()
{
	Semaphore_post(m_sem_askfrontid);
}

/*
 * 向站台申请进站
 */
void V2CEnterStation()
{
	Semaphore_post(m_sem_enterstation);
}

/*
 * 申请出站
 */
void V2CLeaveStation()
{
	Semaphore_post(m_sem_leavestation);
}

/*
 * 开关屏蔽门申请
 * 传入1表示打开，0表示关闭
 */
void V2COpenDoor(uint8_t openOrClose)
{
	m_openOrCloseDoor = openOrClose;
	Semaphore_post(m_sem_openDoor);
}
