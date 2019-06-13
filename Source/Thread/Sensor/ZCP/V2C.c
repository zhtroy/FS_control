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
#include "Message/Message.h"
#include "Moto/task_moto.h"
#include "Moto/Parameter.h"



#define V2C_ZCP_UART_DEV_NUM    (6)
#define V2C_ZCP_DEV_NUM (1)
/*
 * 确保接收task优先级比较低
 */
#define V2C_RECV_TASK_PRIO 		(5)
#define V2C_SESSION_TASK_PRIO   (V2C_RECV_TASK_PRIO + 1)

static ZCPInstance_t v2cInst;

/*
 * sems
 */

/*
 * mailbox
 */
static Mailbox_Handle m_mb_forcarid;

/*
 * tasks
 */
static Task_Handle m_task_askForwardId = NULL;


/*
 * 启动一个Task，如果task正在运行，则不启动Task
 */
static void StartIfNotRunning(Task_Handle task,Task_FuncPtr func, int pri )
{
	Task_Params taskParams;

	//保证同一时刻只有一个task在运行
	if(task != NULL && Task_getMode(task) != Task_Mode_TERMINATED)
	{
		return;
	}

	if(task != NULL && Task_getMode(task) == Task_Mode_TERMINATED )
	{
		Task_delete(task);
	}

	Task_Params_init(&taskParams);
	taskParams.priority = pri;
	task = Task_create(func, &taskParams, NULL);

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
				break;
			}
			case ZCP_TYPE_V2C_RESP_STATION_DOOR:
			{
				break;
			}
			case ZCP_TYPE_V2C_RESP_STATION_LEAVESTATION:
			{
				break;
			}
		}
	}
}

static void V2CAskFrontIdTask(UArg arg0, UArg arg1)
{
	const uint32_t TIMEOUT = 1000;
	const int RETRY_NUM = 2;
    ZCPUserPacket_t sendPacket, recvPacket;
    v2c_req_forward_id_t req;
    v2c_resp_forward_id_t resp;
    Bool pendResult;
    int retryNum;



    retryNum = RETRY_NUM;
	do
	{
		/*
		 * 构造请求报文
		 */

		req.railpos = RailGetRailState();
		memcpy(req.rfid, RFIDGetRaw(), EPC_SIZE);
		memcpy(sendPacket.data, &req, sizeof(req));
		sendPacket.addr = g_param.station_addr;
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

	if(pendResult == FALSE) //发送后没收到响应
	{
		Message_postError(ERROR_V2C);

	}
	else
	{
		memcpy(&resp, recvPacket.data, sizeof(resp));
		if(resp.status == 1)  //有前车
		{
			V2VHandShakeFrontCar(resp.forwardid);
		}
	}

	return;
}

/*
 * API
 */

void V2CInit()
{
    Task_Handle task;
    Task_Params taskParams;

	ZCPInit(&v2cInst, V2C_ZCP_DEV_NUM,V2C_ZCP_UART_DEV_NUM );

    Task_Params_init(&taskParams);

    taskParams.stackSize = 2048;
    taskParams.priority = V2C_RECV_TASK_PRIO;

	task = Task_create((Task_FuncPtr)V2CRecvTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	/*
	 * 邮箱
	 */
	m_mb_forcarid = Mailbox_create(sizeof(ZCPUserPacket_t),4,NULL,NULL);
}

/*
 * 向站台申请前车ID
 */
void V2CAskFrontID()
{
	StartIfNotRunning(m_task_askForwardId, V2CAskFrontIdTask,V2C_SESSION_TASK_PRIO);
}



