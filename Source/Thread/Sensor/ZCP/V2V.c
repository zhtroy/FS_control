/*
 * V2V.c
 * 车辆之间通信
 *  Created on: 2019-6-4
 *      Author: zhtro
 */

#include "ZCP/zcp_driver.h"
#include "ZCP/V2V.h"
#include "ZCP/V2C.h"
#include "Moto/task_moto.h"
#include "Message/Message.h"

#include "stdint.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include "Sensor/ZCP/V2C.h"
#include "Sensor/RFID/EPCdef.h"
#include "Decision/CarState.h"

#define V2V_ZCP_UART_DEV_NUM    (1)
#define V2V_ZCP_DEV_NUM (0)
/*
 * 确保接收task优先级比较低
 */
#define V2V_RECV_TASK_PRIO 		(5)
#define V2V_SESSION_TASK_PRIO   (V2V_RECV_TASK_PRIO + 1)

//通信参数
static v2v_param_t m_param = {
		.backId = V2V_ID_NONE,
		.frontId = V2V_ID_NONE
};
//前车状态
static v2v_req_carstatus_t frontCarStatus;
//ZCP驱动实例
static ZCPInstance_t v2vInst;

/*
 * task
 */
static Task_Handle m_task_handshakefrontcar = NULL;

/*
 * sems
 */
static Semaphore_Handle m_sem_handshakefrontcar_resp;
static Semaphore_Handle m_sem_handshakefrontcar_start;


static void V2VSendTask(UArg arg0, UArg arg1)
{

	const int INTERVAL = 100;

	v2v_req_carstatus_t carstatus;
	ZCPUserPacket_t sendPacket;

	while(1)
	{
		Task_sleep(INTERVAL);
		//发送carstatus报文给后车
		if(m_param.backId != V2V_ID_NONE)
		{
			carstatus.rpm = MotoGetRealRPM();
			if(MotoGetCarMode() == ForceBrake)
			{
				carstatus.status = 2;
			}
			else
			{
				if(MotoGetRealRPM() == 0)
					carstatus.status = 0;
				else
					carstatus.status = 1;
			}

			memcpy(carstatus.epc, RFIDGetRaw(), EPC_SIZE);
			carstatus.distance = MotoGetCarDistance();

			memcpy(sendPacket.data, &carstatus, sizeof(carstatus));
			sendPacket.addr = m_param.backId;
			sendPacket.type = ZCP_TYPE_V2V_REQ_FRONT_CARSTATUS;
			sendPacket.len = sizeof(carstatus);

			ZCPSendPacket(&v2vInst, &sendPacket, NULL, BIOS_NO_WAIT);
		}
	}
}

static void V2VRecvTask(UArg arg0, UArg arg1)
{
    int32_t timestamp;
    ZCPUserPacket_t recvPacket, sendPacket;

	while(1)
	{
		ZCPRecvPacket(&v2vInst, &recvPacket, &timestamp, BIOS_WAIT_FOREVER);

		switch(recvPacket.type)
		{
			case ZCP_TYPE_V2V_REQ_BACK_HANDSHAKE:
			{
				/*
				 * 更新后车RFID，
				 */
				m_param.backId = recvPacket.addr;

				/*
				 * 并回复一个响应报文给后车
				 */
				sendPacket.addr = m_param.backId;
				sendPacket.type = ZCP_TYPE_V2V_RESP_FRONT_HANDSHAKE;
				sendPacket.len = 0;
				ZCPSendPacket(&v2vInst, &sendPacket, NULL, BIOS_NO_WAIT);
				break;
			}

			case ZCP_TYPE_V2V_RESP_FRONT_HANDSHAKE:
			{
				Semaphore_post(m_sem_handshakefrontcar_resp);
				break;
			}

			case ZCP_TYPE_V2V_REQ_FRONT_CARSTATUS:
			{
				//如果收到的包不是前车发来的，不处理
				if(recvPacket.addr != m_param.frontId)
				{
					break;
				}
				memcpy(&frontCarStatus, recvPacket.data, sizeof(frontCarStatus));
				break;
			}
		}
	}
}

static void V2VHandShakeFrontCarTask(UArg arg0, UArg arg1)
{
	const uint32_t TIMEOUT = 1000;
	const int RETRY_NUM = 2;

	ZCPUserPacket_t sendPacket;
    Bool pendResult;
    int retryNum;

    while(1)
    {
    	Semaphore_pend(m_sem_handshakefrontcar_start,BIOS_WAIT_FOREVER);

		retryNum = RETRY_NUM;
		do
		{
			/*
			 * 构造请求报文
			 */

			sendPacket.addr = m_param.frontId;
			sendPacket.type = ZCP_TYPE_V2V_REQ_BACK_HANDSHAKE;
			sendPacket.len = 0;

			/*
			 * 发送请求
			 */
			ZCPSendPacket(&v2vInst, &sendPacket, NULL, BIOS_NO_WAIT);

			pendResult = Semaphore_pend(m_sem_handshakefrontcar_resp, TIMEOUT);
			retryNum --;
		}
		while(retryNum>0 && pendResult==FALSE);

		if(pendResult == FALSE)
		{
			Message_postError(ERROR_V2C);
		}
    }


}

/*
 * API
 */

void V2VInit()
{
	Task_Handle task;
	Task_Params taskParams;
	Semaphore_Params semParams;

	ZCPInit(&v2vInst, V2V_ZCP_DEV_NUM,V2V_ZCP_UART_DEV_NUM );

	Task_Params_init(&taskParams);

	taskParams.stackSize = 2048;
	taskParams.priority = V2V_RECV_TASK_PRIO;

	task = Task_create((Task_FuncPtr)V2VRecvTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	taskParams.priority = V2V_SESSION_TASK_PRIO;
	task = Task_create((Task_FuncPtr)V2VHandShakeFrontCarTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}
	/*
	 * sems
	 */
	Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_BINARY;
	m_sem_handshakefrontcar_start = Semaphore_create(0, &semParams, NULL);
	m_sem_handshakefrontcar_resp = Semaphore_create(0, &semParams, NULL);
}
/*
 * 和前车frontid建立连接,通知前车向本车发送
 */
void V2VHandShakeFrontCar()
{
	Semaphore_post(m_sem_handshakefrontcar_start);
}

void V2VSetFrontCarId(uint16_t frontid)
{
	m_param.frontId = frontid;
}


