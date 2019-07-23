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
#include "Sensor/RFID/RFID_task.h"
#include "Sensor/SonicRadar/SonicRadar.h"
#include <ti/sysbios/knl/Clock.h>
#include "logLib.h"

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
		.backIdAdd = V2V_ID_NONE,
		.frontId = V2V_ID_NONE,
		.leftRoadID = {0,0,0,0,0}
};
//前车状态
static v2v_req_carstatus_t m_frontCarStatus;
static uint32_t m_distanceToFrontCar = V2V_DISTANCE_INFINITY;
static int32_t m_deltaDistance;
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

	epc_t myepc;
	epc_t frontepc;

	uint32_t distanceDiff;
	uint16_t backCarId = 0;
	uint8_t backCarIndex = 0;

	while(1)
	{
		Task_sleep(INTERVAL);

		/*
		 * 交替发送
		 */
		if(backCarIndex  == 0)
		{
		    backCarId = m_param.backId;
		    backCarIndex = 1;
		}
		else
		{
		    backCarId = m_param.backIdAdd;
		    backCarIndex = 0;
		}

		//发送carstatus报文给后车
		if(backCarId != V2V_ID_NONE)
		{
			carstatus.rpm = MotoGetRealRPM();
			if(MotoGetCarMode() == ForceBrake)
			{
				carstatus.status = V2V_CARSTATUS_ERROR;
			}
			else
			{
				if(MotoGetRealRPM() == 0)
					carstatus.status = V2V_CARSTATUS_STOP;
				else
					carstatus.status = V2V_CARSTATUS_RUNNING;
			}

			memcpy(carstatus.epc, RFIDGetRaw(), EPC_SIZE);
			carstatus.distance = MotoGetCarDistance();
			carstatus.deltadistance = m_deltaDistance;

			memcpy(sendPacket.data, &carstatus, sizeof(carstatus));
			sendPacket.addr = backCarId;
			sendPacket.type = ZCP_TYPE_V2V_REQ_FRONT_CARSTATUS;
			sendPacket.len = sizeof(carstatus);

			ZCPSendPacket(&v2vInst, &sendPacket, NULL, BIOS_NO_WAIT);
		}

		/*
		 * 定时计算前车距离
		 */
		myepc=RFIDGetEpc();

		if(myepc.areaType == EPC_AREATYPE_STATION)
		{
			MotoSetSafeDistance(SAFE_DISTANCE_STATION,SAFE_DISTANCE_STATION);
		}

		if(myepc.areaType == EPC_AREATYPE_NORMAL)
		{
			MotoSetSafeDistance(MIN_SAFE_DISTANCE,MAX_SAFE_DISTANCE);
		}

		/*
		if(myepc.areaType == EPC_AREATYPE_STATION)
		{
			m_distanceToFrontCar = SonicGetDistance()/100;
		}
		else  //普通区距离
		*/
		{
			if(m_param.frontId == V2V_ID_NONE)
			{
				m_distanceToFrontCar = V2V_DISTANCE_INFINITY;
			}
			else
			{

				EPCfromByteArray(&frontepc, m_frontCarStatus.epc);

				if(m_frontCarStatus.distance< MotoGetCarDistance())
				{
					distanceDiff = m_frontCarStatus.distance + TOTAL_DISTANCE - MotoGetCarDistance();
				}
				else
				{
					distanceDiff = m_frontCarStatus.distance - MotoGetCarDistance();
				}
				if(EPCinSameRoad(&myepc, &frontepc))  //在同一条路上
				{
					if(EPC_AB_A == myepc.ab && EPC_AB_B == frontepc.ab)
					{
						m_distanceToFrontCar = distanceDiff  - m_frontCarStatus.deltadistance;
					}
					else
					{
						m_distanceToFrontCar = distanceDiff;
					}
				}
				else  //不在同一条路
				{
					if(myepc.funcType == EPC_FUNC_NORMAL)
					{
						m_distanceToFrontCar = V2V_DISTANCE_INFINITY;
					}
					else
					{
						if( (myepc.funcType == EPC_FUNC_LADJUST || myepc.funcType == EPC_FUNC_RADJUST)
						   && (frontepc.funcType == EPC_FUNC_LADJUST || frontepc.funcType == EPC_FUNC_RADJUST)
						   && myepc.adjustAreaNo == frontepc.adjustAreaNo)
						{
							//如果和前车在同一调整区，距离不翻转
							if(m_frontCarStatus.distance < MotoGetCarDistance())
							{
								m_distanceToFrontCar = 0;
							}
							else
							{
								m_distanceToFrontCar = m_frontCarStatus.distance - MotoGetCarDistance();
							}

						}
						else if(0 == memcmp(&m_frontCarStatus.epc[1],&m_param.leftRoadID,sizeof(roadID_t)) &&
					            EPC_AB_B == frontepc.ab)
					    {
					        /*
					         * 1.前车处于相邻轨道(主轨)
					         * 2.前车处于B段
					         */
					        m_distanceToFrontCar = distanceDiff  - m_frontCarStatus.deltadistance;
					    }
					    else
					    {
					        m_distanceToFrontCar = distanceDiff;
					    }
					}
				}
			}

		}

		g_fbData.forwardCarDistance = m_distanceToFrontCar;
		g_fbData.forwardCarRPM = m_frontCarStatus.rpm;

	}
}
static Clock_Handle clock_front_car;

static xdc_Void FrontCarDisconnected(xdc_UArg arg)
{
    p_msg_t msg;
    /*
    * 前车连接超时，发送错误消息到主线程
    */
    if(m_param.frontId != 0)
    {
        /*
         * 有前车，超时异常
         */
        msg = Message_getEmpty();
        msg->type = error;
        msg->data[0] = ERROR_FRONT_CAR_TIMEOUT;
        msg->dataLen = 1;
        Message_post(msg);
    }
    else
    {
        /*
         * 无前车，则重新设置定时器
         */
        Clock_setTimeout(clock_front_car,TIMEOUT_FRONT_CAR_DISCONNECT);
        Clock_start(clock_front_car);
    }
}
static void InitTimer()
{
    Clock_Params clockParams;


    Clock_Params_init(&clockParams);
    clockParams.period = 0;       // one shot
    clockParams.startFlag = FALSE;

    clock_front_car = Clock_create(FrontCarDisconnected, TIMEOUT_FRONT_CAR_DISCONNECT, &clockParams, NULL);
}

static void V2VRecvTask(UArg arg0, UArg arg1)
{
    int32_t timestamp;
    ZCPUserPacket_t recvPacket, sendPacket;
    epc_t lastFrontEpc, frontEpc;

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
			    if(recvPacket.addr != m_param.backId)
			    {
			        m_param.backIdAdd = m_param.backId;
			        m_param.backId = recvPacket.addr;
			    }

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

				/*
                 * 收到前车报文，重启定时器
                 */
                Clock_setTimeout(clock_front_car,TIMEOUT_FRONT_CAR_DISCONNECT);
                Clock_start(clock_front_car);
				break;
			}

			case ZCP_TYPE_V2V_REQ_FRONT_CARSTATUS:
			{

				//如果收到的包不是前车发来的，不处理
				if(recvPacket.addr != m_param.frontId)
				{
					break;
				}
				memcpy(&m_frontCarStatus, recvPacket.data, sizeof(m_frontCarStatus));

				//如果前车上一次在分离区，这次在普通区，即离开分离区，发送消息
				EPCfromByteArray(&frontEpc, m_frontCarStatus.epc);
				if(EPC_FUNC_NORMAL == frontEpc.funcType && EPC_FUNC_SEPERATE == lastFrontEpc.funcType)
				{
					Message_postEvent(internal, IN_EVTCODE_V2V_FRONTCAR_LEAVE_SEPERATE);
				}
				lastFrontEpc = frontEpc;

				/*
                 * 收到前车报文，重启定时器
                 */
                Clock_setTimeout(clock_front_car,TIMEOUT_FRONT_CAR_DISCONNECT);
                Clock_start(clock_front_car);
				break;
			}
		}
	}
}

static void V2VHandShakeFrontCarTask(UArg arg0, UArg arg1)
{
	const uint32_t TIMEOUT = 200;
	const int RETRY_NUM = 10;

	ZCPUserPacket_t sendPacket;
    Bool pendResult;
    int retryNum;

    while(1)
    {
    	Semaphore_pend(m_sem_handshakefrontcar_start,BIOS_WAIT_FOREVER);

    	Semaphore_reset(m_sem_handshakefrontcar_resp,0);

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
			Message_postError(ERROR_V2V);
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
	InitTimer();

	Task_Params_init(&taskParams);

	taskParams.stackSize = 2048;
	taskParams.priority = V2V_RECV_TASK_PRIO;

	task = Task_create((Task_FuncPtr)V2VRecvTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	task = Task_create((Task_FuncPtr)V2VSendTask, &taskParams, NULL);
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

void V2VSetDeltaDistance(int32_t delta)
{
	LogDebug("Delta distance: %d", delta);
	m_deltaDistance = delta;
}

void V2VSetFrontCarId(uint16_t frontid)
{
	g_fbData.frontCarID = frontid;

	m_param.frontId = frontid;
}

void V2VSetLeftRoadID(roadID_t raodID)
{
    m_param.leftRoadID = raodID;
}

uint16_t V2VGetFrontCarId()
{
	return m_param.frontId;
}

uint32_t V2VGetDistanceToFrontCar()
{
	return m_distanceToFrontCar;
}

uint16_t V2VGetFrontCarSpeed()
{
	return m_frontCarStatus.rpm;
}

epc_t V2VGetFrontCarEpc()
{
	epc_t epc;
	EPCfromByteArray(&epc, m_frontCarStatus.epc);

	return epc;
}


