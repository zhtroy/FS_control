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
#include "Moto/Parameter.h"

#define V2V_ZCP_UART_DEV_NUM    (1)
#define V2V_ZCP_DEV_NUM (0)
/*
 * 确保接收task优先级比较低
 */
#define V2V_RECV_TASK_PRIO 		(5)
#define V2V_SESSION_TASK_PRIO   (V2V_RECV_TASK_PRIO + 1)
#define BACK_CAR_NUM (3)

//通信参数
static v2v_param_t m_param = {
		.frontId = V2V_ID_NONE,
		.leftRoadID = {0,0,0,0,0}
};

static uint16_t m_backId[BACK_CAR_NUM] = {0};
static int m_nextBackCarIdx = 0;

//前车状态
static v2v_req_carstatus_t m_frontCarStatus;
static uint8_t m_isFrontCarDataUpdated = 0;
static uint32_t m_distanceToFrontCar = V2V_DISTANCE_INFINITY;
static int32_t m_deltaDistance;
//ZCP驱动实例
static ZCPInstance_t v2vInst;
static uint8_t isSameAdjustArea = 0;
/*
 * task
 */
static Task_Handle m_task_handshakefrontcar = NULL;

/*
 * sems
 */
static Semaphore_Handle m_sem_handshakefrontcar_resp;
static Semaphore_Handle m_sem_handshakefrontcar_start;

static void V2VFillCarStatusPacket(ZCPUserPacket_t* psendPacket, uint16_t backCarId)
{
	v2v_req_carstatus_t carstatus;

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

	memcpy(psendPacket->data, &carstatus, sizeof(carstatus));
	psendPacket->addr = backCarId;
	psendPacket->len = sizeof(carstatus);
}

static void V2VSendTask(UArg arg0, UArg arg1)
{

	const int INTERVAL = 100;

	ZCPUserPacket_t sendPacket;

	epc_t myepc;
	epc_t frontepc;

	uint32_t distanceDiff;
	uint16_t backCarId = 0;
	uint8_t backCarIndex = 0;

	int backIdx = 0;

	while(1)
	{
		Task_sleep(INTERVAL);

		/*
		 * 交替发送
		 */

		backCarId = m_backId[backIdx];

		backIdx = (backIdx+1) % BACK_CAR_NUM;

		//发送carstatus报文给后车
		if(backCarId != V2V_ID_NONE)
		{
			V2VFillCarStatusPacket(&sendPacket, backCarId);
			sendPacket.type = ZCP_TYPE_V2V_REQ_FRONT_CARSTATUS;
			ZCPSendPacket(&v2vInst, &sendPacket, NULL, BIOS_NO_WAIT);
		}


		myepc=RFIDGetEpc();

		if(myepc.areaType == EPC_AREATYPE_STATION)
		{
			//进站车辆安全距离设置为站台安全距离
			if(g_param.cycleRoute == 0)
			{
				MotoSetSafeDistance(MIN_SAFE_DISTANCE_STATION,MAX_SAFE_DISTANCE_STATION);
			}
			else
			{
				MotoSetSafeDistance(MIN_SAFE_DISTANCE,MAX_SAFE_DISTANCE);
			}
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

		/*
		 * 定时计算前车距离
		 */
		if(m_isFrontCarDataUpdated)
		{
		    isSameAdjustArea = 0;
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
						    isSameAdjustArea = 1;
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



			if(isSameAdjustArea == 0 && m_distanceToFrontCar < DANGER_DISTANCE && MotoGetCarMode() == Auto )
			{
				/*
				 * 自动模式下，非同一调整区，前车距离小于碰撞距离
				 */
				Message_postError(ERROR_SAFE_DISTANCE);
			}
		}
		else
		{
			//如果前车数据无效，则将前车距离设置为无穷大
			m_distanceToFrontCar = V2V_DISTANCE_INFINITY;
		}

		g_fbData.forwardCarDistance = m_distanceToFrontCar;
		g_fbData.forwardCarRPM = m_frontCarStatus.rpm;

	}
}

static void InitTimer()
{
    Clock_Params clockParams;


    Clock_Params_init(&clockParams);
    clockParams.period = 0;       // one shot
    clockParams.startFlag = FALSE;


}

static void V2VRecvTask(UArg arg0, UArg arg1)
{
    int32_t timestamp;
    ZCPUserPacket_t recvPacket, sendPacket;
    epc_t lastFrontEpc, frontEpc;
    int i;

	while(1)
	{
		ZCPRecvPacket(&v2vInst, &recvPacket, &timestamp, BIOS_WAIT_FOREVER);

		switch(recvPacket.type)
		{
			case ZCP_TYPE_V2V_REQ_BACK_STOPSENDING:
			{
				/*
				 * 停止往后车发送，将后车ID设为0
				 */
				for(i = 0; i<BACK_CAR_NUM; i++)
				{
					if(m_backId[i] == recvPacket.addr)
					{
						m_backId[i] = 0;
						break;
					}
				}

				break;
			}

			case ZCP_TYPE_V2V_REQ_BACK_HANDSHAKE:
			{
				/*
				 * 检查是否已经有该后车,如果有了就把这个位置设为0
				 */
				for(i = 0; i<BACK_CAR_NUM; i++)
				{
					if(m_backId[i] == recvPacket.addr)
					{
						m_backId[i] = V2V_ID_NONE;
					}
				}


				/*
				 * 循环更新后车ID，
				 */
				m_backId[m_nextBackCarIdx] = recvPacket.addr;
				m_nextBackCarIdx = (m_nextBackCarIdx + 1) % BACK_CAR_NUM;


				/*
				 * 并回复一个响应报文给后车
				 */

				V2VFillCarStatusPacket(&sendPacket, recvPacket.addr);
				sendPacket.type = ZCP_TYPE_V2V_RESP_FRONT_HANDSHAKE;

				ZCPSendPacket(&v2vInst, &sendPacket, NULL, BIOS_NO_WAIT);


				break;
			}

			case ZCP_TYPE_V2V_RESP_FRONT_HANDSHAKE:
			{
				Semaphore_post(m_sem_handshakefrontcar_resp);

				m_isFrontCarDataUpdated = 1;
				memcpy(&m_frontCarStatus, recvPacket.data, sizeof(m_frontCarStatus));

				break;
			}

			case ZCP_TYPE_V2V_REQ_FRONT_CARSTATUS:
			{

				//如果收到的包不是前车发来的，不处理
				if(recvPacket.addr != m_param.frontId)
				{
					break;
				}
				m_isFrontCarDataUpdated = 1;
				memcpy(&m_frontCarStatus, recvPacket.data, sizeof(m_frontCarStatus));

				//如果前车上一次在分离区，这次在普通区，即离开分离区，发送消息
				EPCfromByteArray(&frontEpc, m_frontCarStatus.epc);
				if(EPC_FUNC_NORMAL == frontEpc.funcType && EPC_FUNC_SEPERATE == lastFrontEpc.funcType)
				{
					Message_postEvent(internal, IN_EVTCODE_V2V_FRONTCAR_LEAVE_SEPERATE);
				}
				lastFrontEpc = frontEpc;


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
	ZCPUserPacket_t sendPacket;

	g_fbData.frontCarID = frontid;

	//先向原来的前车发停止发送命令
	if(V2VGetFrontCarId()!=V2V_ID_NONE)
	{
		sendPacket.addr = V2VGetFrontCarId();
		sendPacket.type = ZCP_TYPE_V2V_REQ_BACK_STOPSENDING;
		sendPacket.len = 0;
		/*
		 * 发送请求
		 */
		ZCPSendPacket(&v2vInst, &sendPacket, NULL, BIOS_NO_WAIT);
	}

	m_param.frontId = frontid;
	//清除前车信息
	m_isFrontCarDataUpdated = 0;
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

uint8_t V2VIsSameAdjustArea()
{
    return isSameAdjustArea;
}
