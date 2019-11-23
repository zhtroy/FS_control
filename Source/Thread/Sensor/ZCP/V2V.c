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
#define BACK_CAR_NUM (6)

//通信参数
static v2v_param_t m_param = {
		.frontId = V2V_ID_NONE,
		.leftRoadID = {0,0,0,0,0}
};

static uint16_t m_backId[BACK_CAR_NUM] = {0};
static int m_nextBackCarIdx = 0;

//前车状态
static v2v_req_carstatus_t m_frontCarStatus[V2V_MAX_FRONT_CAR];
static uint8_t m_isFrontCarDataUpdated[V2V_MAX_FRONT_CAR] = {0};
static uint32_t m_distanceToFrontCar[V2V_MAX_FRONT_CAR] = {V2V_DISTANCE_INFINITY, V2V_DISTANCE_INFINITY};
static int32_t m_deltaDistance;
static int m_nearestFrontCarIdx = -1;
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
static Semaphore_Handle m_sem_handshakefrontcar_resp[V2V_MAX_FRONT_CAR];
static Semaphore_Handle m_sem_handshakefrontcar_start;


static int V2VIdxOfFrontCar(uint16_t carid)
{
	int i;
	for(i = V2V_MAX_FRONT_CAR-1;i>=0;i--)
	{
		if(m_param.frontId[i] == carid)
			break;
	}

	return i;
}

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

	uint32_t distanceDiff[V2V_MAX_FRONT_CAR];
	uint16_t backCarId = 0;
	uint8_t backCarIndex = 0;

	int backIdx = 0;
	int i;
	uint32_t minDistance;
	int cyclecount = 0;

	while(1)
	{
		Task_sleep(INTERVAL);

		memcpy(g_fbData.backCarIds, m_backId, sizeof(uint16_t) * BACK_CAR_NUM);
		/*
		 * 交替发送
		 */
		for(i = 0; i<BACK_CAR_NUM; i++)
		{
			backCarId = m_backId[(backIdx+i) % BACK_CAR_NUM];

			if(backCarId != V2V_ID_NONE)
			{
				V2VFillCarStatusPacket(&sendPacket, backCarId);
				sendPacket.type = ZCP_TYPE_V2V_REQ_FRONT_CARSTATUS;
				ZCPSendPacket(&v2vInst, &sendPacket, NULL, BIOS_NO_WAIT);

				backIdx = (backIdx+i+1) % BACK_CAR_NUM;
				break;
			}
		}


		myepc=RFIDGetEpc();

		if(myepc.areaType == EPC_AREATYPE_STATION)
		{
			//进站车辆安全距离设置为站台安全距离
			if(g_var.cycleRoute == 0)
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
		for(i=0;i<V2V_MAX_FRONT_CAR;i++)
		{
			if(m_isFrontCarDataUpdated[i])
			{
				isSameAdjustArea = 0;
				if(m_param.frontId[i] == V2V_ID_NONE)
				{
					m_distanceToFrontCar[i] = V2V_DISTANCE_INFINITY;
				}
				else
				{

					EPCfromByteArray(&frontepc, m_frontCarStatus[i].epc);

					if(m_frontCarStatus[i].distance< MotoGetCarDistance())
					{
						distanceDiff[i] = m_frontCarStatus[i].distance + TOTAL_DISTANCE - MotoGetCarDistance();
					}
					else
					{
						distanceDiff[i] = m_frontCarStatus[i].distance - MotoGetCarDistance();
					}
					if(EPCinSameRoad(&myepc, &frontepc))  //在同一条路上
					{
						if(EPC_AB_A == myepc.ab && EPC_AB_B == frontepc.ab)
						{
							m_distanceToFrontCar[i] = distanceDiff[i]  - m_frontCarStatus[i].deltadistance;
						}
						else
						{
							m_distanceToFrontCar[i] = distanceDiff[i];
						}
					}
					else  //不在同一条路
					{
						if(myepc.funcType == EPC_FUNC_NORMAL)
						{
							m_distanceToFrontCar[i] = V2V_DISTANCE_INFINITY;
						}
						else
						{
							if( (myepc.funcType == EPC_FUNC_LADJUST || myepc.funcType == EPC_FUNC_RADJUST)
							   && (frontepc.funcType == EPC_FUNC_LADJUST || frontepc.funcType == EPC_FUNC_RADJUST)
							   && myepc.adjustAreaNo == frontepc.adjustAreaNo)
							{
								isSameAdjustArea = 1;
								//如果和前车在同一调整区，距离不翻转
								if(m_frontCarStatus[i].distance < MotoGetCarDistance())
								{
									m_distanceToFrontCar[i] = 0;
								}
								else
								{
									m_distanceToFrontCar[i] = m_frontCarStatus[i].distance - MotoGetCarDistance();
								}

							}
							else if(0 == memcmp(&m_frontCarStatus[i].epc[1],&m_param.leftRoadID,sizeof(roadID_t)) &&
									EPC_AB_B == frontepc.ab)
							{
								/*
								 * 1.前车处于相邻轨道(主轨)
								 * 2.前车处于B段
								 */
								m_distanceToFrontCar[i] = distanceDiff[i]  - m_frontCarStatus[i].deltadistance;
							}
							else
							{
								m_distanceToFrontCar[i] = distanceDiff[i];
							}
						}
					}
				}



				if(m_distanceToFrontCar[i] < DANGER_DISTANCE && MotoGetCarMode() == Auto )
				{
					if(isSameAdjustArea == 0)
					{
						/*
						 * 自动模式下，非同一调整区，前车距离小于碰撞距离
						 */
						Message_postError(ERROR_SAFE_DISTANCE);
					}
					if(isSameAdjustArea == 1 && myepc.funcType == frontepc.funcType)
					{
						/*
						 * 自动模式下，同一调整区,同一轨道，前车距离小于碰撞距离
						 */
						Message_postError(ERROR_SAFE_DISTANCE);
					}
				}
			}
			else
			{
				//如果前车数据无效，则将前车距离设置为无穷大
				m_distanceToFrontCar[i] = V2V_DISTANCE_INFINITY;
			}
		}

		//找到最小距离的前车
		minDistance = V2V_DISTANCE_INFINITY;
		for(i=0; i < V2V_MAX_FRONT_CAR; i++)
		{
			if(m_distanceToFrontCar[i]<=minDistance)
			{
				m_nearestFrontCarIdx = i;
				minDistance = m_distanceToFrontCar[i];
			}
		}

		g_fbData.forwardCarDistance[0] = m_distanceToFrontCar[0];
		g_fbData.forwardCarDistance[1] = m_distanceToFrontCar[1];

		if(m_nearestFrontCarIdx>=0)
		{
			g_fbData.forwardCarRPM = m_frontCarStatus[m_nearestFrontCarIdx].rpm;
		}
		else
		{
			g_fbData.forwardCarRPM = 0;
		}

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
    epc_t lastFrontEpc[V2V_MAX_FRONT_CAR], frontEpc[V2V_MAX_FRONT_CAR];
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
				//如果收到的包不是前车发来的，不处理
				int idx = V2VIdxOfFrontCar(recvPacket.addr);
				if(idx <0)
				{
					LogMsg("V2V handShake resp from (%x), not equal to my frontCar (%x|%x), ignore\n",
							recvPacket.addr,
							m_param.frontId[0],
							m_param.frontId[1]);
					break;
				}
				else
				{
					LogMsg("V2V handshake accepted from: %x \n",recvPacket.addr);
				}

				Semaphore_post(m_sem_handshakefrontcar_resp[idx]);

				m_isFrontCarDataUpdated[idx] = 1;
				memcpy(&m_frontCarStatus[idx], recvPacket.data, sizeof(v2v_req_carstatus_t));

				break;
			}

			case ZCP_TYPE_V2V_REQ_FRONT_CARSTATUS:
			{

				//如果收到的包不是前车发来的，不处理
				int idx = V2VIdxOfFrontCar(recvPacket.addr);
				if(idx <0)
				{
//					LogMsg("V2V carstatus resp from (%x), not equal to my frontCar (%x|%x), ignore\n",
//							recvPacket.addr,
//							m_param.frontId[0],
//							m_param.frontId[1]);
					break;
				}

				m_isFrontCarDataUpdated[idx] = 1;
				memcpy(&m_frontCarStatus[idx], recvPacket.data, sizeof(v2v_req_carstatus_t));

				//如果前车上一次在分离区，这次在普通区，即离开分离区，发送消息
				EPCfromByteArray(&frontEpc[idx], m_frontCarStatus[idx].epc);
				if(EPC_FUNC_NORMAL == frontEpc[idx].funcType && EPC_FUNC_SEPERATE == lastFrontEpc[idx].funcType)
				{
					Message_postEvent(internal, IN_EVTCODE_V2V_FRONTCAR_LEAVE_SEPERATE);
				}
				lastFrontEpc[idx] = frontEpc[idx];


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
    int i;

    while(1)
    {
    	Semaphore_pend(m_sem_handshakefrontcar_start,BIOS_WAIT_FOREVER);

    	for(i=0;i<V2V_MAX_FRONT_CAR;i++	)
    	{
    		if(m_param.frontId[i] != V2V_ID_NONE)
    		{
				Semaphore_reset(m_sem_handshakefrontcar_resp[i],0);

				retryNum = RETRY_NUM;
				do
				{
					/*
					 * 构造请求报文
					 */

					sendPacket.addr = m_param.frontId[i];
					sendPacket.type = ZCP_TYPE_V2V_REQ_BACK_HANDSHAKE;
					sendPacket.len = 0;

					/*
					 * 发送请求
					 */
					ZCPSendPacket(&v2vInst, &sendPacket, NULL, BIOS_NO_WAIT);

					pendResult = Semaphore_pend(m_sem_handshakefrontcar_resp[i], TIMEOUT);
					retryNum --;
				}
				while(retryNum>0 && pendResult==FALSE);

				LogMsg("V2V handshake with: %x \n",m_param.frontId[i]);

				if(pendResult == FALSE)
				{
					Message_postError(ERROR_V2V);
				}
    		}
    	}
    }


}

/*
 * API
 */




void V2VStartUpTask()
{
	int i;
	Task_Handle task;
	Task_Params taskParams;
	Semaphore_Params semParams;

	ZCPInit(&v2vInst, V2V_ZCP_DEV_NUM,V2V_ZCP_UART_DEV_NUM, g_sysParam.carID );
	InitTimer();

	/*
	 * sems
	 */
	Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_BINARY;
	m_sem_handshakefrontcar_start = Semaphore_create(0, &semParams, NULL);
	for(i=0;i<V2V_MAX_FRONT_CAR;i++)
	{
		m_sem_handshakefrontcar_resp[i] = Semaphore_create(0, &semParams, NULL);
	}

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

}

void V2VInit()
{
	Task_Handle task;
	Task_Params taskParams;

	Task_Params_init(&taskParams);
	taskParams.stackSize = 2048;
	taskParams.priority = 5;

	task = Task_create((Task_FuncPtr)V2VStartUpTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}
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
	m_deltaDistance = delta;
}

uint8_t V2VSetFrontCarId(uint16_t frontid[])
{
	int i,j;
	ZCPUserPacket_t sendPacket;
	uint8_t frontCarChanged = 0;
	uint16_t _frontid[V2V_MAX_FRONT_CAR];

	if(frontid == V2V_ID_NONE)
	{
		memset(_frontid,0,sizeof(_frontid));
	}
	else
	{
		memcpy(_frontid,frontid, sizeof(_frontid));
	}

	//确定前车是否修改过了
	if( (_frontid[0] == m_param.frontId[0] && _frontid[1] == m_param.frontId[1])
		|| (_frontid[0] == m_param.frontId[1] && _frontid[1] == m_param.frontId[0])	)
	{

	}
	else
	{
		frontCarChanged = 1;
	}


	if(frontCarChanged)
	{
		//先向原来的前车发停止发送命令
		for(i=0;i<V2V_MAX_FRONT_CAR;i++)
		{
			for(j=0;j<V2V_MAX_FRONT_CAR;j++)
			{
				if(m_param.frontId[i] == _frontid[j])
				{
					break;
				}
			}

			if(j>=V2V_MAX_FRONT_CAR && m_param.frontId[i]!=V2V_ID_NONE)
			{

				sendPacket.addr =  m_param.frontId[i];
				sendPacket.type = ZCP_TYPE_V2V_REQ_BACK_STOPSENDING;
				sendPacket.len = 0;
				/*
				 * 发送请求
				 */
				ZCPSendPacket(&v2vInst, &sendPacket, NULL, BIOS_NO_WAIT);
			}
		}

		memcpy(m_param.frontId, _frontid, V2V_MAX_FRONT_CAR*sizeof(uint16_t));

		memcpy(g_fbData.frontCarID, _frontid, V2V_MAX_FRONT_CAR*sizeof(uint16_t));

		//清除前车信息
		memset(m_isFrontCarDataUpdated ,0, V2V_MAX_FRONT_CAR*sizeof(uint8_t));

		LogMsg("V2V setfrontID: %x %x\n",m_param.frontId[0],m_param.frontId[1]);
	}

	return frontCarChanged;
}

void V2VSetLeftRoadID(roadID_t raodID)
{
    m_param.leftRoadID = raodID;
}

uint16_t V2VGetFrontCarId()
{
	return m_param.frontId[0];
}

uint32_t V2VGetDistanceToFrontCar()
{
	return m_distanceToFrontCar[m_nearestFrontCarIdx];
}

uint16_t V2VGetFrontCarSpeed()
{
	return m_frontCarStatus[m_nearestFrontCarIdx].rpm;
}

epc_t V2VGetFrontCarEpc()
{
	epc_t epc;
	EPCfromByteArray(&epc, m_frontCarStatus[0].epc);

	return epc;
}

uint8_t V2VIsSameAdjustArea()
{
    return isSameAdjustArea;
}


