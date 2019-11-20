/*
 * RFID.c
 *
 *  管理RFID读写器
 *
 *  接收RFID数据，解析之后放入消息队列
 *
 *  Created on: 2018-12-3
 *      Author: zhtro
 */

#include <xdc/std.h>
#include "uartns550.h"
#include "xil_types.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include "Sensor/RFID/RFID_drv.h"
#include <xdc/runtime/Log.h>
#include "Message/Message.h"
#include <xdc/runtime/Timestamp.h>
#include "task_moto.h"
#include <ti/sysbios/knl/Clock.h>
#include "common.h"
#include <xdc/runtime/Types.h>
#include "Sensor/RFID/EPCdef.h"
#include <ti/sysbios/knl/Mailbox.h>
#include "mpu9250_drv.h"
#include "logLib.h"
#include "Lib/vector.h"
#include "CarState.h"
#include "ZCP/V2V.h"
#include "Sensor/RFID/EPCdef.h"


#define RFID_DEVICENUM  0  //TODO: 放入一个配置表中

extern fbdata_t g_fbData;
static Clock_Handle clock_rfid_heart;
static uint8_t timeout_flag = 0;
static uint32_t circleNum = 0;
static uint8_t lastrfid;
static uint8_t rfidOnline = 0;
static Mailbox_Handle RFIDV2vMbox;
static uint8_t m_rawrfid[12];
static epc_t m_lastepc = {0};
static epc_t m_lastlastepc = {0};
Mailbox_Handle bSecMbox;

static xdc_Void RFIDConnectionClosed(xdc_UArg arg)
{
#if 0
    p_msg_t msg;  
    /*
    * RFID设备连接超时，发送错误消息到主线程
    */
    msg = Message_getEmpty();
	msg->type = error;
	msg->data[0] = ERROR_RFID_TIMEOUT;
	msg->dataLen = 1;
	Message_post(msg);
#endif
    timeout_flag = 1;
    //Clock_setTimeout(clock_rfid_heart,3000);
    //Clock_start(clock_rfid_heart);

    //RFIDStartLoopCheckEpc(RFID_DEVICENUM);
    //setErrorCode(ERROR_CONNECT_TIMEOUT);
}

static void InitTimer()
{
	Clock_Params clockParams;


	Clock_Params_init(&clockParams);
	clockParams.period = 0;       // one shot
	clockParams.startFlag = FALSE;

	clock_rfid_heart = Clock_create(RFIDConnectionClosed, 10000, &clockParams, NULL);
}

int32_t GetMs()
{
	Types_FreqHz freq;
	Types_Timestamp64 timestamp;
	long long timecycle;
	long long freqency;
	Timestamp_getFreq(&freq);
	Timestamp_get64(&timestamp);
	timecycle = _itoll(timestamp.hi, timestamp.lo);
	freqency  = _itoll(freq.hi, freq.lo);
	return  timecycle/(freqency/1000);
}

static uint32_t codeCnt=0;
static void RFIDcallBack(uint16_t deviceNum, uint8_t type, uint8_t data[], uint32_t len )
{
	p_msg_t msg;
	epc_t epc;
	int32_t timeMs;
	car_mode_t mode = Manual;

	switch(type)
	{
		case 0x97:
			Clock_setTimeout(clock_rfid_heart,3000);
			Clock_start(clock_rfid_heart);

			//epc 从第2字节开始，长度12字节
			EPCfromByteArray(&epc, &data[2]);

			/*筛除保留字段不为0的epc*/
			if(epc.reserved != 0x00)
			{
				break;
			}
			/*筛除重复的EPC */
			if(EPCequal(&m_lastepc, &epc))
			{
				break;
			}

			codeCnt++;
			LogMsg("RFID_CODE128 count %d, dist %d\r\n",codeCnt,epc.distance);
			m_lastepc = epc;
#if 0
			/*
			 * 物理RFID发送条件
			 * 1.非自动模式 或
			 * 2.出轨点
			 */
			mode = MotoGetCarMode();
			if(mode == Auto && epc.roadBreak == 0)
			{
			    break;
			}


			//将读到的RFID反馈
			memcpy(g_fbData.rfid, &data[2], EPC_SIZE);

			m_lastlastepc = m_lastepc;
			m_lastepc = epc;
			memcpy(m_rawrfid, &data[2], EPC_SIZE);

			/*
			 * 每读到一个新的EPC值，都存入EEPROM
			 */
//			if(mpu9250WriteBytesFreeLength(EEPROM_SLV_ADDR,EEPROM_ADDR_EPC,EEPROM_LEN_EPC,m_rawrfid) == -1)
//			{
//				LogPrintf("EEPROM: fail to save EPC\n");
//			}

			//g_fbData.distance = epc.distance;
			/*记录圈数*/
			if(epc.distance == 0)
			{
				circleNum++;
			}
			g_fbData.circleNum = circleNum;

			if(epc.roadBreak == 0)
			{
				Mailbox_post(RFIDV2vMbox,(Ptr*)&epc,BIOS_NO_WAIT);
			}

			msg = Message_getEmpty();
			msg->type = rfid;
			memcpy(msg->data, &epc, sizeof(epc_t));
			msg->dataLen = sizeof(epc_t);
			Message_post(msg);

			/*
			 * 发送RFID至V2V模块
			 */
//			userGetMS(&timeMs);
//			epc.timeStamp = timeMs;
#endif
			break;
       case 0x40:
            Clock_setTimeout(clock_rfid_heart,3000);
            Clock_start(clock_rfid_heart);
            if(timeout_flag == 1 || rfidOnline == 0)
            {
            	timeout_flag = 0;
            	rfidOnline = 1;
            	RFIDStartLoopCheckEpc(RFID_DEVICENUM);
            }
            break;

	}
}

Mailbox_Handle RFIDGetV2vMailbox()
{
    return RFIDV2vMbox;
}

/****************************************************************************/
/*                                                                          */
/*              函数定义                                                        */
/*                                                                          */
/****************************************************************************/
Void taskRFID(UArg a0, UArg a1)
{

	RFIDDeviceOpen (RFID_DEVICENUM);
	RFIDRegisterReadCallBack(RFID_DEVICENUM, RFIDcallBack);   //回调函数会在RFIDProcess里面调用

	RFIDStartLoopCheckEpc(RFID_DEVICENUM);
    InitTimer();
    Clock_start(clock_rfid_heart);

    RFIDV2vMbox = Mailbox_create (sizeof (epc_t),4, NULL, NULL);

    /*
     * 从EEPROM中读取上次关机前的EP
     */
    if(mpu9250ReadBytesBlocking(EEPROM_SLV_ADDR,EEPROM_ADDR_EPC,EEPROM_LEN_EPC,m_rawrfid) == -1)
    {
    	LogPrintf("EEPROM: fail to load EPC\n");
    }
    else
    {
    	EPCfromByteArray(&m_lastepc, m_rawrfid);
		Mailbox_post(RFIDV2vMbox,(Ptr*)&m_lastepc,BIOS_NO_WAIT);
    }

	while(1)
	{
		RFIDProcess(RFID_DEVICENUM);

	}

}


static rfidPoint_t *rfidQueue=0;

void taskCreateRFID(UArg a0, UArg a1)
{
    uint8_t size = 0;
    uint32_t carPos = 0;
    uint32_t lastPos = 0;
    uint32_t rfidDist = 0;
    car_mode_t mode = Manual;
    enum motoGear gear = GEAR_NONE;
    rfidPoint_t virtualRfid;
    epc_t epc;
    p_msg_t msg;
    int32_t deltaDist = 0;

    bSecMbox = Mailbox_create (sizeof (int32_t),4, NULL, NULL);
    while(1)
    {
        Task_sleep(100);

        /*
         * 更新车辆位置信息
         */
        lastPos = carPos;
        carPos = MotoGetCarDistance();


        /*
         * 队列为空，无法产生RFID;
         */
        size = vector_size(rfidQueue);
        if(size == 0)
        {
            continue;
        }




        /*
         * 获取队首的距离信息
         *
         */
        rfidDist = (rfidQueue[0].byte[8] << 16) +
                (rfidQueue[0].byte[9] << 8) +
                rfidQueue[0].byte[10];

        gear = MotoGetGear();

        //特殊处理用于0点翻转
        if( (lastPos <= rfidDist && rfidDist <= carPos) || (lastPos>carPos && (rfidDist <= 10 || rfidDist >=TOTAL_DISTANCE-10)) )
        {
            /*
             * 生成RFID，发送消息，并删除当前RFID
             */
            EPCfromByteArray(&epc,rfidQueue[0].byte);

            m_lastlastepc = m_lastepc;
            if(EPC_AB_A == m_lastepc.ab && EPC_AB_B == epc.ab)
			{
            	/*
            	 * 进入B段，发送B段长度
            	 */
                deltaDist = (rfidQueue[0].byte[14] << 24) +
                            (rfidQueue[0].byte[15] << 16) +
                            (rfidQueue[0].byte[16] << 8) +
                            rfidQueue[0].byte[17] ;
				Mailbox_post(bSecMbox,&deltaDist,BIOS_NO_WAIT);
			}
            else if(EPC_AB_A == epc.ab)
            {
                V2VSetDeltaDistance(0);
            }
            m_lastepc = epc;
            memcpy(m_rawrfid, rfidQueue[0].byte, EPC_SIZE);

            /*
             * 每读到一个新的EPC值，都存入EEPROM
             */
//            if(mpu9250WriteBytesFreeLength(EEPROM_SLV_ADDR,EEPROM_ADDR_EPC,EEPROM_LEN_EPC,m_rawrfid) == -1)
//            {
//                LogMsg("EEPROM: fail to save EPC\n");
//            }
			//将读到的RFID反馈
			memcpy(g_fbData.rfid, m_rawrfid, EPC_SIZE);

            msg = Message_getEmpty();
            msg->type = rfid;
            memcpy(msg->data, &epc, sizeof(epc_t));
            msg->dataLen = sizeof(epc_t);
            Message_post(msg);

            if(epc.distance == 0)
            {
                circleNum++;
            }
            g_fbData.circleNum = circleNum;

            LogMsg("Create RFID %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\r\n",
                    rfidQueue[0].byte[0],rfidQueue[0].byte[1],rfidQueue[0].byte[2],rfidQueue[0].byte[3],
                    rfidQueue[0].byte[4],rfidQueue[0].byte[5],rfidQueue[0].byte[6],rfidQueue[0].byte[7],
                    rfidQueue[0].byte[8],rfidQueue[0].byte[9],rfidQueue[0].byte[10],rfidQueue[0].byte[11]);



            if(g_var.cycleRoute == 1)
            {
                /*
                 * 循环路线，将路线压入队尾
                 */
                virtualRfid = rfidQueue[0];
                vector_erase(rfidQueue,0);
                vector_push_back(rfidQueue,virtualRfid);
            }
            else
            {
                vector_erase(rfidQueue,0);
            }

        }

    }
}

void RFIDUpdateQueue(rfidPoint_t *rfidQ)
{
	uint8_t size;
	uint8_t i;
	size = vector_size(rfidQ);
	vector_free(rfidQueue);
	rfidQueue =0;
	for(i=0;i<size;i++)
	{
		vector_push_back(rfidQueue,rfidQ[i]);
	}
}

uint8_t RFIDAppendQueue(rfidPoint_t *rfidQ)
{
    uint16_t size;
    uint16_t i;
    int16_t index;
    epc_t headepc;
    epc_t tempepc;
    epc_t firstrouteepc;

    EPCfromByteArray(&headepc, rfidQ[0].byte);

    size = vector_size(rfidQueue);

    if(size == 0)
    {
        /*
         * RFID队列已空，无法对接路线
         */
        return 0;
    }



    for(i=0;i<vector_size(rfidQ);i++)
    {
    	if(rfidQ[i].byte[12] == 1) //是路径点
    	{
    		break;
    	}
    }

    //路线只有一个点的情况，认为是挪车，都不拒绝
    if(vector_size(rfidQ) > 1)
    {
		//如果下发的路线中没有关键点，拒绝
		if(i>=vector_size(rfidQ))
		{
			return 0;
		}
		else
		{
			EPCfromByteArray(&firstrouteepc, rfidQ[i].byte);

			//如果当前位置离第一个关键点距离前后小于3m，修改失败
			if( abs((int32_t)firstrouteepc.distance - (int32_t)MotoGetCarDistance()) < 30)
			{
				return 0;
			}
		}
	}

    index = -1;
    for(i=0;i<size;i++)
    {
    	EPCfromByteArray(&tempepc, rfidQueue[i].byte);
        if(EPCinSameRoad(&tempepc, &headepc) && tempepc.distance == headepc.distance)
        {
            index = i;
            break;
        }
    }

    //下发的路径第一个点在当前路径列表中，且不是最后一个点
    if(index >= 0 && index < size-1)
    {
        /*
         * 清除相同点之后的路点
         */
    	vector_set_size(rfidQueue, index +1);

        /*
         * 追加新的路点
         */
        size = vector_size(rfidQ);
        for(i=1;i<size;i++)
        {
            vector_push_back(rfidQueue,rfidQ[i]);
        }

        return 1;
    }
    else
    {
        return 0;
    }

}

uint8_t * RFIDGetRaw()
{
	return m_rawrfid;
}

epc_t RFIDGetEpc()
{
	return m_lastepc;
}

epc_t RFIDGetLastEpc()
{
	return m_lastlastepc;
}

uint64_t RFIDGetNID()
{
	return EPCgetShortID(&m_lastepc);
}

void RFIDSetRaw(uint8_t* pRaw)
{
	//将读到的RFID反馈
	memcpy(g_fbData.rfid, pRaw, EPC_SIZE);

	EPCfromByteArray(&m_lastepc, pRaw);

	memcpy(m_rawrfid, pRaw, EPC_SIZE);
}
