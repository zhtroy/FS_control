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
#include "Parameter.h"


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

static xdc_Void RFIDConnectionClosed(xdc_UArg arg)
{
    p_msg_t msg;  
    /*
    * RFID设备连接超时，发送错误消息到主线程
    */
    msg = Message_getEmpty();
	msg->type = error;
	msg->data[0] = ERROR_RFID_TIMEOUT;
	msg->dataLen = 1;
	Message_post(msg);
	timeout_flag = 1;
    
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

			//将读到的RFID反馈
			memcpy(g_fbData.rfid, &data[2], EPC_SIZE);
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

			m_lastlastepc = m_lastepc;
			m_lastepc = epc;
			memcpy(m_rawrfid, &data[2], EPC_SIZE);

			/*
			 * 每读到一个新的EPC值，都存入EEPROM
			 */
			if(mpu9250WriteBytesFreeLength(EEPROM_SLV_ADDR,EEPROM_ADDR_EPC,EEPROM_LEN_EPC,m_rawrfid) == -1)
			{
				LogPrintf("EEPROM: fail to save EPC\n");
			}

			//g_fbData.distance = epc.distance;
			/*记录圈数*/
			if(epc.distance == 0)
			{
				circleNum++;
			}
			g_fbData.circleNum = circleNum;

			Mailbox_post(RFIDV2vMbox,(Ptr*)&epc,BIOS_NO_WAIT);

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
    if(mpu9250ReadBytes(EEPROM_SLV_ADDR,EEPROM_ADDR_EPC,EEPROM_LEN_EPC,m_rawrfid) == -1)
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
    car_mode_t modeOld = Manual;
    enum motoGear gear = GEAR_NONE;
    rfidPoint_t virtualRfid;
    epc_t epc;
    p_msg_t msg;
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
         * 车辆从自动模式跳出
         */
        modeOld = mode;
        mode = MotoGetCarMode();
        if(mode != Auto)
        {
            if(modeOld == Auto)
            {
                /*
                 * 从自动模式跳出，清除队列
                 */
                vector_free(rfidQueue);
                rfidQueue = 0;
            }

            /*
             * 非自动模式采用物理RFID
             */
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

        if(gear == GEAR_DRIVE && lastPos <= rfidDist && rfidDist <= carPos)
        {
            /*
             * 生成RFID，发送消息，并删除当前RFID
             */
            EPCfromByteArray(&epc,rfidQueue[0].byte);

            m_lastlastepc = m_lastepc;
            m_lastepc = epc;
            memcpy(m_rawrfid, rfidQueue[0].byte, EPC_SIZE);

            /*
             * 每读到一个新的EPC值，都存入EEPROM
             */
            if(mpu9250WriteBytesFreeLength(EEPROM_SLV_ADDR,EEPROM_ADDR_EPC,EEPROM_LEN_EPC,m_rawrfid) == -1)
            {
                LogPrintf("EEPROM: fail to save EPC\n");
            }
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

            if(g_param.cycleRoute == 1)
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

