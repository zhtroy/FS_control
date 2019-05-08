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


#define RFID_DEVICENUM  0  //TODO: 放入一个配置表中

extern fbdata_t g_fbData;
static Clock_Handle clock_rfid_heart;
static uint8_t timeout_flag = 0;
static uint32_t circleNum = 0;
static uint8_t lastrfid;
static uint8_t rfidOnline = 0;
static Mailbox_Handle RFIDV2vMbox;

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
	static epc_t lastepc = {0};

	switch(type)
	{
		case 0x97:
			Clock_setTimeout(clock_rfid_heart,3000);
			Clock_start(clock_rfid_heart);

			//epc 从第2字节开始，长度12字节
			EPCfromByteArray(&epc, &data[2]);
			/*筛除重复的EPC */
			if(EPCequal(&lastepc, &epc))
			{
				break;
			}
			lastepc = epc;

			/*记录圈数*/
			if(data[2] != lastrfid && data[2] == 0x06)
			{
				circleNum++;
			}
			lastrfid = data[2];
			g_fbData.circleNum = circleNum;

			msg = Message_getEmpty();
			msg->type = rfid;
			memcpy(msg->data, &epc, sizeof(epc_t));
			msg->dataLen = sizeof(epc_t);
			Message_post(msg);

			/*
			 * 发送RFID至V2V模块
			 */
			userGetMS(&timeMs);
			epc.timeStamp = timeMs;
			Mailbox_post(RFIDV2vMbox,(Ptr*)&epc,BIOS_NO_WAIT);
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

	while(1)
	{
		RFIDProcess(RFID_DEVICENUM);

	}

}



