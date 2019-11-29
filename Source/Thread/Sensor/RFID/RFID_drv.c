/*
 * RFID_drv.c
 *
 *  Created on: 2018-11-14
 *      Author: zhtro
 */

#include "RFID_drv.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "uartns550.h"
#include "stdint.h"
#include <xdc/runtime/Log.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include "assert.h"
#include "common.h"

#define RFID_MBOX_DEPTH (16)
/****************************************************************************/
/*                                                                          */
/*              变量定义                                                        */
/*                                                                          */
/****************************************************************************/

//RFID配置表
static RFID_instance_t rfid_cfg_table[]={
		//RFID设备0
		{
		  //串口设备号,不能重复
		  0
		}
};

static uartDataObj_t rfidUartDataObj;

/****************************************************************************/
/*                                                                          */
/*              函数声明                                                        */
/*                                                                          */
/****************************************************************************/
static void uartRFIDIntrHandler(void *callBackRef, u32 event, unsigned int eventData);
static RFID_instance_t * getInstanceByDeviceNum(uint16_t deviceNum);


//utils function====================
#if 0
static unsigned char calculateCRCAdd(unsigned char A, unsigned char B)
{
	return (unsigned char)(A + B)&(0xFF);
}
#endif

static unsigned char calculateCRC(unsigned char * data, int len)
{
	int sum = 0;
	int i;

	for (i = 0; i < len; i++)
	{
		sum += data[i];
	}
	return (unsigned char)(sum & 0xFF);
}

static unsigned char buffer[RFID_TX_BUFFER_SIZE];

static int RFID_send_packet(uint16_t deviceNum, unsigned char *data , int len)
{
#if 0
	assert((len+4)<=RFID_TX_BUFFER_SIZE);
	RFID_instance_t * pinst;

	pinst =getInstanceByDeviceNum(deviceNum);



	unsigned char buffer[RFID_TX_BUFFER_SIZE];
	buffer[0] = 0xBB;
	memcpy(&buffer[1], data, len);
	buffer[len+1] = calculateCRC(data,len);
	buffer[len+2] = 0x0D;
	buffer[len+3] = 0x0A;


	UartNs550Send(pinst->uartDeviceNum, &(buffer[0]), len+4);
#endif
    RFID_instance_t * pinst;
    pinst =getInstanceByDeviceNum(deviceNum);
	memcpy(buffer, data, len);
	UartNs550Send(pinst->uartDeviceNum, buffer, len);
	return 1;
}


#if 0
static void RFIDstateMachineReset(RFID_stateMachine_t * inst)
{
    inst->crc = 0;
    inst->curLen = 0;
    inst->msgLen = 0;
    inst->type = 0;
    inst->state = Ready;
}
#else
static void RFIDstateMachineReset(RFID_stateMachine_t * inst)
{
    inst->crc = 0;
    inst->curLen = 0;
    inst->msgLen = 0;
    inst->type = 0;
    inst->state = Prefix0;
}
#endif


static RFID_instance_t * getInstanceByDeviceNum(uint16_t deviceNum)
{
	RFID_instance_t * pinst;
	int rfid_cfg_num;

	rfid_cfg_num = sizeof(rfid_cfg_table)/ sizeof (rfid_cfg_table[0]);

	if(deviceNum>=rfid_cfg_num){
		Log_error2("RFID: requested deviceNum [%d], total devices [%d]",deviceNum, rfid_cfg_num);
		BIOS_exit(0);
	}

	pinst = &(rfid_cfg_table[deviceNum]);

	return pinst;
}

// 根据UART devicenum 获取RFID num
static uint16_t getRFIDnumByUartNum(uint16_t uartNum)
{
	uint16_t i;
//	RFID_instance_t * pinst;
	int rfid_cfg_num;

	rfid_cfg_num = sizeof(rfid_cfg_table)/ sizeof (rfid_cfg_table[0]);

	for(i=0;i<rfid_cfg_num;i++){
		if(rfid_cfg_table[i].uartDeviceNum ==  uartNum)
		{
			return i;
		}
	}
	return 0xFFFF;

}

//API
//-----------------------------//


/*****************************************************************************
 * 函数名称: void RFIDDeviceOpen(uint16_t deviceNum)
 * 函数说明: 打开RFID读写器
 * 输入参数:
 *        deviceNum：设备号，对应的是设备配置表rfid_cfg_table中的数组下标
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/
void RFIDDeviceOpen(uint16_t deviceNum)
{
	RFID_instance_t * pinst;
	//Semaphore_Params semParams;
	Mailbox_Params mboxParams; 

	pinst = getInstanceByDeviceNum(deviceNum);


    /* 初始化接收邮箱 */
    Mailbox_Params_init(&mboxParams);
    pinst->recvMbox = Mailbox_create (sizeof (uartDataObj_t),RFID_MBOX_DEPTH, &mboxParams, NULL);

	RFIDstateMachineReset(&(pinst->sminst));
    
	//连接串口
	UartNs550Init(pinst->uartDeviceNum,uartRFIDIntrHandler);
    UartNs550Recv(deviceNum, rfidUartDataObj.buffer, UART_REC_BUFFER_SIZE);
}

/*****************************************************************************
 * 函数名称: void RFIDRegisterReadCallBack(uint16_t deviceNum, RFIDcallback cb)
 * 函数说明: 为指定RFID设备注册一个读数据的回调函数cb
 * 输入参数:
 *        deviceNum：设备号，对应的是设备配置表rfid_cfg_table中的数组下标
 *        cb: 回调函数，当读到一个数据包时，调用cb
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/

void RFIDRegisterReadCallBack(uint16_t deviceNum, RFIDcallback cb)
{
	RFID_instance_t * pinst;

	pinst = getInstanceByDeviceNum(deviceNum);
	pinst->callback = cb;

}

/*****************************************************************************
 * 函数名称: int RFIDStartLoopCheckEpc(uint16_t deviceNum)
 * 函数说明: 开始循环查询EPC
 * 输入参数:
 *        deviceNum：设备号，对应的是设备配置表rfid_cfg_table中的数组下标
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/

int RFIDStartLoopCheckEpc(uint16_t deviceNum)
{
	return RFID_send_packet(deviceNum, "LON\r", 4);
}

/*****************************************************************************
 * 函数名称: int RFIDStopLoopCheckEpc(uint16_t deviceNum)
 * 函数说明: 停止循环查询EPC
 * 输入参数:
 *        deviceNum：设备号，对应的是设备配置表rfid_cfg_table中的数组下标
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/

int RFIDStopLoopCheckEpc(uint16_t deviceNum)
{
	return RFID_send_packet(deviceNum,"LOFF\r",5);
}




#if 0
//RFID 状态机
static RFID_state protocolStateMachine(uint8_t c,uint16_t deviceNum)
{

	RFID_instance_t * pinst;
	RFID_stateMachine_t *inst;
	unsigned char calCRC;

	pinst =getInstanceByDeviceNum(deviceNum);
	inst = &(pinst->sminst);

	switch(inst->state)
	{
		case Ready:
			if(0xBB == c)
			{
				inst->state = Head;
			}
			break;
		case Head:
			inst->type = c;
			inst->state = Type;
			break;
		case Type:
			inst->msgLen = c;
			inst->state = Len;
			break;
		case Len:
			inst->msg[inst->curLen++] = c;
			if(inst->msgLen == inst->curLen)
			{
				inst->state = Data;
			}
			break;
		case Data:
			inst->crc = c;
			inst->state = CRC;
			break;
		case CRC:
			calCRC = calculateCRC(inst->msg, inst->msgLen);
			calCRC = calculateCRCAdd(calCRC, inst->type);
			calCRC = calculateCRCAdd(calCRC, inst->msgLen);
			if(inst->crc == calCRC)
			{
				if (0x0D == c)
				{
					inst->state = END1;
				}
				else {
				    RFIDstateMachineReset(inst);
				}
			}
			else
			{
			    Log_warning2("RFID CRC error, recv %d ,calc %d\n",  inst->crc, calCRC);
			    RFIDstateMachineReset(inst);

			}
			break;

		case END1:
			if(0x0A == c)
			{
				inst->state = END2;
			}
			else{
			    RFIDstateMachineReset(inst);
			}
			break;

		case END2:
			if(pinst->callback==0)
			{
				Log_error0("RFID state machine error, need to register call back fucntion\n");
			}
			else{
				pinst->callback(deviceNum, inst->type, inst->msg, inst->msgLen);
			}
			RFIDstateMachineReset(inst);
			break;

		default:
			Log_error1("RFID state machine error, unknown state: %d\n",  inst->state);
		



	}
	return inst->state;
}
#else
//RFID 状态机

static void code128ToRFID(uint8_t *dat)
{
    uint8_t val[12];
    val[0]  = (dat[0] >> 0) | ((dat[1] & 0x01) << 7);

    val[1]  = (dat[1] >> 1) | ((dat[2] & 0x03) << 6);

    val[2]  = (dat[2] >> 2) | ((dat[3] & 0x07) << 5);

    val[3]  = (dat[3] >> 3) | ((dat[4] & 0x0F) << 4);

    val[4]  = (dat[4] >> 4) | ((dat[5] & 0x1F) << 3);

    val[5]  = (dat[5] >> 5) | ((dat[6] & 0x3F) << 2);

    val[6]  = (dat[6] >> 6) | ((dat[7] & 0x7F) << 1);

    val[7]  = (dat[8] >> 0) | ((dat[9] & 0x01) << 7);

    val[8]  = (dat[9] >> 1) | ((dat[10] & 0x03) << 6);

    val[9]  = (dat[10] >> 2) | ((dat[11] & 0x07) << 5);

    val[10] = (dat[11] >> 3) | ((dat[12] & 0x0F) << 4);

    val[11] = (dat[12] >> 4) | ((dat[13] & 0x1F) << 3);

    /*原RFID数据是从第2个开始的*/
    memcpy(&dat[2],val,12);
}
static RFID_state protocolStateMachine(uint8_t c,uint16_t deviceNum)
{

    RFID_instance_t * pinst;
    RFID_stateMachine_t *inst;
    unsigned char calCRC;

    pinst =getInstanceByDeviceNum(deviceNum);
    inst = &(pinst->sminst);

    switch(inst->state)
    {
        case Prefix0:
            if(PREFIX0_CODE == c)
            {
                inst->state = Prefix1;
            }
            break;
        case Prefix1:
            if(PREFIX1_CODE == c)
            {
                inst->state = Data;
                inst->msgLen = 14;
                inst->type = 0x97;
            }
            else
            {
                inst->state = Prefix0;
            }
            break;
        case Data:
            inst->msg[inst->curLen++] = c;
            if(inst->msgLen == inst->curLen)
            {
                inst->state = CRC;
            }
            break;
        case CRC:
            inst->crc = c;
            inst->state = END1;
            break;
        case END1:
            calCRC = calculateCRC(inst->msg, inst->msgLen);
            if(inst->crc == (calCRC & 0x7f))
            {
                if (0x0D == c)
                {
                    inst->state = END2;
                }
                else {
                    RFIDstateMachineReset(inst);
                }
            }
            else
            {
                Log_warning2("RFID CRC error, recv %d ,calc %d\n",  inst->crc, calCRC);
                RFIDstateMachineReset(inst);

            }
            break;

        case END2:
            if(0x0A == c)
            {
                if(pinst->callback==0)
                {
                    Log_error0("RFID state machine error, need to register call back fucntion\n");
                }
                else{
                    code128ToRFID(inst->msg);
                    pinst->callback(deviceNum, inst->type, inst->msg, inst->msgLen);
                }

                inst->state = Prefix0;
            }
			RFIDstateMachineReset(inst);
            break;


        default:
            Log_error1("RFID state machine error, unknown state: %d\n",  inst->state);

    }
    return inst->state;
}


#endif
/*****************************************************************************
 * 函数名称: void RFIDProcess(uint16_t deviceNum)
 * 函数说明: 处理串口输入字符
 * 输入参数:
 *        deviceNum：设备号，对应的是设备配置表rfid_cfg_table中的数组下标
 * 输出参数: 无
 * 返 回 值: 无
 * 备注: 从Mailbox中取出所有接收到的字符，输入到状态机中处理串口协议
*****************************************************************************/
void RFIDProcess(uint16_t deviceNum)
{
	int i;
	RFID_instance_t * pinst;
    uartDataObj_t uartDataObj;

	pinst =getInstanceByDeviceNum(deviceNum);

    Mailbox_pend(pinst->recvMbox, (Ptr *)&uartDataObj, BIOS_WAIT_FOREVER);

	for(i=0;i<uartDataObj.length;i++)
	{
        protocolStateMachine(uartDataObj.buffer[i], deviceNum );
    }


}

static void uartRFIDIntrHandler(void *callBackRef, u32 event, unsigned int eventData)
{
//	u8 Errors;
	u16 UartDeviceNum = *((u16 *)callBackRef);
	u16 RFIDDeviceNum;
//    u8 *NextBuffer;
    RFID_instance_t * pinst;


    RFIDDeviceNum = getRFIDnumByUartNum(UartDeviceNum);

    pinst =getInstanceByDeviceNum(RFIDDeviceNum);

	if (event == XUN_EVENT_SENT_DATA) {

	}

	if (event == XUN_EVENT_RECV_DATA || event == XUN_EVENT_RECV_TIMEOUT) {
        rfidUartDataObj.length = eventData;
        Mailbox_post(pinst->recvMbox, (Ptr *)&rfidUartDataObj, BIOS_NO_WAIT);
        UartNs550Recv(UartDeviceNum, rfidUartDataObj.buffer, UART_REC_BUFFER_SIZE);
	}

	if (event == XUN_EVENT_RECV_ERROR) {
//		Errors = UartNs550GetLastErrors(UartDeviceNum);
	}
}



