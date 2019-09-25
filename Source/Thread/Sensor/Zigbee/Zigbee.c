/*
 * Zigbee.c
 *
 *  Created on: 2019-4-14
 *      Author: zhtro
 */

#include "Zigbee/Zigbee.h"
#include "ZCP/zcp_driver.h"
#include "uartns550.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <xdc/runtime/System.h>
#include "stdint.h"
#include "string.h"
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/BIOS.h>
#include "Message/Message.h"

#define ZIGBEE_MBOX_DEPTH (32)

static Semaphore_Handle m_sem_zigbee_send;
static Mailbox_Handle recvMbox;
static uartDataObj_t recvDataObj;





static void UartZigbeeIntrHandler(void *CallBackRef, u32 Event, unsigned int EventData)
{
	u8 Errors;
	u16 UartDeviceNum = *((u16 *)CallBackRef);
	/*
	 * All of the data has been sent.
	 */
	if (Event == XUN_EVENT_SENT_DATA) {
//		TotalSentCount = EventData;
		Semaphore_post(m_sem_zigbee_send);
	}

	/*
	 * All of the data has been received.
	 */
	/*
	 * Data was received, but not the expected number of bytes, a
	 * timeout just indicates the data stopped for 4 character times.
	 */
	if (Event == XUN_EVENT_RECV_DATA || Event == XUN_EVENT_RECV_TIMEOUT) {
	    recvDataObj.length = EventData;
        Mailbox_post(recvMbox, (Ptr *)&recvDataObj, BIOS_NO_WAIT);
        UartNs550Recv(UartDeviceNum, &recvDataObj.buffer, UART_REC_BUFFER_SIZE);
	}

	/*
	 * Data was received with an error, keep the data but determine
	 * what kind of errors occurred.
	 */
	if (Event == XUN_EVENT_RECV_ERROR) {
//		TotalReceivedCount = EventData;
//		TotalErrorCount++;
		Errors = UartNs550GetLastErrors(UartDeviceNum);
	}
}

void ZigbeeInit()
{
	Semaphore_Params semParams;
	Mailbox_Params mboxParams;




    /* 初始化接收邮箱 */
    Mailbox_Params_init(&mboxParams);
    recvMbox = Mailbox_create (sizeof (uartDataObj_t),ZIGBEE_MBOX_DEPTH, &mboxParams, NULL);

	Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_COUNTING;
	m_sem_zigbee_send = Semaphore_create(1, &semParams, NULL);

	/*初始化FPGA串口*/
	UartNs550Init(ZIGBEE_UART_NUM, UartZigbeeIntrHandler);
	UartNs550Recv(ZIGBEE_UART_NUM, recvDataObj.buffer, UART_REC_BUFFER_SIZE);

}

/*
 * zigbee遥控发送回上位机软件
 * pData :包指针
 * type :包类型
 * len :包长度
 */
void ZigbeeSend(void * pData,uint8_t type, int len)
{

	/*
	 * 数据包格式为 包头(0xAA 0x42 0x55) + 类型 + 长度  + 数据 + CRC8 + 包尾(0x0D 0x99 0xCC)
	 */
	uint8_t  sendbuff[512];
	int bufflen = len + 9;

	sendbuff[0] = 0xAA;
	sendbuff[1] = 0x42;
	sendbuff[2] = 0x55;
	sendbuff[3] = type;
	sendbuff[4] = len;
	sendbuff[bufflen-3] = 0x0D;
	sendbuff[bufflen-2] = 0x99;
	sendbuff[bufflen-1] = 0xCC;

	if(pData && len)
	{
		memcpy(sendbuff+5, pData, len );
	}

	/*
	 * 插入CRC8
	 */
	sendbuff[bufflen-4] = ZCPCrcCalc(pData, len);

	Semaphore_pend(m_sem_zigbee_send, BIOS_WAIT_FOREVER);

	UartNs550Send(ZIGBEE_UART_NUM, sendbuff, bufflen);
}

Void taskZigbee(UArg a0, UArg a1)
{
	int state = 0;
	int i;
	uint8_t c;
	p_msg_t pMsg;
	int recvLen = 0;
	int dataLen;
	uartDataObj_t buffObj;
	uint8_t dataBuff[MSGSIZE];

	ZigbeeInit();

	while(1)
	{
		Mailbox_pend(recvMbox,(Ptr*) &buffObj, BIOS_WAIT_FOREVER);

		for(i = 0;i<buffObj.length;i++)
		{
			c = buffObj.buffer[i];
			switch(state)
			{
				case 0:  //等待包头 0xAA
				{
					if(0xAA == c)
					{
						state = 1;
					}
					break;
				}
				case 1:
				{
                    if(0x42==c)
                    {
                    	state = 2;
                    }
                    else
                    {
                    	state = 0;
                    }
                    break;
				}
				case 2:
				{
                    if(0x55==c)
                    {
                    	state = 3;
                    }
                    else
                    {
                    	state = 0;
                    }
                    break;
				}
				case 3://接收长度
				{
					recvLen = 0;
					dataLen = c;
					memset(dataBuff,0,MSGSIZE);
					state = 4;
					break;
				}
				case 4:  //接收命令
				{
					dataBuff[recvLen++] = c;
					if(recvLen>=dataLen)
					{
						state = 5;
					}
					break;
				}

				case 5:
				{
					state = 0;

					if(0x0D == c)
					{
						System_printf("zigbee recv cmd:%d\n", pMsg->data[0]);

						pMsg = Message_getEmpty();
						pMsg->type = zigbee;
						memcpy(pMsg->data,dataBuff, recvLen);
						pMsg->dataLen = recvLen;

						Message_post(pMsg);
					}

					break;
				}


			} // switch(state)
		}

	} /*  while(1)  */
}

