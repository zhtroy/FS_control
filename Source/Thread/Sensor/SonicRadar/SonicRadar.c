/*
 * SonicRadar.c
 *
 *  Created on: 2018-12-7
 *      Author: zhtro
 */
#include <xdc/std.h>
#include "uartns550.h"
#include "xil_types.h"
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/BIOS.h>
#include "xdc/runtime/System.h"
#include "Message/Message.h"
#include <ti/sysbios/knl/Task.h>

#define SONIC_UARTNUM 4
#define SONIC_MBOX_DEPTH (16)

/****************************************************************************/
/*                                                                          */
/*         变量                                                        */
/*                                                                          */
/****************************************************************************/
//static Semaphore_Handle sem_sonicRadar;
static Mailbox_Handle recvMbox;
static uint16_t m_distance;
static uartDataObj_t sonicUartDataObj;
static uartDataObj_t recvUartDataObj;
/****************************************************************************/
/*                                                                          */
/*              函数声明                                                        */
/*                                                                          */
/****************************************************************************/

void SonicRadarUartIntrHandler(void *CallBackRef, u32 Event, unsigned int EventData);

/****************************************************************************/
/*                                                                          */
/*              函数定义                                                        */
/*                                                                          */
/****************************************************************************/
Void taskSonicRadar(UArg a0, UArg a1)
{
	uint8_t Sonic_query[3] = {0xe8,0x02,0xbc};
	Mailbox_Params mboxParams;
	p_msg_t pmsg;
	const int INTERVAL = 500;
	Bool result;

	//初始化串口
	UartNs550Init(SONIC_UARTNUM,SonicRadarUartIntrHandler);

	UartNs550Recv(SONIC_UARTNUM, sonicUartDataObj.buffer, UART_REC_BUFFER_SIZE);

	/* 初始化接收邮箱 */
	Mailbox_Params_init(&mboxParams);
	recvMbox = Mailbox_create (sizeof (uartDataObj_t),SONIC_MBOX_DEPTH, &mboxParams, NULL);

	while(1)
	{
		Task_sleep(INTERVAL);

		UartNs550Send(SONIC_UARTNUM, Sonic_query, 3);

		//Semaphore_pend(sem_sonicRadar,BIOS_WAIT_FOREVER);
		result = Mailbox_pend(recvMbox,(Ptr*) &recvUartDataObj, 1000);

		if(!result)
		{
			continue;
		}

		///Buf = UartNs550PopBuffer(SONIC_UARTNUM);

		if(recvUartDataObj.length != 2)
		{
			System_printf("Sonic Radar error: recved %d bytes\n", recvUartDataObj.length);
//			BIOS_exit(0);
		}

		m_distance = recvUartDataObj.buffer[0] * 256 + recvUartDataObj.buffer[1];

		pmsg = Message_getEmpty();
		pmsg->type = sonicradar;
		memcpy( pmsg->data , &m_distance ,sizeof(m_distance));
		pmsg->dataLen = sizeof(m_distance);

		Message_post(pmsg);

	}


}


/*
 * 单位为mm
 */
uint16_t SonicGetDistance()
{
	return m_distance;
}

void SonicRadarUartIntrHandler(void *callBackRef, u32 event, unsigned int eventData)
{
	u8 Errors;
	u16 DeviceNum = *((u16 *)callBackRef);

	/*
	 * All of the data has been sent.
	 */
	if (event == XUN_EVENT_SENT_DATA) {
//		TotalSentCount = EventData;

	}

	/*
	 * All of the data has been received.
	 */
	if (event == XUN_EVENT_RECV_DATA || event == XUN_EVENT_RECV_TIMEOUT) {
//		TotalReceivedCount = EventData;
		sonicUartDataObj.length = eventData;
		Mailbox_post(recvMbox, (Ptr *)&sonicUartDataObj, BIOS_NO_WAIT);
        UartNs550Recv(DeviceNum, sonicUartDataObj.buffer, UART_REC_BUFFER_SIZE);

	}
	/*
	 * Data was received with an error, keep the data but determine
	 * what kind of errors occurred.
	 */
	if (event == XUN_EVENT_RECV_ERROR) {
//		TotalReceivedCount = EventData;
//		TotalErrorCount++;
		Errors = UartNs550GetLastErrors(DeviceNum);
	}
}
