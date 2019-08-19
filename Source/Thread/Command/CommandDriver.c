/*
 * Command.c
 *
 *  Created on: 2019-7-4
 *      Author: zhtro
 */


#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>
#include "uartns550.h"
#include "Command/CommandDriver.h"
#include "string.h"

static uartDataObj_t uartRxData;
static Mailbox_Handle uartRecvMbox;
static Mailbox_Handle packetRecvMbox;
static 	command_packet_t m_sendPacket;

typedef enum{
	//等待head
	command_wait,
	//接收START标志
	command_head0,
	command_head1,

	command_len,
	command_type,

	//接收
	command_data

}command_state_t;

static void CommandUartIntrHandler(void *callBackRef, u32 event, unsigned int eventData)
{
    uint8_t errors;
    uint16_t uartDeviceNum = *((uint16_t *)callBackRef);

    if (event == XUN_EVENT_SENT_DATA) {

    }

    if (event == XUN_EVENT_RECV_DATA || event == XUN_EVENT_RECV_TIMEOUT) {
        /*
         * step1:获取数据长度
         * step2:Post数据到邮箱，若处理不及时，数据会丢失
         * step3:启动新的串口接收
         */
    	uartRxData.length = eventData;

        Mailbox_post(uartRecvMbox, (Ptr *)&uartRxData, BIOS_NO_WAIT);

        UartNs550Recv(uartDeviceNum, uartRxData.buffer, UART_REC_BUFFER_SIZE);
    }

    if (event == XUN_EVENT_RECV_ERROR) {
        /*
         * 暂不处理
         */
        errors = UartNs550GetLastErrors(uartDeviceNum);
    }
}



static void CommandReciveTask(UArg arg0, UArg arg1)
{
	command_state_t state = command_wait;
	uartDataObj_t recvBuf;
	uint8_t c;
	command_packet_t packet;
	int i;
	int recvDataNum = 0;

    while(1)
    {

        Mailbox_pend(uartRecvMbox, (Ptr *)&recvBuf, BIOS_WAIT_FOREVER);

        for(i = 0;i<recvBuf.length; i++)
        {
        	c = recvBuf.buffer[i];
        	switch(state)
			{
				case command_wait:
				{
					if(c == COMMAND_PACKET_HEAD_0)
					{
						packet.head[0] = c;
						state = command_head0;
					}

					break;
				}
				case command_head0:
				{
					if(c == COMMAND_PACKET_HEAD_1)
					{
						packet.head[1] = c;
						state = command_len;
					}
					else
					{
						state = command_wait;
					}
					break;
				}
				case command_len:
				{
					packet.len = c;
					state = command_type;
					break;
				}
				case command_type:
				{
					packet.type = c;
					state = command_data;
					if(packet.len == 2)
					{
						recvDataNum = 0;
						state = command_wait;
						//接收到一个完整包，发送到邮箱
						Mailbox_post(packetRecvMbox, (Ptr *)&packet, BIOS_NO_WAIT);
					}
					break;
				}
				case command_data:
				{
					packet.data[recvDataNum] = c;
					recvDataNum++;
					if(recvDataNum >= packet.len-2)
					{
						recvDataNum = 0;
						state = command_wait;
						//接收到一个完整包，发送到邮箱
						Mailbox_post(packetRecvMbox, (Ptr *)&packet, BIOS_NO_WAIT);

					}
					break;
				}

			} //switch
        }

	}
}


/*
 * API
 */

void CommandDriverInit()
{
	Task_Handle task;
	Task_Params taskParams;


	//mailbox
	uartRecvMbox = Mailbox_create (sizeof (uartDataObj_t),32, NULL, NULL);
	packetRecvMbox = Mailbox_create (sizeof (command_packet_t),32, NULL, NULL);

	/*初始化FPGA串口*/
	UartNs550Init(COMMAND_UART_NUM, CommandUartIntrHandler);
	UartNs550Recv(COMMAND_UART_NUM, uartRxData.buffer, UART_REC_BUFFER_SIZE);

	//task
	Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(CommandReciveTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}



}


Bool CommandRecv(command_packet_t * packet,UInt timeout)
{
	Bool ret;

	ret = Mailbox_pend(packetRecvMbox,(Ptr*) packet, timeout);

	return ret;
}

void CommandSend(char* data, int datalen, uint8_t type)
{
	m_sendPacket.head[0] = COMMAND_PACKET_HEAD_0;
	m_sendPacket.head[1] = COMMAND_PACKET_HEAD_1;
	m_sendPacket.len = datalen + 2;
	m_sendPacket.type = type;

	memset(m_sendPacket.data,0,COMMAND_PACKET_MAX_LEN);
	if(datalen>0)
	{
		memcpy(m_sendPacket.data, data, datalen );
	}


	UartNs550Send(COMMAND_UART_NUM,&m_sendPacket,datalen+4);  //加上包头，len, type
}





