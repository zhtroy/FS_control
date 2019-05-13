/*
 * CellCommunication.c
 *  4G 模块
 *
 *  Created on: 2018-12-8
 *      Author: zhtro
 */

#include <xdc/std.h>
#include "DSP_Uart/dsp_uart2.h"
#include "soc_C6748.h"
#include "stdio.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/BIOS.h>
#include "stdint.h"
#include "interrupt.h"
#include <xdc/runtime/System.h>
#include "Message/Message.h"
#include "Sensor/CellCommunication/CellDriver.h"
#include "Sensor/CellCommunication/NetPacket.h"
#include "common.h"




static Semaphore_Handle sem_cell_data_received = 0;
static Mailbox_Handle m_mbox_hdl = 0;
static int8_t cell_data_buffer[CELL_BUFF_SIZE];
static uint16_t buff_head = 0;
static uint16_t buff_tail = 0;


static void CellUartHandler(uint32_t event, unsigned char data)
{
	 if(UART_INTID_RX_DATA == event)
	{
		cell_data_buffer[buff_tail] = data;
		buff_tail = (buff_tail+1) % CELL_BUFF_SIZE;
		Semaphore_post(sem_cell_data_received);
	}
}

static void SemInit()
{
	Semaphore_Params semParams;

	//创建信号量
	Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_COUNTING;
	sem_cell_data_received = Semaphore_create(0, &semParams, NULL);
}


/*
 * 接收task
 */
static Void taskCellReceive(UArg a0, UArg a1)
{


	int8_t c;
	cell_state_t state = cell_wait;
	net_packet_t packet;
	int div_num = 0;
	int recv_head_num = 0;
	char headbuff[HDR_LEN];
	int recv_data_num = 0;

	while(1)
	{
		Semaphore_pend(sem_cell_data_received, BIOS_WAIT_FOREVER);
		c = cell_data_buffer[buff_head];
		switch(state)
		{
			case cell_wait:
			{
				if(c == CELL_START0)
				{
					headbuff[recv_head_num] = c;
					recv_head_num++;
					state = cell_start0;
				}

				break;
			}
			case cell_start0:
			{
				if(c == CELL_START1)
				{
					headbuff[recv_head_num] = c;
					recv_head_num++;
					state = cell_start1;
				}
				else
				{
					recv_head_num=0;
					state = cell_wait;
				}
				break;
			}
			case cell_start1:
			{
				if(c == CELL_START2)
				{
					headbuff[recv_head_num] = c;
					recv_head_num++;
					state = cell_start2;
				}
				else
				{
					recv_head_num=0;
					state = cell_wait;
				}
				break;
			}
			case cell_start2:
			{
				if(c == CELL_START3)
				{
					headbuff[recv_head_num] = c;
					recv_head_num++;
					state = cell_recv_head;
				}
				else
				{
					recv_head_num=0;
					state = cell_wait;
				}
				break;
			}
			case cell_recv_head:
				headbuff[recv_head_num] = c;
				recv_head_num++;
				if(recv_head_num>=HDR_LEN)  //4字节start
				{
					NetPacketBuildHeaderFromRaw(&packet,headbuff);
					//TODO: check CRC
					recv_head_num=0;
					state = cell_recv_data;
				}
				break;

			case cell_recv_data:
				packet.data[recv_data_num] = c;
				recv_data_num++;
				if(recv_data_num >= packet.len-HDR_LEN)
				{
					recv_data_num = 0;
					state = cell_wait;
					//接收到一个完整包，发送到邮箱
					Mailbox_post(m_mbox_hdl, (Ptr *)&packet, BIOS_NO_WAIT);

				}

		} //switch

		buff_head = (buff_head+1) % CELL_BUFF_SIZE;
	}
}
/****************************************************************************/
/*                                                                          */
/*              函数定义                                                        */
/*                                                                          */
/****************************************************************************/

void CellInit()
{
	Task_Handle task;
	Task_Params taskParams;
	Mailbox_Params mboxParams;

	SemInit();

	/* 注册UART2 中断回调函数 */
	UART2RegisterHandler(CellUartHandler);

	/*创建接收task*/
	Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(taskCellReceive, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	/*创建接收邮箱*/

    Mailbox_Params_init(&mboxParams);
    m_mbox_hdl = Mailbox_create (sizeof (net_packet_t),CELL_MAILBOX_DEPTH, &mboxParams, NULL);

}

Bool CellRecvPacket(net_packet_t * packet,UInt timeout)
{
	Bool ret;

	ret = Mailbox_pend(m_mbox_hdl,(Ptr*) packet, timeout);

	return ret;
}

void CellSendPacket(net_packet_t * packet)
{
	int len = packet->len;
	//转换成网络字节序
	NetPacketToNetOrder(packet);
	//使用阻塞发送
	UART2Send(&packet, len);
	//在发送完成后，转回主机字节序
	NetPacketToHostOrder(packet);
}






