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
#include "common.h"
#include "Lib/bitop.h"




static Semaphore_Handle sem_cell_data_received = 0;
static Mailbox_Handle m_mbox_hdl = 0;
static int8_t cell_data_buffer[CELL_BUFF_SIZE];
static uint16_t buff_head = 0;
static uint16_t buff_tail = 0;
static uint8_t m_isInited = 0;

/*
 * 协议解析状态机的各个状态
 */
typedef enum{
	//等待<START>标志
	cell_wait,
	//接收START标志
	cell_start0,
	cell_start1,
	cell_start2,
	cell_start3,
	//接收HEAD
	cell_recv_head,
	//接收DATA
	cell_recv_data
}cell_state_t;


/*
 * static function
 */



/*
 * 将主机字节序的net_packet_t转换成网络字节序
 */
static void CellPacketToNetOrder(cell_packet_t * p)
{

	p->cks = htons(p->cks);
	p->len = htons(p->len);
	p->cmd = htons(p->cmd);
	p->reqid = htonl(p->reqid);
	p->srcid = htonl(p->srcid);
	p->dstid = htonl(p->dstid);
	p->opt = htons(p->opt);

}

/*
 * 将网络字节序的net_packet_t转换成主机字节序
 */
static void CellPacketToHostOrder(cell_packet_t * p)
{
	p->cks = ntohs(p->cks);
	p->len = ntohs(p->len);
	p->cmd = ntohs(p->cmd);
	p->reqid = ntohl(p->reqid);
	p->srcid = ntohl(p->srcid);
	p->dstid = ntohl(p->dstid);
	p->opt = ntohs(p->opt);

}
/*
 * 将32bytes的原始网络数据填充到net_packet_t结构体中,注意网络字节序和主机字节序的转化
 */
static cell_packet_t* CellPacketBuildHeaderFromRaw(cell_packet_t* p, char* raw)
{
	memcpy(((char *) p), (void*) raw, CELL_HEADER_LEN);
	CellPacketToHostOrder(p);

	return p;
}


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
	cell_packet_t packet;
	int div_num = 0;
	int recv_head_num = 0;
	char headbuff[CELL_HEADER_LEN];
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
				if(recv_head_num>=CELL_HEADER_LEN)  //4字节start
				{
					CellPacketBuildHeaderFromRaw(&packet,headbuff);
					//TODO: check CRC
					recv_head_num=0;
					state = cell_recv_data;
				}
				break;

			case cell_recv_data:
				packet.data[recv_data_num] = c;
				recv_data_num++;
				if(recv_data_num >= packet.len-CELL_HEADER_LEN)
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

void CellDriverInit()
{
	Task_Handle task;
	Task_Params taskParams;
	Mailbox_Params mboxParams;

	if(m_isInited)
		return;

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
    m_mbox_hdl = Mailbox_create (sizeof (cell_packet_t),CELL_MAILBOX_DEPTH, &mboxParams, NULL);

    m_isInited = 1;
}

Bool CellRecvPacket(cell_packet_t * packet,UInt timeout)
{
	Bool ret;

	ret = Mailbox_pend(m_mbox_hdl,(Ptr*) packet, timeout);

	return ret;
}

void CellSendPacket(cell_packet_t * packet)
{
	int len = packet->len;
	//转换成网络字节序
	CellPacketToNetOrder(packet);
	//使用阻塞发送
	UART2Send(&packet, len);
	//在发送完成后，转回主机字节序
	CellPacketToHostOrder(packet);
}


/*
 * net_packet_t 构造器
 */
cell_packet_t* CellPacketCtor(cell_packet_t* p,
							  uint8_t flag,
							  uint16_t cmd,
							  uint32_t req,
							  uint32_t src_cab,
							  uint32_t dst_cab,
							  const char * p_data,
							  uint16_t data_len)
{
	if(data_len>PACKET_DATA_MAX_LEN){
		System_abort("packet data too long");
	}
	memset(p,0,sizeof(cell_packet_t));
	p->start[0] =  CELL_START0;
	p->start[1] =  CELL_START1;
	p->start[2] =  CELL_START2;
	p->start[3] =  CELL_START3;
	p->cks = 	   0xFFFF;
	p->ver = 	   0x01;
	p->flag = 	   flag;
	p->len = CELL_HEADER_LEN + data_len;
	p->cmd = cmd;
	p->reqid = req;
	p->srcid = src_cab;
	p->dstid = dst_cab;


	memcpy(p->data, p_data, data_len);


	return p;
}

/*
 * 构造响应包
 */
void CellPacketBuildResponse(const cell_packet_t * req, cell_packet_t * response, uint8_t allowOrDeny, const char * data, uint16_t data_len)
{
	uint8_t flag = 0;

	if(allowOrDeny) //允许
	{
		BF_SET(flag,CELL_TYPE_ALLOW,0,2);
	}
	else
	{
		BF_SET(flag,CELL_TYPE_DENY,0,2);
	}
	CellPacketCtor(response,flag,req->cmd,req->reqid,req->dstid, req->srcid,data,data_len);
}

uint8_t CellPacketGetType(cell_packet_t* p)
{
	return BF_GET(p->flag,0,2);
}





