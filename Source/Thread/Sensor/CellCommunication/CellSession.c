/*
 * CellSession.c
 *
 * 负责从串口接收网络数据包，并维护一个session list
 *
 *  Created on: 2019-5-13
 *      Author: zhtro
 */


#include <xdc/std.h>
#include "Sensor/CellCommunication/CellDriver.h"
#include "Sensor/CellCommunication/CellSession.h"
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/MailBox.h>
#include <ti/sysbios/knl/Task.h>


/*
 * session list
 */
#define CELL_SESSION_NUM (16)
static cell_session_t m_sessions[CELL_SESSION_NUM];
static Mailbox_Handle m_mailbox = 0;


static void CellSessionClose(cell_session_t * ps)
{
	ps->isOpen = 0;
	Clock_delete(ps->clock);
}
/*
 * session超时
 */
static xdc_Void CellSessionTimeout(xdc_UArg arg)
{
	int index = (int) arg;

	m_sessions[index].timeouthandler();

	CellSessionClose( &(m_sessions[index]));
}

/*
 * 收到回复后，进行session的后续处理
 */
static void CellSessionHandle(cell_packet_t * p)
{
	int i;
	uint8_t type;

	for(i = 0; i<CELL_SESSION_NUM; i++)
	{
		if(m_sessions[i].isOpen)
 		{
			if(m_sessions[i].cmd == p->cmd &&
			   m_sessions[i].reqid == p->reqid)
			{
				/*
				 * 根据回复类型，执行相应的回调
				 */
				type = CellPacketGetType(p);
				if(type == CELL_TYPE_ALLOW)
				{
					m_sessions[i].allowhandler(p);
				}
				else if (type == CELL_TYPE_DENY)
				{
					m_sessions[i].denyhandler(p);
				}
				/*
				 * 关闭session
				 */
				CellSessionClose( &(m_sessions[i]));
				break;
			}

		}
	}
}

static Void taskSessionManage(UArg a0, UArg a1)
{
	cell_packet_t packet;

	while(1)
	{
		CellRecvPacket(&packet, BIOS_WAIT_FOREVER);

		switch(CellPacketGetType(&packet))
		{
			case CELL_TYPE_NONE:
			case CELL_TYPE_REQ:
			{

				break;
			}//case CELL_TYPE_REQ:

			case CELL_TYPE_ALLOW:
			case CELL_TYPE_DENY:
			{
				CellSessionHandle(&packet);
			}
		}
	}
}


/*
 * API
 */

void CellSessionInit(Mailbox_Handle mb)
{

	Task_Handle task;
	Task_Params taskParams;

	//初始化驱动
	CellDriverInit();

	/*创建task*/
	Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(taskSessionManage, &taskParams, NULL);
	if (task == NULL) {
		BIOS_exit(0);
	}

	/*
	 * 初始化session list
	 */
	memset(&m_sessions,0 , sizeof(m_sessions));

	m_mailbox = mb;

}


/*
 * 开始一个session
 * 返回1 成功
 * 返回0 失败
 */
uint8_t CellSessionOpen(//报文指令
					uint16_t cmd,
					//和通讯报文头部的reqid一致
					uint16_t reqid,
					//超时timeout后，关闭这个session
					uint32_t timeout,
					//拒绝后的回调函数
					DenyHandler denyhandler,
					//同意后的回调函数
					AllowHandler allowhandler,
					TimeoutHandler timeouthandler)
{

	int i;
	Clock_Params clockParams;
	/*
	 * 找到一个空的session
	 */
	for(i=0;i<CELL_SESSION_NUM;i++)
	{
		if(!m_sessions[i].isOpen)
		{
			m_sessions[i].cmd = cmd;
			m_sessions[i].reqid = reqid;
			m_sessions[i].timeout = timeout;
			m_sessions[i].denyhandler = denyhandler;
			m_sessions[i].allowhandler = allowhandler;
			m_sessions[i].timeouthandler = timeouthandler;
			m_sessions[i].isOpen = 1;
			/*
			 * 设定一个timeout clock
			 */

			Clock_Params_init(&clockParams);
			clockParams.period = 0;       // one shot
			clockParams.startFlag = TRUE;
			clockParams.arg = i;
			m_sessions[i].clock = Clock_create(CellSessionTimeout, timeout, &clockParams, NULL);

			return 1;
		}
	}

	return 0;

}
