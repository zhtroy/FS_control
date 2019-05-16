/*
 * PacketHandler.c
 *
 *  Created on: 2019-5-16
 *      Author: zhtro
 */
#include "Sensor/CellCommunication/CellDriver.h"
#include "Lib/bitop.h"

void PacketHandlerUpdateRoute(cell_packet_t * p)
{
	uint8_t flag;

	/*
	 * TODO: 更新路径数据
	 */

	/*
	 * 发送响应包
	 */
	/*
	 * 这里直接使用传进来的p指针，避免在task的栈上分配packet内存
	 */
	BF_SET(flag,CELL_TYPE_ALLOW,0,2);
	CellPacketCtor(p, flag, CELL_CMD_UPDATEROUTE,p->reqid,p->dstid,
					p->srcid, null, 0);
	CellSendPacket(p);
}
