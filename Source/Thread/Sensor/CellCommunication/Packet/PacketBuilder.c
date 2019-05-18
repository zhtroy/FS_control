/*
 * PacketBuilder.c
 *
 * 构造发送报文
 *
 *  Created on: 2019-5-18
 *      Author: zhtro
 */
#include "Sensor/CellCommunication/Packet/PacketDef.h"
#include "Sensor/CellCommunication/CellDriver.h"
#include "Lib/bitop.h"

void PacketBuildCabStateChange(cell_packet_t * packet,uint32_t reqid, uint32_t srcid, uint32_t dstid, packet_cabstatechange_t statechange)
{
	uint8_t flag = 0;

	/*
	 * 设置flag
	 */
	BF_SET(flag, CELL_TYPE_REQ,0,2);
	/*
	 * 更换成网络字节序,每种包都要手动的转换结构体
	 */
	statechange.cs = htonl(statechange.cs);
	statechange.ps = htonl(statechange.ps);
	statechange.nid = htonll(statechange.nid);

	CellPacketCtor(packet, flag,CELL_CMD_CABSTATECHANGE,reqid,srcid,dstid,&statechange,sizeof(statechange));
}

