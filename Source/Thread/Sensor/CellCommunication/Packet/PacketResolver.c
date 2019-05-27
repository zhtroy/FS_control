/*
 * PacketResolve.c
 *
 *  Created on: 2019-5-23
 *      Author: zhtro
 */


#include "Sensor/CellCommunication/Packet/Packet.h"
#include "Sensor/CellCommunication/CellDriver.h"
#include "Lib/vector.h"
#include "Lib/bitop.h"
#include "stdio.h"

/**
 * @brief 解析路径报文，返回一个路径节点列表
 * @param packet 接收到的包
 * @return vector<packet_routenode_t> 使用完后需要用vector_free()释放内存
 */
packet_routenode_t * PacketResolveUpdateRoute(cell_packet_t * packet)
{
	packet_routenode_t * vnode = NULL;  //vector of nodes
	packet_routenode_t node;

	int nodeNum;
	int i;

	nodeNum = (packet->len - CELL_HEADER_LEN) / 32 ;

	for(i=0; i<nodeNum; i++)
	{

		/*
		 * 使用拷贝的方式，不改变原来packet的内存
		 */
		memcpy(&node, &(packet->data[i*32]),sizeof(node));

		node.flag = ntohl(node.flag);
//		node.length = ntohll(pnode->length);
		node.nid = ntohll(node.nid);
		node.reserved = ntohl(node.reserved);
//		node.speedlimit = ntohll(pnode->speedlimit);

		vector_push_back(vnode,node);
	}

	return vnode;
}
