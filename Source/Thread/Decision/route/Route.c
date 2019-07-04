/*
 * Route.c
 *
 *  Created on: 2019-5-28
 *      Author: zhtro
 */

#include "stdint.h"
#include "Decision/route/Route.h"
#include "Lib/vector.h"
#include "Lib/bitop.h"
#include "logLib.h"
#include "stdio.h"

static packet_routenode_t * m_vnode = 0;

/*
 * 将路径打印到串口
 */
void RouteShow(packet_routenode_t * vnode)
{
	char outputbuff[100];
	/*
	 * 将收到的路径打印出来
	 */
	LogMsg("\n current route (%d):\n====================\n",vector_size(vnode));

	if(RouteHasOngoing())
	{
		size_t i;
		for(i=0; i<vector_size(vnode); i++)
		{
			LogMsg("nid: %llu\t NT:%d\t WS:%d\n",
					vnode[i].nid,
					BF_GET(vnode[i].flag,4,4),
					BF_GET(vnode[i].flag,0,4));

		}

	}
	else
	{
		LogMsg("EMPTY\n");
	}
	LogMsg("=====================\n");
}
/*
 * API
 */

/*
 * 更新路径
 * vnode: 路径列表
 */
void RouteUpdate(packet_routenode_t * vnode)
{
	RouteFree();
	m_vnode = vnode;

	RouteShow(m_vnode);
}

/*
 * 更新路径（拷贝的方式）
 */
void RouteUpdateCopy(packet_routenode_t * vnode)
{
	size_t i;

	RouteFree();
	for(i = 0;i< vector_size(vnode);i++)
	{
		vector_push_back( m_vnode , vnode[i]);
	}

	RouteShow(m_vnode);
}

/*
 * 从路径队列头部弹出一个节点
 */
packet_routenode_t RoutePop()
{
	if(!RouteHasOngoing())
		return;

	packet_routenode_t head = * (vector_begin(m_vnode));
	vector_erase(m_vnode,0);

	RouteShow(m_vnode);

	return head;
}

/*
 * 查看当前路径中第一个节点
 */
packet_routenode_t RoutePeek()
{
	if(!RouteHasOngoing())
		return;

	return *(vector_begin(m_vnode));
}

/*
 * 返回是否有正在执行中的路径
 */
uint8_t RouteHasOngoing()
{
	return (m_vnode!=0 && !vector_empty(m_vnode));
}

/*
 * 返回NT域
 */
uint8_t RouteGetNodeNT(packet_routenode_t node)
{
	return BF_GET(node.flag, 4, 4);
}

/*
 * 返回WS域
 */
uint8_t RouteGetNodeWS(packet_routenode_t node)
{
	return BF_GET(node.flag, 0, 4);
}

/*
 * 删除路径
 */
void RouteFree()
{
	if(m_vnode !=0)
	{
		vector_free(m_vnode);
	}
	m_vnode = 0;

	RouteShow(m_vnode);
}

/*
 * 往路径后添加点
 */
void RouteAddNode(packet_routenode_t node)
{
	vector_push_back(m_vnode,node);

	RouteShow(m_vnode);
}

/*
 * 改变停站点，即路径终点
 */
void RouteChangeDestination(packet_routenode_t node)
{

	if(!RouteHasOngoing())
		return;

	vector_pop_back(m_vnode);
	vector_push_back(m_vnode,node);

	RouteShow(m_vnode);
}

/*
 * 获取路径终点
 */
packet_routenode_t RouteGetDestination()
{
	if(!RouteHasOngoing())
		return;
	return m_vnode[vector_size(m_vnode)-1];
}


