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

static packet_routenode_t * m_vnode = 0;
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
 * 删除路径
 */
void RouteFree()
{
	if(m_vnode !=0)
	{
		vector_free(m_vnode);
	}
	m_vnode = 0;
}

/*
 * 往路径后添加点
 */
void RouteAddNode(packet_routenode_t node)
{
	vector_push_back(m_vnode,node);
}

