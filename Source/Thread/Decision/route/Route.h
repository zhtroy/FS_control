/*
 * Route.h
 *
 *	路线,是一个元素为packet_routenode_t 的队列
 *
 *  Created on: 2019-5-28
 *      Author: zhtro
 */

#ifndef ROUTE_H_
#define ROUTE_H_

#include "stdint.h"

#define ROUTE_NT_MOVE (0)
#define ROUTE_NT_STOP (1)
#define ROUTE_NT_START (2)

typedef struct {
	uint64_t nid;
	double length;
	double speedlimit;
	uint32_t reserved;
	uint32_t flag;
}packet_routenode_t;
#pragma STRUCT_ALIGN(packet_routenode_t, 8)

void RouteUpdate(packet_routenode_t * vnode);
packet_routenode_t RoutePop();
packet_routenode_t RoutePeek();
uint8_t RouteHasOngoing();
uint8_t RouteGetNodeNT(packet_routenode_t node);
void RouteFree();
void RouteAddNode(packet_routenode_t node);
void RouteChangeDestination(packet_routenode_t node);
packet_routenode_t RouteGetDestination();

#endif /* ROUTE_H_ */
