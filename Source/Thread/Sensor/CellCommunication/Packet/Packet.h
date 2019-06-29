/*
 * PacketBuilder.h
 *
 *  Created on: 2019-5-18
 *      Author: zhtro
 */

#ifndef PACKETBUILDER_H_
#define PACKETBUILDER_H_


#include "stdint.h"
#include "Sensor/CellCommunication/CellDriver.h"
#include "Decision/route/Route.h"

typedef struct {
	uint64_t nid;
	double offset;
	double speed;
	double acc;
	uint64_t ts;
	uint32_t state;
	uint32_t reserved;
}packet_cabstatus_t;

typedef struct {
	uint64_t nid;
	uint32_t ps;
	uint32_t cs;
}packet_cabstatechange_t;




/*
 * PacketBuild
 */
void PacketBuildCabStateChange(cell_packet_t * packet,uint32_t reqid, uint32_t srcid, uint32_t dstid, packet_cabstatechange_t statechange);
void PacketBuildCabPulse(cell_packet_t * packet, uint32_t reqid, uint32_t srcid, uint32_t dstid);
void PacketBuildCabStatus(cell_packet_t * packet, uint32_t reqid, uint32_t srcid, uint32_t dstid, packet_cabstatus_t cabstatus);


/*
 * PacketResolve
 */
packet_routenode_t * PacketResolveUpdateRoute(cell_packet_t * packet);

#endif /* PACKETBUILDER_H_ */
