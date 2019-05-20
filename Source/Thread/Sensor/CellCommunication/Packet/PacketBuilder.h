/*
 * PacketBuilder.h
 *
 *  Created on: 2019-5-18
 *      Author: zhtro
 */

#ifndef PACKETBUILDER_H_
#define PACKETBUILDER_H_

#include "Sensor/CellCommunication/Packet/PacketDef.h"

void PacketBuildCabStateChange(cell_packet_t * packet,uint32_t reqid, uint32_t srcid, uint32_t dstid, packet_cabstatechange_t statechange);
void PacketBuildCabPulse(cell_packet_t * packet, uint32_t reqid, uint32_t srcid, uint32_t dstid);
#endif /* PACKETBUILDER_H_ */
