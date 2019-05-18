/*
 * Packetdef.h
 *
 *  Created on: 2019-5-16
 *      Author: zhtro
 */

#ifndef PACKETDEF_H_
#define PACKETDEF_H_

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

typedef struct {

};
#endif /* PACKETDEF_H_ */
