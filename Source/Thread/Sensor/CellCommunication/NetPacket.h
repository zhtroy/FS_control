/*
 * NetPacket.h
 *
 *  Created on: 2019-2-27
 *      Author: zhtro
 */

#ifndef NETPACKET_H_
#define NETPACKET_H_

#include "stdint.h"

/*
 * 通信包定义
 */
#define HDR_LEN (32)

/*包头*/
#define	CELL_START0 (0x00)
#define	CELL_START1 (0x00)
#define	CELL_START2 (0x00)
#define	CELL_START3 (0x01)

/*
 * 驱动内部定义
 */
#define PACKET_DATA_MAX_LEN (1024)



typedef struct net_packet{
	//header part
	uint8_t 	start[4];
	uint16_t	cks;
	uint8_t		ver;
	uint8_t		flag;
	uint16_t	len;
	uint16_t	cmd;
	uint32_t	reqid;
	uint32_t	srcid;
	uint32_t	dstid;
	uint16_t 	opt;
	uint8_t		rsv[6];
	/*data part TODO:使用的固定大小data*/
	char 		data[PACKET_DATA_MAX_LEN];
}net_packet_t;

net_packet_t* NetPacketBuildHeaderFromRaw(net_packet_t* p, char* raw);

net_packet_t* NetPacketCtor(net_packet_t* p,
							  uint8_t flag,
							  uint16_t cmd,
							  uint32_t req,
							  uint32_t src_cab,
							  uint32_t dst_cab,
							  const char * p_data,
							  uint16_t data_len);

void NetPacketToNetOrder(net_packet_t * p);
void NetPacketToHostOrder(net_packet_t * p);

#endif /* NETPACKET_H_ */
