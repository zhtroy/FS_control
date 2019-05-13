/*
 * NetPacket.c
 *
 *  Created on: 2019-2-27
 *      Author: zhtro
 */

#include "Sensor/CellCommunication/NetPacket.h"
#include "string.h"
#include <xdc/runtime/System.h>

/* Network Support Macros for "Unix-like" functions */
#ifdef BIGENDIAN
#define htons(a) (a)
#define htonl(a) (a)
#define ntohl(a) (a)
#define ntohs(a) (a)
#else
#define htons(a) ( (((a)>>8)&0xff) + (((a)<<8)&0xff00) )
/*
 * Fix warning when compiling for IAR (SDOCM00103001):
 *
 *     Warning[Pe061]: integer operation result is out of range
 *
 * This macro has been updated to perform masking operations before shifting.
 * In its previous form (which shifts THEN masks), the IAR compiler generated
 * a warning because it did not like shaving off bits, as it is a (potential)
 * accidental loss of data.  Changing the code to mask first (purposefully
 * losing the data) then shifting afterward fixes the warning.
 *
 * Note that the TI and GCC compilers never cared about this ...
 *
 */
#define htonl(a) ((((a) & 0xff000000) >> 24) | (((a) & 0x00ff0000) >> 8) | \
                  (((a) & 0x0000ff00) << 8)  | (((a) & 0x000000ff) << 24) )

#define ntohl(a) htonl(a)
#define ntohs(a) htons(a)
#endif


/*
 * 将32bytes的原始网络数据填充到net_packet_t结构体中,注意网络字节序和主机字节序的转化
 */
net_packet_t* NetPacketBuildHeaderFromRaw(net_packet_t* p, char* raw)
{
	memcpy(((char *) p), (void*) raw, HDR_LEN);
	NetPacketToHostOrder(p);

	return p;
}

/*
 * 将主机字节序的net_packet_t转换成网络字节序
 */
void NetPacketToNetOrder(net_packet_t * p)
{

	p->cks = htons(p->cks);
	p->len = htons(p->len);
	p->cmd = htons(p->cmd);
	p->reqid = htonl(p->reqid);
	p->srcid = htonl(p->srcid);
	p->dstid = htonl(p->dstid);
	p->opt = htons(p->opt);

}

/*
 * 将网络字节序的net_packet_t转换成主机字节序
 * 返回总长度
 */
void NetPacketToHostOrder(net_packet_t * p)
{
	p->cks = ntohs(p->cks);
	p->len = ntohs(p->len);
	p->cmd = ntohs(p->cmd);
	p->reqid = ntohl(p->reqid);
	p->srcid = ntohl(p->srcid);
	p->dstid = ntohl(p->dstid);
	p->opt = ntohs(p->opt);

}

/*
 * net_packet_t 构造器
 */
net_packet_t* NetPacketCtor(net_packet_t* p,
							  uint8_t flag,
							  uint16_t cmd,
							  uint32_t req,
							  uint32_t src_cab,
							  uint32_t dst_cab,
							  const char * p_data,
							  uint16_t data_len)
{
	if(data_len>PACKET_DATA_MAX_LEN){
		System_abort("packet data too long");
	}
	memset(p,0,sizeof(net_packet_t));
	p->start[0] =  CELL_START0;
	p->start[1] =  CELL_START1;
	p->start[2] =  CELL_START2;
	p->start[3] =  CELL_START3;
	p->cks = 	   0xFFFF;
	p->ver = 	   0x01;
	p->flag = 	   flag;
	p->len = HDR_LEN + data_len;
	p->cmd = cmd;
	p->reqid = req;
	p->srcid = src_cab;
	p->dstid = dst_cab;


	memcpy(p->data, p_data, data_len);


	return p;
}
