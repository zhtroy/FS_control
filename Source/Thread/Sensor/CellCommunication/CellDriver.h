/*
 * CellCommunication.h
 *
 *  Created on: 2018-12-31
 *      Author: zhtro
 */

#ifndef CELLDRIVER_H_
#define CELLDRIVER_H_

#include "stdint.h"
#include <xdc/std.h>


/*
 * 驱动内部定义
 */
#define PACKET_DATA_MAX_LEN (1024)

/*中断接收buffer大小*/
#define CELL_BUFF_SIZE (128)
/*
 * 接收邮箱深度
 */
#define CELL_MAILBOX_DEPTH    (4)

/*
 * 驱动内部定义 end
 */

/*
 * 通信包定义
 */

/*包头*/
#define CELL_HEADER_LEN (32)
#define	CELL_START0 (0x46)
#define	CELL_START1 (0x46)
#define	CELL_START2 (0x53)
#define	CELL_START3 (0x53)


/*
 * cmd字段定义
 */
#define CELL_CMD_CABSTATUS  		(0x1000)
#define CELL_CMD_CABPULSE			(0x1001)
#define CELL_CMD_CABSTATECHANGE		(0x1002)
#define CELL_CMD_CABREQUESTROUTE	(0x1003)

#define CELL_CMD_UPDATEROUTE		(0x2000)
#define CELL_CMD_ORDERCAB			(0x2001)

/*
 * type 定义
 */
#define CELL_TYPE_NONE 	(0)
#define CELL_TYPE_REQ	(1)
#define CELL_TYPE_ALLOW (2)
#define CELL_TYPE_DENY 	(3)

/*
 * 地址定义
 */
#define CELL_ADDR_VRC_MAIN (0x01)
#define CELL_ADDR_VRC_BACKUP (0x02)
#define CELL_ADDR_TESTCAR (0x100)
#define SESSIONID (0x100)

typedef struct cell_packet{
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
}cell_packet_t;

/*
 * 通信包定义 end
 */

/*
 * API
 */

cell_packet_t* CellPacketCtor(cell_packet_t* p,
							  uint8_t flag,
							  uint16_t cmd,
							  uint32_t req,
							  uint32_t src_cab,
							  uint32_t dst_cab,
							  const char * p_data,
							  uint16_t data_len);

void CellPacketBuildResponse(const cell_packet_t * req,
							 cell_packet_t * response,
							 uint8_t allowOrDeny,
							 const char * data,
							 uint16_t data_len);

uint8_t CellPacketGetType(cell_packet_t* p);

void CellDriverInit();
void CellSendPacket(cell_packet_t * packet);
Bool CellRecvPacket(cell_packet_t * packet,UInt timeout);




#endif /* CELLDRIVER_H_ */
