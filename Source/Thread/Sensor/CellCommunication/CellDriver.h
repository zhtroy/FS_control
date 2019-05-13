/*
 * CellCommunication.h
 *
 *  Created on: 2018-12-31
 *      Author: zhtro
 */

#ifndef CELLCOMMUNICATION_H_
#define CELLCOMMUNICATION_H_

#include "stdint.h"
#include <xdc/std.h>
#include "Sensor/CellCommunication/NetPacket.h"

/*中断接收buffer大小*/
#define CELL_BUFF_SIZE (128)
/*
 * 接收邮箱深度
 */
#define CELL_MAILBOX_DEPTH    (4)


/*
 * 协议解析状态机的各个状态
 */
typedef enum{
	//等待<START>标志
	cell_wait,
	//接收START标志
	cell_start0,
	cell_start1,
	cell_start2,
	cell_start3,
	//接收HEAD
	cell_recv_head,
	//接收DATA
	cell_recv_data
}cell_state_t;



void CellInit();
void CellSendPacket(net_packet_t * packet);
Bool CellRecvPacket(net_packet_t * packet,UInt timeout);




#endif /* CELLCOMMUNICATION_H_ */
