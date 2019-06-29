/*
 * CellCommunication.h
 *
 *  Created on: 2019-6-28
 *      Author: zhtro
 */

#ifndef CELLCOMMUNICATION_H_
#define CELLCOMMUNICATION_H_

#define CELL_HSM_MAILBOX_DEPTH (16)

/*
 * 发给CarHsm状态机的消息码
 */
#define CELL_MSG_ENTERAUTOMODE   0
#define CELL_MSG_STARTRUN		 1

typedef enum
{
	cell_msg_tick,
	cell_ordercab,
	cell_updateroute,
	carhsm_arrived,
	door_opened
}cellhsm_msg_type_t;

typedef struct{
	cellhsm_msg_type_t type;
}cellhsm_msg_t;



extern void CellHsmPost(cellhsm_msg_type_t type);
extern void CellCommunicationInit();

#endif /* CELLCOMMUNICATION_H_ */
