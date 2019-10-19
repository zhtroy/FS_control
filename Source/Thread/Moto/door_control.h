/*
 * door_control.h
 *
 *  Created on: 2019-10-19
 *      Author: zhtro
 */

#ifndef DOOR_CONTROL_H_
#define DOOR_CONTROL_H_

#include "stdint.h"

#define DOOR_CTRL_CAN_DEV (3)
#define DOOR_STATE_OPEN 	(1)
#define DOOR_STATE_CLOSE 	(0)
#define DOOR_OP_TIMEOUT     (10000)

extern uint8_t DoorGetState();
extern uint8_t DoorOpen();
extern uint8_t DoorClose();
extern void DoorCtrlInit();

#endif /* DOOR_CONTROL_H_ */
