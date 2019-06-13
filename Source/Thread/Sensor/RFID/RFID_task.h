/*
 * RFID_task.h
 *
 *  Created on: 2019-6-3
 *      Author: zhtro
 */

#ifndef RFID_TASK_H_
#define RFID_TASK_H_

#include "RFID_drv.h"
#include "Sensor/RFID/EPCdef.h"

extern uint8_t * RFIDGetRaw();
extern epc_t RFIDGetEpc();


#endif /* RFID_TASK_H_ */
