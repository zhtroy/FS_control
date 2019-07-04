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
#include <ti/sysbios/knl/Mailbox.h>


extern uint8_t * RFIDGetRaw();
extern epc_t RFIDGetEpc();
extern epc_t RFIDGetLastEpc();
extern uint64_t RFIDGetNID();
Mailbox_Handle RFIDGetV2vMailbox();


#endif /* RFID_TASK_H_ */
