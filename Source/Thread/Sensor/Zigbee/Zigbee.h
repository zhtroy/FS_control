/*
 * Zigbee.h
 *
 *  Created on: 2019-4-14
 *      Author: zhtro
 */

#ifndef ZIGBEE_H_
#define ZIGBEE_H_

#include "stdint.h"
#include "xdc/std.h"

/*
 * define
 */
/*FPGA 串口设备号*/
#define ZIGBEE_UART_NUM (3)
/*
 * API
 */
extern void ZigbeeInit();
extern void ZigbeeSend(void * pData, int len);

extern Void taskZigbee(UArg a0, UArg a1);


#endif /* ZIGBEE_H_ */
