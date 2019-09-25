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

//车辆信息反馈
#define ZIGBEE_PACKET_CARINFO  (1)

//车辆参数
#define ZIGBEE_PACKET_PARAM    (2)

//车辆参数设置成功
#define ZIGBEE_PACKET_PARAM_FINISH    (3)
/*
 * API
 */
extern void ZigbeeInit();
extern void ZigbeeSend(void * pData,uint8_t type, int len);

extern Void taskZigbee(UArg a0, UArg a1);


#endif /* ZIGBEE_H_ */
