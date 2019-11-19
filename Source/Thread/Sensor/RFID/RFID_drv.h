/*
 * RFID_drv.h
 *
 *  Created on: 2018-11-13
 *      Author: zhtro
 *
 *  RFID 驱动
 */

#ifndef RFID_DRV_H_
#define RFID_DRV_H_

#include "stdint.h"
#include <ti/sysbios/knl/Mailbox.h>


//macro=======================
#define RFID_HEAD  0xBB
#define RFID_END1  0x0D
#define RFID_END2  0x0A
#define RFID_CMD_LOOPCHECK_EPC 0x17

#define RFID_REPONSE_MAX_LEN   128
#define RFID_TX_BUFFER_SIZE  128
#define RFID_MAX_DEVICE 8


//类型定义=====================
#if 0
typedef enum  {
	Ready,
	Head,
	Type,
	Len,
	Data,
	CRC,
	CRCcorrect,
	END1,
	END2,
	Recv
}RFID_state;
#else
typedef enum  {
    Prefix0,
    Prefix1,
    Head,
    Type,
    Len,
    Data,
    CRC,
    CRCcorrect,
    END1,
    END2,
    Recv
}RFID_state;

#define PREFIX0_CODE (0x56)
#define PREFIX1_CODE (0x43)
#define END0_CODE (0x0D)
#define END1_CODE (0x0A)
#endif

typedef struct {
	RFID_state state;
	unsigned char type;
	unsigned char msgLen;
	unsigned char curLen;
	unsigned char msg[RFID_REPONSE_MAX_LEN];
	unsigned char crc;

} RFID_stateMachine_t;

/*
 * deviceNum: RFID 设备号
 * type:     RFID响应包类型
 * data:     RFID数据包
 * len       RFID数据包长度
 */
typedef void (*RFIDcallback) (uint16_t deviceNum, uint8_t type, uint8_t data[], uint32_t len );

typedef struct{
	uint16_t uartDeviceNum ;
	RFIDcallback callback ;
	Mailbox_Handle recvMbox;
	RFID_stateMachine_t sminst;
}RFID_instance_t;



//API-------------------------------//

void RFIDDeviceOpen(uint16_t deviceNum);
void RFIDRegisterReadCallBack(uint16_t deviceNum, RFIDcallback cb);
//循环查询EPC
int RFIDStartLoopCheckEpc(uint16_t deviceNum);
int RFIDStopLoopCheckEpc(uint16_t deviceNum);
void RFIDProcess(uint16_t deviceNum);




#endif /* RFID_DRV_H_ */
