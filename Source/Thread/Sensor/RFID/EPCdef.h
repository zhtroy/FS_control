/*
 * EPCdef.h
 *
 *  Created on: 2019-4-17
 *      Author: zhtro
 */

#ifndef EPCDEF_H_
#define EPCDEF_H_

#include "stdint.h"

#define EPC_SIZE  				(12)

#define EPC_STRAIGHT   			(0x01)
#define EPC_PRE_CURVE  			(0x02)
#define EPC_CURVING    			(0x03)
#define EPC_UPHILL	   			(0x04)
#define EPC_PRE_DOWNHILL  		(0x05)
#define EPC_DOWNHILL  			(0x06)
#define EPC_PRE_SEPERATE  		(0x07)
#define EPC_SEPERATE  			(0x08)
#define EPC_ENTER_STATION  		(0x09)
#define EPC_STOP_STATION  		(0x0A)
#define EPC_LEAVE_STATION  		(0x0B)
#define EPC_PRE_MERGE 			(0x0C)
#define EPC_MERGE  				(0x0D)

#define EPC_AUXILIARY_TRACK_START (0x55)
#define EPC_AUXILIARY_TRACK_END   (0xAA)

/*道路特性*/
typedef enum{
	EPC_FEAT_CRUISING,
	EPC_FEAT_HORIZONTAL_STRAIGHT,
	EPC_FEAT_HORIZONTAL_CURVE_L1,
	EPC_FEAT_HORIZONTAL_CURVE_L2,
	EPC_FEAT_HORIZONTAL_CURVE_L3,
	EPC_FEAT_UPHILL_L1,
	EPC_FEAT_UPHILL_L2,
	EPC_FEAT_UPHILL_L3,
	EPC_FEAT_DOWNHILL_L1,
	EPC_FEAT_DOWNHILL_L2,
	EPC_FEAT_DOWNHILL_L3,
	EPC_FEAT_HORIZONTAL_STATION,
	EPC_FEAT_UPHILL_STATION,
	EPC_FEAT_DOWNHILL_STATION,
	EPC_FEAT_END
}epc_feat_t;

/*
 * AB段
 */
#define EPC_AB_A  	(0)
#define EPC_AB_B	(1)
/*
 * 路段类型
 */
#define EPC_AREATYPE_NORMAL 	(0)
#define EPC_AREATYPE_STATION 	(1)

/*
 * 断头路类型
 */
#define EPC_ROADBREAK_LEFT_AHEAD 	(1)
#define EPC_ROADBREAK_LEFT_BEHIND 	(2)
#define EPC_ROADBREAK_RIGHT_AHEAD 	(3)
#define EPC_ROADBREAK_RIGHT_BEHIND 	(4)

/*
 * 坡道类型
 */
#define EPC_RAMPTYPE_FLAT  (0)
#define EPC_RAMPTYPE_UPHILL  (1)
#define EPC_RAMPTYPE_DOWNHILL  (2)

/*
 * 功能区
 */
#define EPC_FUNC_NORMAL 	(0)
#define EPC_FUNC_SEPERATE 	(1)
#define EPC_FUNC_LADJUST 	(2)
#define EPC_FUNC_RADJUST	(3)

#define EPC_FEAT_AUXILIARY_TRACK_START (0x0F)
#define EPC_FEAT_AUXILIARY_TRACK_END (0x10)
/*未完。。。*/

#pragma pack(1)
typedef struct epc_tag
{
	uint8_t reserved;
	//干道编号
	uint8_t mainNo;
	//1级支道编号
	uint8_t firstNo;
	//2级支道编号
	uint8_t secondNo;
	//3级支道编号
	uint8_t thirdNo;
	//4级支道编号
	uint8_t fourthNo;
	//AB
	uint8_t ab;
	//调整区编号
	uint8_t adjustAreaNo;
	//路段类型
	uint8_t areaType;
	//功能区
	uint8_t funcType;
	//路段速度(km/h)
	uint8_t roadSpeed;
	//距起始位置距离
	uint32_t distance;
	//断头路信息
	uint8_t roadBreak;
	//坡道信息
	uint8_t rampType;
}epc_t;

typedef struct
{
    uint8_t byte[5];
}roadID_t;

#define RFID_POINT_LEN (18)
typedef struct{
    uint8_t byte[RFID_POINT_LEN];
}rfidPoint_t;

extern void EPCfromByteArray(epc_t * epc, uint8_t array[]);
extern uint8_t EPCequal(epc_t * a, epc_t * b);
extern uint64_t EPCgetShortID(epc_t * epc);
extern uint8_t EPCinSameRoad(epc_t * a, epc_t * b);

#endif /* EPCDEF_H_ */
