/*
 * EPCdef.h
 *
 *  Created on: 2019-4-17
 *      Author: zhtro
 */

#ifndef EPCDEF_H_
#define EPCDEF_H_

#include "stdint.h"

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
#define EPC_FEAT_HORIZONTAL_STRAIGHT (0x01)
#define EPC_FEAT_HORIZONTAL_CURVE	 (0x02)
#define EPC_FEAT_UPHILL_GRADUAL		 (0x03)
#define EPC_FEAT_UPHILL_STEEP  		 (0x04)
#define EPC_FEAT_DOWNHILL_GRADUAL	 (0x05)
#define EPC_FEAT_DOWNHILL_STEEP		 (0x06)

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
	uint16_t firstNo;
	//2级支道编号
	uint8_t secondNo;
	//3级支道编号
	uint8_t thirdNo;
	//4级支道编号
	uint8_t fourthNo;
	//区域类型
	uint8_t areaType;
	//区域编号
	uint16_t areaNo;
	//子区域类型
	uint8_t subareaType;
	//道路特性
	uint8_t roadFeature;
	//距起始位置距离
	uint32_t distance;
	uint32_t timeStamp;
}epc_t;


extern void EPCfromByteArray(epc_t * epc, uint8_t array[]);
extern uint8_t EPCequal(epc_t * a, epc_t * b);
extern uint64_t EPCgetShortID(epc_t * epc);

#endif /* EPCDEF_H_ */
