/*
 * EPC.c
 *
 *  Created on: 2019-5-7
 *      Author: zhtro
 */

#include "Sensor/RFID/EPCdef.h"
#include "string.h"
#include "Lib/bitop.h"

void EPCfromByteArray(epc_t * epc, uint8_t array[])
{
	/*
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
	//路段类型
	uint8_t areaType;
	//功能区
	uint8_t funcType;
	//变轨点
	uint8_t changePoint;
	//道路特性
	uint8_t roadFeature;
	//距起始位置距离
	uint32_t distance;
	*/

	epc->reserved = array[0];
	epc->mainNo = array[1];
	epc->firstNo = array[2];
	epc->secondNo = array[3];
	epc->thirdNo = array[4];
	epc->fourthNo = array[5];
	epc->ab = BF_GET(array[6],7,1);
	epc->adjustAreaNo = BF_GET(array[6],2,5);
	epc->areaType = BF_GET(array[6],0,2);
	epc->funcType = BF_GET(array[7],6,2);
	epc->changePoint = BF_GET(array[7],5,1);
	epc->roadFeature = BF_GET(array[7],0,5);


	epc->distance = array[8]*65536 + array[9]*256 + array[10];

	epc->roadBreak = BF_GET(array[11], 5,3 );


}

uint8_t EPCequal(epc_t * a, epc_t * b)
{
	return !(memcmp(a,b, sizeof(epc_t)));
}

uint8_t EPCinSameRoad(epc_t * a, epc_t * b)
{
	return ((a->mainNo == b->mainNo) &&
		   (a->firstNo == b->firstNo) &&
		   (a->secondNo == b->secondNo) &&
		   (a->thirdNo == b->thirdNo) &&
		   (a->fourthNo == b->fourthNo)) ? 1:0;
}

/*
 * 得到epc的64位短ID
 */
uint64_t EPCgetShortID(epc_t * epc)
{
	return 	(((uint64_t)epc->mainNo)<<56)| \
			(((uint64_t)epc->firstNo)<<48)| \
			(((uint64_t)epc->secondNo)<<40)| \
			(((uint64_t)epc->thirdNo)<<32)| \
			(((uint64_t)epc->fourthNo)<<24)| \
			(((uint64_t)epc->distance)) ;
}
