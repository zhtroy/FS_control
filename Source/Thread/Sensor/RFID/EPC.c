/*
 * EPC.c
 *
 *  Created on: 2019-5-7
 *      Author: zhtro
 */

#include "Sensor/RFID/EPCdef.h"

void EPCfromByteArray(epc_t * epc, uint8_t array[])
{
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

	mainNo = array[1];
	firstNo = array[2];
	firstNo = firstNo <<2;
	firstNo += array[3] >> 6;
	secondNo = (array[3] & 0x3F) <<2;
	secondNo += array[4]>>6;
	thirdNo = array[4] & 0x3F;
	fourthNo = array[5] >>2;
	areaType = ((array[5] & 0x03) <<1) + (array[6] >> 7);
	areaNo = array[6] & 0x7F;
	areaNo = areaNo << 3;
	areaNo += array[7] >> 5;
	subareaType = array[7] & 0x1F;
	roadFeature = array[8] >> 3;
	distance = array[8] & 0x07;
	distance = distance << 8;
	distance += array[9];
	distance = distance <<8;
	distance += array[10];
	distance = distance << 5;
	distance += array[11] >> 3;

	epc->areaNo = areaNo;
	epc->areaType = areaType;
	epc->distance = distance;
	epc->firstNo = firstNo;
	epc->secondNo = secondNo;
	epc->thirdNo = thirdNo;
	epc->fourthNo = fourthNo;
	epc->mainNo = mainNo;
	epc->roadFeature = roadFeature;
	epc->distance = distance;


}

uint8_t EPCequal(epc_t * a, epc_t * b)
{
	return !(memcmp(a,b, sizeof(epc_t)));
}
