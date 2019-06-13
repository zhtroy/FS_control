/*
 * PhotoElectric.h
 *
 *  Created on: 2018-12-18
 *      Author: zhtro
 */

#ifndef PHOTOELECTRIC_H_
#define PHOTOELECTRIC_H_

#include <ti/sysbios/knl/Semaphore.h>

#define PHOTO_CAN_DEV 2          //暂时使用CAN 2


#define PHOTO_LIGHT_0_ON  (1)
#define PHOTO_LIGHT_1_ON  (2)
#define PHOTO_LIGHT_2_ON  (4)

#define PHOTO_MBOX_DEPTH (16)


#pragma pack(1)
typedef struct{
	uint8_t cmd;
	uint8_t chn;
	uint32_t data;
}photo_data_t;

typedef struct{
	uint32_t deviceNum;
	photo_data_t content;
}photo_t;


/*
 * DSP发送到光电对管
 */
#define CAN_CMD_VER  0x00   //查询版本号
#define CAN_CMD_ID   0x0A   //写CAN ID


/*
 * 光电对管返回数据
 */
#define CAN_CMD_RISE  0x10  //上升沿    这时的data代表距离上一次跳变的时间(ms)
#define CAN_CMD_FALL  0x11  //下降沿    这时的data代表距离上一次跳变的时间(ms)
#define CAN_CMD_CTRL  0x12  //控制对管发射端通断命令


/*
 * API
 */


/*
 * 修改对管设备ID（即CAN ID)
 */
extern void PhotoEleChangeID(uint32_t oldID, uint32_t newID);
/*
 * 设置对管发射端通断
 */
extern void PhotoEleSetLight(uint32_t id, uint8_t mask);

extern void taskPhotoElectric();

extern Semaphore_Handle g_sem_photoelectric;

#endif /* PHOTOELECTRIC_H_ */
