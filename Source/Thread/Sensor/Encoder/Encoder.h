/*
 * Encoder.h
 *
 *  Created on: 2019-9-18
 *      Author: zhtro
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stdint.h"

#define ENCODER_POINTS_CYCLE (200)

/*
 * 返回上一次读取后，编码器转动距离（单位10cm）
 */
extern float Encoder0GetDeltaDistance();
/*
 * 返回上一次读取后，编码器转动的点数
 */
extern int16_t Encoder0GetDeltaPoint();
/*
 * 返回上一个50ms内编码器转动的点数，可以用于计算速度
 */
extern int16_t Encoder0GetPointsIn50ms();

void EncoderInit();

float Encoder0GetSpeed();

#endif /* ENCODER_H_ */
