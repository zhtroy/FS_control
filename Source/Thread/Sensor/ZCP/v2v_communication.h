/*
 * v2v_communication.h
 *
 *  Created on: 2019-5-8
 *      Author: DELL
 */

#ifndef V2V_COMMUNICATION_H_
#define V2V_COMMUNICATION_H_
/*
 * 轮子周长,轨道总长
 * 单位(10cm)
 */
#define WHEEL_PERIMETER (11.78)
#define WHEEL_SPEED_RATIO (3.286)
#define TOTAL_DISTANCE (2775)

#define V2V_CAR_FATAL (1)
#define V2V_CAR_OK (0)

typedef struct{
    uint32_t distance;
    uint16_t rpm;
    uint8_t state;
}carStatus_t;

uint16_t V2VGetFrontCarSpeed();
uint32_t V2VGetCarDistance();
#endif /* V2V_COMMUNICATION_H_ */
