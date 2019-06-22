/*
 * v2v_communication.h
 *
 *  Created on: 2019-5-8
 *      Author: DELL
 */

#ifndef V2V_COMMUNICATION_H_
#define V2V_COMMUNICATION_H_


#define V2V_CAR_FATAL (1)
#define V2V_CAR_OK (0)

typedef struct{
    uint32_t distance;
    uint16_t rpm;
    uint8_t state;
}carStatus_t;

uint16_t V2VGetFrontCarSpeed();
uint32_t V2VGetDistanceToFrontCar();
#endif /* V2V_COMMUNICATION_H_ */
