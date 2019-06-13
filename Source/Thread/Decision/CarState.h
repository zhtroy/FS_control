/*
 * CarState.h
 *
 *  Created on: 2019-4-17
 *      Author: zhtro
 */

#ifndef CARSTATE_H_
#define CARSTATE_H_

/*
 * 状态机状态
 */
typedef enum{
	idle,
	running,
	running_normal,
	running_adjust,
	running_seperate,
	stop,
	car_state_None
}car_state_t;


typedef enum
{
	Manual,
	Setting,
	Auto,
	ForceBrake
}car_mode_t;

#endif /* CARSTATE_H_ */
