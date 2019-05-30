/*
 * test4GControl.h
 *
 *  Created on: 2019-1-10
 *      Author: zhtro
 */

#ifndef TEST4GCONTROL_H_
#define TEST4GCONTROL_H_

#include "stdint.h"



#define FORCE_BRAKE				(150)


typedef enum{
	s_wait_photon,
	seperating,
	seperate_ok,
	seperate_cancel
}car_seperate_state_t;

typedef enum{
	m_wait_photon,
	merging,
	merge_ok
}car_merge_state_t;





/*
 * 延时
 */
uint32_t g_timeout[]=
{
		2000,
		3000,
		5000,
		2000,
		4000,
		20000,
		10000,
		10000,
		10000,
		10000
};
//#define TIMEOUT_seperate_wait_photon (1000)
//#define TIMEOUT_seperate_wait_changerail_complete (2000)

#endif /* TEST4GCONTROL_H_ */
