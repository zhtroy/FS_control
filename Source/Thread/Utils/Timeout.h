/*
 * Timeout.h
 *
 *  Created on: 2019-4-19
 *      Author: zhtro
 */

#ifndef TIMEOUT_H_
#define TIMEOUT_H_


typedef enum
{
	changerail_wait_photon = 0,
	changerail_wait_changerail,
	station_stop,
	merge_wait_photon,
	merge_wait_changerail,
	seperate_wait_enter_station,
	seperate_wait_stop_station,
	seperate_wait_leave_station,
	seperate_wait_pre_merge,
	seperate_wait_merge
}timeout_type_t;

extern void TimeoutSet(timeout_type_t msg);
extern void TimeoutInit();

#endif /* TIMEOUT_H_ */
