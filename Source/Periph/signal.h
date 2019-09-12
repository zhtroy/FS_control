/*
 * signal.h
 *
 *  Created on: 2019-7-18
 *      Author: DELL
 */

#ifndef SIGNAL_H_
#define SIGNAL_H_
#include "common.h"

uint8_t SignalGetHardIntStatus();
void SignalCallBack(uint8_t intStatus);
void SignalInit();
void SignalSetIntEnable(uint8_t value);
#endif /* SIGNAL_H_ */
