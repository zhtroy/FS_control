/*
 * Encoder.c
 *
 *  Created on: 2019-9-18
 *      Author: zhtro
 */


#include "Sensor/Encoder/Encoder.h"
#include "fpga_periph_def.h"
#include "emifa/emifa_app.h"

extern int16_t EncoderGetDeltaPoint()
{
    uint16_t dco = 0;
    int16_t * dptr;

    dco = EMIFAReadWord(FPGA_DCO_LOW,0);
    dco += (EMIFAReadWord(FPGA_DCO_HIGH,0) << 8);
    dptr = &dco;

    return *dptr;
}

extern int16_t EncoderGetPointsIn50ms()
{
    uint16_t sco = 0;
    int16_t * sptr;

    sco = EMIFAReadWord(FPGA_SCO_LOW,0);
	sco += (EMIFAReadWord(FPGA_SCO_HIGH,0) << 8);
	sptr = &sco;

	return *sptr;
}
