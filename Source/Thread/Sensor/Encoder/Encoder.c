/*
 * Encoder.c
 *
 *  Created on: 2019-9-18
 *      Author: zhtro
 */


#include "Sensor/Encoder/Encoder.h"
#include "fpga_periph_def.h"
#include "emifa/emifa_app.h"
#include "Periph/system_param.h"
#include "Moto/task_moto.h"
#include "math.h"
#include <ti/sysbios/knl/Task.h>
#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include "Message/Message.h"

int16_t EncoderGetDeltaPoint()
{
    uint16_t dco = 0;
    int16_t * dptr;

    dco = EMIFAReadWord(FPGA_DCO_LOW,0);
    dco += (EMIFAReadWord(FPGA_DCO_HIGH,0) << 8);
    dptr = &dco;

    return *dptr;
}

int16_t EncoderGetPointsIn50ms()
{
    uint16_t sco = 0;
    int16_t * sptr;

    sco = EMIFAReadWord(FPGA_SCO_LOW,0);
	sco += (EMIFAReadWord(FPGA_SCO_HIGH,0) << 8);
	sptr = &sco;

	return *sptr;
}

/*
 * 返回编码器测量速度 m/s
 */
float EncoderGetSpeed()
{
	float pointsIn50ms =  EncoderGetPointsIn50ms();
	return ((float) sysParam.encoderWheelPerimeter) / 10000.0 * pointsIn50ms * 20.0f / (float) ENCODER_POINTS_CYCLE ;
}


static void TaskCheckEncoderStatus()
{
	int abnormalcount = 0;
	const float SPEED_DIFF =  0.5;
	const int ABNORMAL_TIMES = 20;
	while(1)
	{
		Task_sleep(100);
		if(fabs( fabs(EncoderGetSpeed()) - MotoGetSpeed() ) > SPEED_DIFF)
		{
			abnormalcount ++;
		}
		else
		{
			abnormalcount =0;
		}
		if(abnormalcount >= ABNORMAL_TIMES)
		{
			abnormalcount = 0;
			Message_postError(ERROR_ENCODER);
		}

		//g_fbData.encoderSpeed = EncoderGetSpeed();
	}


}

void EncoderInit()
{
	Task_Handle task;
	Task_Params taskParams;


	//task
	Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create(TaskCheckEncoderStatus, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}
}

