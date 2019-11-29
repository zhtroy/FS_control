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
#include "math.h"


static float _pointsToSpeed(int16_t point)
{
	return ((float) g_sysParam.encoderWheelPerimeter) / 10000.0 * point * 20.0f / (float) ENCODER_POINTS_CYCLE ;
}

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

int16_t Encoder1GetDeltaPoint()
{
    uint16_t dco = 0;
    int16_t * dptr;

    dco = EMIFAReadWord(FPGA_DCO_1_LOW,0);
    dco += (EMIFAReadWord(FPGA_DCO_1_HIGH,0) << 8);
    dptr = &dco;

    return *dptr;
}

int16_t Encoder1GetPointsIn50ms()
{
    uint16_t sco = 0;
    int16_t * sptr;

    sco = EMIFAReadWord(FPGA_SCO_1_LOW,0);
	sco += (EMIFAReadWord(FPGA_SCO_1_HIGH,0) << 8);
	sptr = &sco;

	return *sptr;
}

float Encoder1GetSpeed()
{
	return _pointsToSpeed(Encoder1GetPointsIn50ms());
}

/*
 * 返回编码器测量速度 m/s
 * 有正负
 */
float EncoderGetSpeed()
{
	return _pointsToSpeed(EncoderGetPointsIn50ms());
}


static void TaskCheckEncoderStatus()
{
	int abnormalcount = 0;
	int encoderabnormalCount = 0;
	const float SPEED_DIFF_ENCODER_MOTOR =  0.5;
	const float SPEED_DIFF_ENCODER_ENCODER =  0.2;
	const int ABNORMAL_TIMES = 20;
	const int REVERSE_THR = 10; // 1m
	int reversingStartPos;
	int reverseDiff;

	while(1)
	{
		Task_sleep(100);

		//检测编码器返回速度和电机返回速度是否有0.5m/s以上的差别，如果有，则报故障
		if(fabs( fabs(EncoderGetSpeed()) - MotoGetSpeed() ) > SPEED_DIFF_ENCODER_MOTOR)
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

		g_fbData.encoderSpeed = EncoderGetSpeed();

		//检测是否有溜坡
		if( (MotoGetGear()==GEAR_DRIVE && EncoderGetSpeed()>=0)
		  ||(MotoGetGear() == GEAR_REVERSE && EncoderGetSpeed()<=0 ))
		{
			reversingStartPos = MotoGetCarDistance();
		}

		reverseDiff = 0;
		if(MotoGetGear() == GEAR_DRIVE)
		{
			reverseDiff = reversingStartPos - MotoGetCarDistance();
		}
		else if(MotoGetGear() == GEAR_REVERSE )
		{
			reverseDiff =  MotoGetCarDistance() - reversingStartPos;
		}

		if(reverseDiff > REVERSE_THR)
		{
			reversingStartPos = MotoGetCarDistance();
			Message_postError(ERROR_REVERSING);
		}

		//检测两个编码器之间的数据是否有大差异
		if( fabs(EncoderGetSpeed() - Encoder1GetSpeed()) >  SPEED_DIFF_ENCODER_ENCODER)
		{
			encoderabnormalCount ++;
		}
		else
		{
			encoderabnormalCount =0;
		}
		if(encoderabnormalCount >= 3)
		{
			encoderabnormalCount = 0;
			Message_postError(ERROR_ENCODER);
		}

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

