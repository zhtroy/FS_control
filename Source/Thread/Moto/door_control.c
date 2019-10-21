/*
 * door_control.c
 *
 *  Created on: 2019-10-19
 *      Author: zhtro
 */

#ifndef DOOR_CONTROL_C_
#define DOOR_CONTROL_C_

#include "canModule.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include "Moto/door_control.h"
#include "Encoder/Encoder.h"
#include "Lib/bitop.h"
#include "Message/Message.h"
#include "Command/CommandDriver.h"
#include "Moto/task_moto.h"
#include "CarState.h"
/*
 * 0 关闭
 * 1 打开
 */
static uint8_t _doorState = DOOR_STATE_CLOSE;
static Semaphore_Handle _SemDoorOpen;
static Semaphore_Handle _SemDoorClose;
static Semaphore_Handle _txReadySem;
static Mailbox_Handle _rxDataMbox = NULL;
static uint8_t _openTimeout = 0;
static uint8_t _closeTimeout = 0;


static void DoorCanIntrHandler(int32_t devsNum,int32_t event)
{
    canDataObj_t rxData;

	if (event == 1)         /* 收到一帧数据 */
    {
        CanRead(devsNum, &rxData);
        Mailbox_post(_rxDataMbox, (Ptr *)&rxData, BIOS_NO_WAIT);
	}
    else if (event == 2)    /* 一帧数据发送完成 */
    {
        /* 发送中断 */
        Semaphore_post(_txReadySem);
    }

}

#define DOOR_OP_INTERVAL (100)

static void RoutineDoorOpen()
{
	canDataObj_t canSendData;
	int i;

    /*初始化帧类型*/
    canSendData.ID = 0x100;
    canSendData.SendType = 0;
    canSendData.RemoteFlag = 0;
    canSendData.ExternFlag = 0;
    canSendData.DataLen = 8;
    memset(canSendData.Data, 0, 8);
    canSendData.Data[0] = 0x05;


	while(1)
	{
		_openTimeout = 0;
		Semaphore_pend(_SemDoorOpen, BIOS_WAIT_FOREVER);

		//发送开门指令
        Semaphore_pend(_txReadySem, BIOS_WAIT_FOREVER);
        canSendData.Data[1] = 1;
        CanWrite(DOOR_CTRL_CAN_DEV, &canSendData);

        /*
         * 检查超时
         * 包括两种超时：车门控制器返回的超时，超过DOOR_OP_TIMEOUT都没有到位的超时
         */
        for(i = 0;i<DOOR_OP_TIMEOUT/DOOR_OP_INTERVAL; i++)
        {
        	Task_sleep(DOOR_OP_INTERVAL);
        	if(_doorState == DOOR_STATE_OPEN)
        	{
        		break;
        	}
        	if(_openTimeout)
        	{
        		break;
        	}
        }

        if(_doorState == DOOR_STATE_OPEN)
        {
        	CommandSend(&_doorState,sizeof(uint8_t), COMMAND_TYPE_DOOR_STATE);
        }
        else   //超时
        {
        	Message_postError(ERROR_DOOR_OPEN_TIMEOUT);
        }
	}
}

static void RoutineDoorClose()
{
	canDataObj_t canSendData;
	int i;

	/*初始化帧类型*/
	canSendData.ID = 0x100;
	canSendData.SendType = 0;
	canSendData.RemoteFlag = 0;
	canSendData.ExternFlag = 0;
	canSendData.DataLen = 8;
	memset(canSendData.Data, 0, 8);
	canSendData.Data[0] = 0x05;


	while(1)
	{
		_closeTimeout = 0;
		Semaphore_pend(_SemDoorClose, BIOS_WAIT_FOREVER);

		//发送关门指令
		Semaphore_pend(_txReadySem, BIOS_WAIT_FOREVER);
		canSendData.Data[1] = 2;
		CanWrite(DOOR_CTRL_CAN_DEV, &canSendData);

		/*
		 * 检查超时
		 * 包括两种超时：车门控制器返回的超时，超过DOOR_OP_TIMEOUT都没有到位的超时
		 */
		for(i = 0;i<DOOR_OP_TIMEOUT/DOOR_OP_INTERVAL; i++)
		{
			Task_sleep(DOOR_OP_INTERVAL);
			if(_doorState == DOOR_STATE_CLOSE)
			{
				break;
			}
			if(_closeTimeout)
			{
				break;
			}
		}

		if(_doorState == DOOR_STATE_CLOSE)
		{
			CommandSend(&_doorState,sizeof(uint8_t), COMMAND_TYPE_DOOR_STATE);
		}
		else   //超时
		{
			Message_postError(ERROR_DOOR_CLOSE_TIMEOUT);
		}
	}
}

static void DoorRecvTask()
{
    canDataObj_t canRecvData;
	Bool result;

	while(1)
	{
		result = Mailbox_pend(_rxDataMbox, (Ptr *)&canRecvData, 3000);

		if(!result)
		{
			Message_postError(ERROR_DOOR_CONTROLLER_TIMEOUT);
			Task_sleep(10000);
			continue;
		}

		if(BF_GET(canRecvData.Data[1],0,1) == 1)
		{
			_doorState = DOOR_STATE_OPENING;
		}

		if(BF_GET(canRecvData.Data[1],1,1) == 1)
		{
			_doorState = DOOR_STATE_CLOSING;
		}

		if(BF_GET(canRecvData.Data[1],2,1) == 1)
		{
			_doorState = DOOR_STATE_OPEN;
		}
		if(BF_GET(canRecvData.Data[1],3,1) == 1)
		{
			_doorState = DOOR_STATE_CLOSE;
		}

		if(BF_GET(canRecvData.Data[1],4,1) == 1)
		{
			_openTimeout = 1;
		}

		if(BF_GET(canRecvData.Data[1],5,1) == 1)
		{
			_closeTimeout = 1;
		}

	}
}

static void DoorStateCheckTask()
{
	while(1)
	{
		Task_sleep(100);

		//如果自动模式下速度大于0.1 且开门，则报错
		if(EncoderGetSpeed()>0.1 &&
		   MotoGetCarMode() == Auto &&
		   (DOOR_STATE_OPENING == _doorState || DOOR_STATE_OPEN == _doorState) )
		{
			Message_postError(ERROR_DOOR_OPEN_WHILE_RUNNING);
		}
	}
}

uint8_t DoorGetState()
{
	return _doorState;
}

uint8_t DoorOpen()
{
	if(_doorState == DOOR_STATE_OPEN)
	{
		return 0;
	}
	if(EncoderGetSpeed()>0)
	{
		return 0;
	}

	Semaphore_post(_SemDoorOpen);

	return 1;
}

uint8_t DoorClose()
{
	if(_doorState == DOOR_STATE_CLOSE)
	{
		return 0;
	}

	Semaphore_post(_SemDoorClose);

	return 1;
}


void DoorCtrlInit()
{
	Task_Handle task;
	Task_Params taskParams;
	Semaphore_Params semParams;
    Mailbox_Params mboxParams;

    /*初始化CAN设备*/
    CanOpen(DOOR_CTRL_CAN_DEV, DoorCanIntrHandler, DOOR_CTRL_CAN_DEV);

    /*
     * 初始化任务
     */
    Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;

    task = Task_create(RoutineDoorOpen, &taskParams, NULL);
  	if (task == NULL) {
  		System_printf("Task_create() failed!\n");
  		BIOS_exit(0);
  	}

    task = Task_create(RoutineDoorClose, &taskParams, NULL);
  	if (task == NULL) {
  		System_printf("Task_create() failed!\n");
  		BIOS_exit(0);
  	}

    task = Task_create(DoorRecvTask, &taskParams, NULL);
  	if (task == NULL) {
  		System_printf("Task_create() failed!\n");
  		BIOS_exit(0);
  	}

    task = Task_create(DoorStateCheckTask, &taskParams, NULL);
  	if (task == NULL) {
  		System_printf("Task_create() failed!\n");
  		BIOS_exit(0);
  	}

  	/*
  	 * 初始化信号量
  	 */
	Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    _SemDoorOpen = Semaphore_create(0, &semParams, NULL);
    _SemDoorClose = Semaphore_create(0, &semParams, NULL);
    _txReadySem = Semaphore_create(1, &semParams, NULL);

    /*
     * 初始化邮箱
     */
    Mailbox_Params_init(&mboxParams);
	_rxDataMbox = Mailbox_create (sizeof (canDataObj_t),4, &mboxParams, NULL);
}

#endif /* DOOR_CONTROL_C_ */
