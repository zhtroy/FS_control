/*
 * v2v_communication.c
 *
 *  Created on: 2019-5-8
 *      Author: DELL
 */

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/std.h>
#include "ZCP/zcp_driver.h"
#include "ZCP/v2v_communication.h"
#include "task_moto.h"
#include "CarState.h"
#include "RFID/EPCdef.h"
#include "Parameter.h"
#include "Message/Message.h"

#define V2V_ZCP_UART_DEV_NUM    (1)
#define V2V_ZCP_DEV_NUM (0)

extern fbdata_t g_fbData;

static ZCPInstance_t v2vInst;
static uint16_t sMotoCircle = 0;
static uint16_t rMotoCircle = 0;
static carStatus_t carSts;
static carStatus_t fbCarSts = {
        .distance = MAX_AWAY_DISTANCE,
        .rpm = 0,
        .state =0
};

extern Mailbox_Handle RFIDGetV2vMailbox();

void V2VSendTask(UArg arg0, UArg arg1)
{
    ZCPUserPacket_t sendPacket;
    Mailbox_Handle RFIDV2vMbox;
    epc_t epc;
    uint16_t circleNums = 0;
    uint16_t circleDiff = 0;


    while(1)
    {
        RFIDV2vMbox = RFIDGetV2vMailbox();
        if(RFIDV2vMbox == NULL)
        {
        	Task_sleep(100);
            continue;
        }

        if(TRUE == Mailbox_pend(RFIDV2vMbox,(Ptr *)&epc,100))
        {
            circleNums = MotoGetCircles();
            carSts.distance = epc.distance;
        }
        else
        {
            sMotoCircle = MotoGetCircles();

            if(circleNums > sMotoCircle)
            {
                circleDiff = 65535 -circleNums + sMotoCircle;
            }
            else
            {
                circleDiff = sMotoCircle - circleNums;
            }
            circleNums = sMotoCircle;

            carSts.distance += circleDiff * WHEEL_PERIMETER / WHEEL_SPEED_RATIO;
            if(carSts.distance > TOTAL_DISTANCE)
            {
                carSts.distance = carSts.distance - TOTAL_DISTANCE;
            }
        }

        carSts.rpm = MotoGetRpm();

        if(MotoGetCarMode() == ForceBrake)
            carSts.state = V2V_CAR_FATAL;
        else
            carSts.state = V2V_CAR_OK;

        memcpy(sendPacket.data,&carSts,sizeof(carStatus_t));
        sendPacket.addr = g_param.REAR_CAR_ADDR;
        sendPacket.type = V2V_CAR_STATUS_TYPE;
        sendPacket.len = sizeof(carStatus_t);

        if(FALSE == ZCPSendPacket(&v2vInst, &sendPacket, NULL,BIOS_NO_WAIT))
        {
            LogMsg("Send Failed!\r\n");
        }

    }
}
static uint8_t m_zigbeeRecved = 0;
static xdc_Void zigbeeTimeout(xdc_UArg arg)
{
    p_msg_t msg;

    if(0==m_zigbeeRecved)
    {
		msg = Message_getEmpty();
		msg->type = error;
		msg->data[0] = ERROR_ZIGBEE_TIMEOUT;
		msg->dataLen = 1;
		Message_post(msg);
    }
    m_zigbeeRecved = 0;
}

void V2VRecvTask(UArg arg0, UArg arg1)
{
    ZCPUserPacket_t recvPacket;
    uint16_t circleDiff;
    uint32_t dis;
    carStatus_t recvCarSts;
    int32_t timestamp;

    /*Zigbee接收超时，停车*/
	Clock_Params clockParams;
	Clock_Handle recvClock;

	Clock_Params_init(&clockParams);
	clockParams.period = 100000;       // 1s周期检测
	clockParams.startFlag = TRUE;
//	recvClock = Clock_create(zigbeeTimeout, 1000, &clockParams, NULL);

    while(1)
    {
        ZCPRecvPacket(&v2vInst, &recvPacket, &timestamp, BIOS_WAIT_FOREVER);

        m_zigbeeRecved = 1;

        if(recvPacket.type != V2V_CAR_STATUS_TYPE)
        {
            continue;
        }

        if(recvPacket.len != sizeof(carStatus_t))
        {
            continue;
        }

        memcpy(&recvCarSts,recvPacket.data,sizeof(carStatus_t));
        /*
         * 计算接收报文时实际位置
         */
        rMotoCircle = MotoGetCircles();
        if(sMotoCircle > rMotoCircle)
        {
            circleDiff = 65535 -sMotoCircle + rMotoCircle;
        }
        else
        {
            circleDiff = rMotoCircle - sMotoCircle;
        }
        dis = carSts.distance + circleDiff* WHEEL_PERIMETER;
        if(dis > TOTAL_DISTANCE)
        {
            dis = dis - TOTAL_DISTANCE;
        }

        /*
         * 计算距离差
         */
        if(dis > recvCarSts.distance)
        {
            /*
             * 环形线路计算
             */
            fbCarSts.distance = TOTAL_DISTANCE - dis + recvCarSts.distance;
        }
        else
        {
            fbCarSts.distance = recvCarSts.distance - dis;
        }

        fbCarSts.rpm = recvCarSts.rpm;
        fbCarSts.state = recvCarSts.state;

        g_fbData.forwardCarDistance = fbCarSts.distance;
        g_fbData.forwardCarRPM = fbCarSts.rpm;
    }
}

void V2VZCPInit()
{

    Task_Handle task;
    Task_Params taskParams;

    ZCPInit(&v2vInst,V2V_ZCP_DEV_NUM,V2V_ZCP_UART_DEV_NUM);

    Task_Params_init(&taskParams);
    taskParams.priority = 5;
    taskParams.stackSize = 2048;

    task = Task_create((Task_FuncPtr)V2VSendTask, &taskParams, NULL);
   if (task == NULL) {
       System_printf("Task_create() failed!\n");
       BIOS_exit(0);
   }

    task = Task_create((Task_FuncPtr)V2VRecvTask, &taskParams, NULL);
    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

}

uint16_t V2VGetFrontCarSpeed()
{
    return fbCarSts.rpm;
}

uint32_t V2VGetCarDistance()
{
    return fbCarSts.distance;
}
