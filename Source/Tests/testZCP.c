/*
 * testZCP.c
 *
 *  Created on: 2019-5-5
 *      Author: DELL
 */
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>
#include "ZCP/zcp_driver.h"

#define CAR_ZCP_UART_DEV_NUM    (5)
#define CAR_ZCP_DEV_NUM (0)

ZCPInstance_t carInst;


uint32_t ssCnt = 0;
uint32_t sseCnt = 0;
uint32_t sCnt = 0;
uint32_t seCnt = 0;
uint32_t rCnt = 0;
void testZCPStationSendTask(UArg arg0, UArg arg1)
{
    ZCPUserPacket_t sendPacket;
    Semaphore_Handle stationSem;
    Semaphore_Params semParams;

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    stationSem = Semaphore_create(0, &semParams, NULL);

    sendPacket.addr = 0x8002;
    sendPacket.type = 0x80;

    while(1)
    {
        Task_sleep(200);

        memcpy(sendPacket.data,&sCnt,sizeof(ssCnt));
        sendPacket.len = sizeof(ssCnt);

        if(FALSE == ZCPSendPacket(&carInst, &sendPacket, stationSem,BIOS_NO_WAIT))
        {
            LogMsg("Send Failed!\r\n");
        }

        ssCnt++;
        if(FALSE == Semaphore_pend(stationSem,50))
        {
            sseCnt++;
        }
    }
}

void testZCPSendTask(UArg arg0, UArg arg1)
{
    ZCPUserPacket_t sendPacket;
    Semaphore_Handle carSem;
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    carSem = Semaphore_create(0, &semParams, NULL);

    sendPacket.addr = 0x8002;

    sendPacket.type = 0x40;

    while(1)
    {
        Task_sleep(100);

        memcpy(sendPacket.data,&sCnt,sizeof(sCnt));
        sendPacket.len = sizeof(sCnt);

        if(FALSE == ZCPSendPacket(&carInst, &sendPacket, carSem,BIOS_NO_WAIT))
        {
            LogMsg("Send Failed!\r\n");
        }

        sCnt++;
        if(FALSE == Semaphore_pend(carSem,50))
        {
            seCnt++;
        }
    }
}


void testZCPRecvTask(UArg arg0, UArg arg1)
{
    ZCPUserPacket_t recvPacket;
    int32_t timestamp;
    while(1)
    {
        ZCPRecvPacket(&carInst, &recvPacket, &timestamp, BIOS_WAIT_FOREVER);
        rCnt++;
#if 0
        LogMsg("Addr:%x%x,len:%d,type:%d\r\n",
                recvPacket.addrH,
                recvPacket.addrL,
                recvPacket.len,
                recvPacket.type);
#endif
        LogMsg("%d\r\n",timestamp);
    }
}

void testZCPInit()
{

    Task_Handle task;
    Task_Params taskParams;

    ZCPInit(&carInst,CAR_ZCP_DEV_NUM,CAR_ZCP_UART_DEV_NUM);



    Task_Params_init(&taskParams);
    taskParams.priority = 5;
    taskParams.stackSize = 2048;

    task = Task_create((Task_FuncPtr)testZCPStationSendTask, &taskParams, NULL);
   if (task == NULL) {
       System_printf("Task_create() failed!\n");
       BIOS_exit(0);
   }

    task = Task_create((Task_FuncPtr)testZCPSendTask, &taskParams, NULL);
    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

    task = Task_create((Task_FuncPtr)testZCPRecvTask, &taskParams, NULL);
    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }
}
