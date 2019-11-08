/*
 * zcp_driver.c
 *
 *  备注:为支持多设备，驱动下所有函数均为可重入函数
 */
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>
#include <assert.h>
#include "zcp_driver.h"


static ZCPCfgTable_t ZCPCfgTable[MAX_ZCP_DEVIVE_NUMS];

/*****************************************************************************
 * 函数名称: static ZCPInstance_t * ZCPGetInstance(uint8_t devNum)
 * 函数说明: 根据ZCP设备号，获取实例
 * 输入参数:
 *          devNum:ZCP设备号
 * 输出参数: 无
 * 返 回 值: 返回ZCP设备实例
 * 备注:
*****************************************************************************/
static ZCPInstance_t * ZCPGetInstance(uint8_t devNum)
{
    assert(devNum < MAX_ZCP_DEVIVE_NUMS);

    return (ZCPCfgTable[devNum].pInst);
}

/*****************************************************************************
 * 函数名称: static ZCPInstance_t * ZCPGetInstanceByUartDevNum(uint16_t uartDevNum)
 * 函数说明: 根据串口号获取ZCP设备实例
 * 输入参数:
 *          uartDevNum:串口设备号
 * 输出参数: 无
 * 返 回 值: 存在则返回ZCP设备实例，否则返回NULL
 * 备注:
*****************************************************************************/
static ZCPInstance_t * ZCPGetInstanceByUartDevNum(uint16_t uartDevNum)
{
    uint8_t i;
    ZCPInstance_t * pInst = NULL;

    for(i = 0;i < MAX_ZCP_DEVIVE_NUMS;i++)
    {

        if(ZCPCfgTable[i].isUsed == 1 &&
                ZCPCfgTable[i].pInst->uartDevNum == uartDevNum)
            pInst = ZCPCfgTable[i].pInst;
    }

    return pInst;
}

/*****************************************************************************
 * 函数名称: static void ZCPIntrHandler(void *callBackRef, uint32_t event, uint32_t eventData)
 * 函数说明: ZCP设备的串口中断处理函数
 * 输入参数:
 *          callBackRef:输入参数，指针
 *          event:中断事件
 *          eventData:事件相关数据，串口一般为数据长度
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/
static void ZCPUartIntrHandler(void *callBackRef, u32 event, unsigned int eventData)
{
    uint8_t errors;
    uint16_t uartDeviceNum = *((uint16_t *)callBackRef);
    ZCPInstance_t * pInst;

    /*
     * 通过串口设备号获取ZCP实例
     */
    pInst = ZCPGetInstanceByUartDevNum(uartDeviceNum);

    if (event == XUN_EVENT_SENT_DATA) {
        /*
         * 发送结束Post信用量，信用量为Binary模式
         */
        Semaphore_post(pInst->uartSendSem);
    }

    if (event == XUN_EVENT_RECV_DATA || event == XUN_EVENT_RECV_TIMEOUT) {
        /*
         * step1:获取数据长度
         * step2:Post数据到邮箱，若处理不及时，数据会丢失
         * step3:启动新的串口接收
         */
        pInst->uartRxData.length = eventData;

        Mailbox_post(pInst->uartRecvMbox, (Ptr *)&pInst->uartRxData, BIOS_NO_WAIT);

        UartNs550Recv(uartDeviceNum, pInst->uartRxData.buffer, UART_REC_BUFFER_SIZE);
    }

    if (event == XUN_EVENT_RECV_ERROR) {
        /*
         * 暂不处理
         */
        errors = UartNs550GetLastErrors(uartDeviceNum);
    }
}

/*****************************************************************************
 * 函数名称: static uint8_t ZCPCrcCalc(uint8_t *pData, uint8_t len)
 * 函数说明: ZCP的CRC计算
 * 输入参数:
 *          pData:数据指针地址
 *          len:长度
 * 输出参数: 无
 * 返 回 值: 无
 * 备注: 多项式(x8+x5+x4+1)
*****************************************************************************/
uint8_t ZCPCrcCalc(uint8_t *pData, uint8_t len)
{
    uint8_t i,j;
    uint8_t crc;
    crc = 0xff;

    for(j=0; j<len; j++)
    {
        crc = *pData ^ crc;  /* 每次先与需要计算的数据异或,计算完指向下一数据 */

        for (i=0; i<8; i++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc = (crc >> 1);
        }

        pData++;
    }


    return (crc);

}

/*****************************************************************************
 * 函数名称: static void ZCPPack(ZCPUserPacket_t *pPacket, uartDataObj_t *pBuf)
 * 函数说明: ZCP的用户报文打包（帧头，帧尾以及插入CRC）
 * 输入参数:
 *          pPacket:用户报文指针
 *          pBuf:串口Buffer指针
 * 输出参数: 无
 * 返 回 值: 无
*****************************************************************************/
static void ZCPPack(ZCPPacket_t *pPacket, uartDataObj_t *pBuf)
{
    pBuf->buffer[0] = ZIGBEE_HEAD;
    pBuf->buffer[1] = ZIGBEE_TYPE_MASTER;
    memcpy(&pBuf->buffer[2],pPacket,pPacket->len+2);
    pBuf->buffer[pPacket->len+4] = ZCPCrcCalc(&pPacket->len,pPacket->len);
    pBuf->buffer[pPacket->len+5] = ZIGBEE_TAIL;
    pBuf->length = pPacket->len+6;
}

/*****************************************************************************
 * 函数名称: void ZCPReciveTask(void *param)
 * 函数说明: ZCP的接收任务
 * 输入参数:
 *          arg0: 任务参数，传入设备号
 *          arg1: 任务参数，保留
 * 输出参数: 无
 * 返 回 值: 无
 * 备注: 需保证任务不使用任何非实例的全局变量。
*****************************************************************************/
static void ZCPReciveTask(UArg arg0, UArg arg1)
{
    ZCPInstance_t * pInst;
    uartDataObj_t recvBuf;
    ZCPPacket_t recvPacket;
    uint8_t *pc;
    uint8_t zType;
    uint8_t cnt;
    uint8_t crcCheck;
    uint8_t devNum = (uint8_t)arg0;
    ZCPUnpackState_t state = STS_HEAD;

    /*
     * 根据设备号获取设备指针
     */
    pInst = ZCPGetInstance(devNum);

    while(1)
    {

        Mailbox_pend(pInst->uartRecvMbox, (Ptr *)&recvBuf, BIOS_WAIT_FOREVER);

        pc = recvBuf.buffer;

        while(recvBuf.length)
        {
            switch(state)
            {
                case STS_HEAD:
                    if(ZIGBEE_HEAD == *pc)
                    {
                        state = STS_TYPE;
                    }
                    break;
                case STS_TYPE:
                    if(ZIGBEE_TYPE_ACK == *pc)
                    {
                        zType = ZIGBEE_TYPE_ACK;
                        state = STS_DATA;
                    }
                    else if(ZIGBEE_TYPE_MASTER == *pc)
                    {
                        zType = ZIGBEE_TYPE_MASTER;
                        cnt = 1;
                        state = STS_ADDR;
                    }
                    else
                    {
                        /*
                         * 类型不匹配重新检测报文头
                         */
                        zType = 0;
                        state = STS_HEAD;
                    }
                    break;
                case STS_ADDR:
                    if(cnt == 1)
                    {
                        recvPacket.addrH = *pc;
                        cnt--;
                    }
                    else
                    {
                        recvPacket.addrL = *pc;
                        state = STS_USR_LEN;
                    }
                    break;
                case STS_USR_LEN:
                    recvPacket.len = *pc;
                    if(recvPacket.len < MAX_ZCP_PACKET_SIZE &&
                            recvPacket.len >= 6)
                    {
                        state = STS_USR_TYPE;
                    }
                    else
                    {
                        state = STS_HEAD;
                        pInst->stat.userPacketErr++;
                    }

                    break;
                case STS_USR_TYPE:
                    recvPacket.type = *pc;
                    state = STS_SRC_ADDR_L;
                    break;
                case STS_SRC_ADDR_L:
                    recvPacket.src_addr = *pc;
                    state = STS_SRC_ADDR_H;
                    break;
                case STS_SRC_ADDR_H:
                    recvPacket.src_addr += (*pc << 8);
                    state = STS_DATA;
                    cnt = 0;
                    break;
                case STS_DATA:
                    if(ZIGBEE_TYPE_ACK == zType)
                    {
                        pInst->ackSts = *pc;
                        state = STS_TAIL;
                    }
                    else
                    {
                        recvPacket.data[cnt] = *pc;
                        cnt++;

                        if(cnt >= (recvPacket.len-4))
                        {
                            state = STS_USR_CRC;
                        }
                    }
                    break;
                case STS_USR_CRC:
                    /*
                     * 用户报文长度不包括CRC
                     */
                    crcCheck = ZCPCrcCalc(&recvPacket.len,recvPacket.len);

                    if(crcCheck == *pc)
                    {
                        state = STS_TAIL;
                    }
                    else
                    {
                        /*
                         * 统计CRC错误，重新检测报文
                         */
                        pInst->stat.userPacketCrcErr++;
                        state = STS_HEAD;
                    }
                    break;
                case STS_TAIL:
                    if(ZIGBEE_TAIL == *pc)
                    {
                        if(ZIGBEE_TYPE_ACK == zType)
                        {
                            /*
                             * ACK报文：Post信用量到ZCPSendTask
                             */
                            Semaphore_post(pInst->ackSem);
                        }
                        else
                        {
                            /*
                             * 用户报文：Post用户报文数据到userRecvMbox；
                             * 备注：若应用处理不及时，会导致数据丢失
                             */
                            pInst->stat.recvCnt++;
                            Mailbox_post(pInst->userRecvMbox,(Ptr*)&recvPacket,BIOS_NO_WAIT);
                        }
                    }
                    else
                    {
                        pInst->stat.zigbeePacketErr++;
                    }
                    state = STS_HEAD;
                    break;
                default:
                    /*
                     * 状态跳转异常
                     */
                    pInst->stat.stateErr++;
                    state = STS_HEAD;
                    break;

            }/*switch(state)*/

            /*
             * 长度减1，指针前移
             */
            recvBuf.length--;
            pc++;
        }/*while(recvBuf.length)*/

    }
}


/*****************************************************************************
 * 函数名称: void ZCPSendTask(void *param)
 * 函数说明: ZCP的发送任务
 * 输入参数:
 *          arg0: 任务参数，传入设备号
 *          arg1: 任务参数，保留
 * 输出参数: 无
 * 返 回 值: 无
 * 备注: 需保证任务不使用任何非实例的全局变量。
*****************************************************************************/
static void ZCPSendTask(UArg arg0, UArg arg1)
{
    ZCPInstance_t * pInst;
    ZCPPacket_t sendPacket;
    uartDataObj_t sendBuf;
    //ZCPPacket_t userPacket;
    Semaphore_Handle tmpSem;
    uint8_t devNum = (uint8_t)arg0;

    pInst = ZCPGetInstance(devNum);
    while(1)
    {
        Mailbox_pend(pInst->userSendMbox, (Ptr *)&sendPacket, BIOS_WAIT_FOREVER);
        tmpSem = sendPacket.ackSem;

        ZCPPack(&sendPacket,&sendBuf);

        /*
         * 发送之前清除ACK信用量，保证之后接收的为当前发送报文的ACK。
         */
        Semaphore_pend(pInst->ackSem,BIOS_NO_WAIT);

        UartNs550Send(pInst->uartDevNum,sendBuf.buffer,sendBuf.length);
        pInst->stat.sendCnt++;

        if(FALSE == Semaphore_pend(pInst->uartSendSem,ZCP_UART_SEND_TIMEOUT))
        {
            /*
             * 串口发送超时，反馈错误到应用层
             */
#if 0
            userPacket.type = ZCP_TYPE_ERROR;
            userPacket.data[0] = ZCP_ERR_SEND_TIMEOUT;
            userPacket.len = 3;
            userPacket.extSec = sendPacket.extSec;
            pInst->stat.sendErr++;
            Mailbox_post(pInst->userRecvMbox,(Ptr*)&userPacket,BIOS_NO_WAIT);
#endif
            pInst->stat.sendErr++;
            continue;
        }

#if 0
        if(FALSE == Semaphore_pend(pInst->ackSem,ZCP_ACK_TIMEOUT))
        {
            /*
             * 收到ACK超时
             */
            userPacket.type = ZCP_TYPE_ERROR;
            userPacket.data[0] = ZCP_ERR_ACK_TIMEOUT;
            pInst->stat.ackTimeout++;

        }
        else
        {
            /*
             * 收到ACK，返回ack数据
             */
            userPacket.type = ZCP_TYPE_ACK;
            userPacket.data[0] = pInst->ackSts;
            if(pInst->ackSts != ZIGBEE_ACK_OK)
            {
                pInst->stat.ackErr++;

            }

        }

        userPacket.len = 3;
        userPacket.extSec = sendPacket.extSec;
        Mailbox_post(pInst->userRecvMbox,(Ptr*)&userPacket,BIOS_NO_WAIT);
#endif
        if(FALSE == Semaphore_pend(pInst->ackSem,ZCP_ACK_TIMEOUT))
        {
            /*
             * 收到ACK超时
             */
            pInst->stat.ackTimeout++;

        }
        else
        {
            /*
             * 收到ACK，返回ack数据
             */
            if(pInst->ackSts != ZIGBEE_ACK_OK)
            {
                pInst->stat.ackErr++;
            }
            else
            {
                if(tmpSem != NULL)
                {
                    /*
                     * ACK成功，且任务注册了Semaphore
                     * Post信用量
                     */
                    Semaphore_post(tmpSem);
                }
            }

        }

    }/*while(1)*/
}

/*****************************************************************************
 * 函数名称: uint8_t ZCPInit(ZCPInstance_t *pInst, uint8_t devNum,uint8_t uartDevNum)
 * 函数说明: ZCP初始化（初始化信用量、发送任务、接收任务以及统计计数器）
 * 输入参数:
 *          pInst: 实例指针
 *          devNum: ZCP的设备号
 *          uartDevNum: ZCP设备使用的串口号
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/
uint8_t ZCPInit(ZCPInstance_t *pInst, uint8_t devNum,uint8_t uartDevNum,uint16_t src_addr)
{
    Semaphore_Params semParams;
    Mailbox_Params mboxParams;
    Task_Handle task;
    Task_Params taskParams;

    if(devNum > MAX_ZCP_DEVIVE_NUMS)
    {
        /*
         * 初始化设备号，超出最大设备数
         */
        return ZCP_INIT_FAILED;
    }

    if(ZCPCfgTable[devNum].isUsed == 0)
    {
        ZCPCfgTable[devNum].isUsed = 1;
        ZCPCfgTable[devNum].pInst = pInst;
    }
    else
    {
        /*
         * 设备已经被使用
         */
        return ZCP_INIT_FAILED;
    }

    pInst->uartDevNum = uartDevNum;

    /*
     * 初始化接收邮箱 和信用量
     */
    Mailbox_Params_init(&mboxParams);
    pInst->uartRecvMbox = Mailbox_create (sizeof (uartDataObj_t),ZCP_MBOX_DEPTH, &mboxParams, NULL);
    pInst->userRecvMbox = Mailbox_create (sizeof (ZCPPacket_t),ZCP_MBOX_DEPTH, &mboxParams, NULL);
    pInst->userSendMbox = Mailbox_create (sizeof (ZCPPacket_t),ZCP_MBOX_DEPTH, &mboxParams, NULL);

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    pInst->uartSendSem = Semaphore_create(1, &semParams, NULL);
    pInst->ackSem = Semaphore_create(0, &semParams, NULL);

    /*
     * 统计计数器清零
     */
    pInst->stat.sendCnt = 0;
    pInst->stat.recvCnt = 0;
    pInst->stat.ackErr = 0;
    pInst->stat.ackTimeout = 0;
    pInst->stat.sendErr = 0;
    pInst->stat.stateErr = 0;
    pInst->stat.userPacketErr = 0;
    pInst->stat.userPacketCrcErr = 0;
    pInst->stat.zigbeePacketErr = 0;

    /*
     * 初始化发送源地址
     */
    pInst->src_addr = src_addr;

    /*
     * 初始化串口，并启动接收
     */
    UartNs550Init(uartDevNum,ZCPUartIntrHandler);
    UartNs550Recv(uartDevNum, pInst->uartRxData.buffer, UART_REC_BUFFER_SIZE);

    /*
     * 新建任务，将设备号传递给对应接收、发送任务
     */
    Task_Params_init(&taskParams);
    taskParams.priority = 5;
    taskParams.stackSize = 2048;
    taskParams.arg0 = devNum;

    task = Task_create((Task_FuncPtr)ZCPReciveTask, &taskParams, NULL);
    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

    task = Task_create((Task_FuncPtr)ZCPSendTask, &taskParams, NULL);
    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

    return ZCP_INIT_OK;
}


Bool ZCPPend(ZCPInstance_t *pInst,ZCPPacket_t *pUserPacket,UInt timeout)
{
    return Mailbox_pend(pInst->userRecvMbox, (Ptr *)pUserPacket, timeout);
}


Bool ZCPPost(ZCPInstance_t *pInst,ZCPPacket_t *pUserPacket,UInt timeout)
{
    return Mailbox_post(pInst->userSendMbox, (Ptr *)pUserPacket, timeout);
}

/*****************************************************************************
 * 函数名称: Bool ZCPSendPacket(ZCPInstance_t *pInst,
                                ZCPUserPacket_t *userPacket,
                                Semaphore_Handle ackSem,
                                UInt timeout
                                )
 * 函数说明: ZCP报文发送函数，
 * 输入参数:
 *          pInst: 实例指针
 *          userPacket: 用户发送报文指针
 *          ackSem: ACK的返回信用量
 *          timeout: Mailbox_post的超时时间
 * 输出参数: 无
 * 返 回 值: FALSE(邮箱发送失败)/TRUE(邮箱发送成功)
 * 备注:
*****************************************************************************/
Bool ZCPSendPacket(ZCPInstance_t *pInst,
        ZCPUserPacket_t *userPacket,
        Semaphore_Handle ackSem,
        UInt timeout
        )
{
    ZCPPacket_t sendPacket;
    int32_t timeMs;

    sendPacket.addrL = userPacket->addr & 0xff;
    sendPacket.addrH = (userPacket->addr >> 8) & 0xff;
    sendPacket.type = userPacket->type;
    sendPacket.src_addr = pInst->src_addr;
    sendPacket.ackSem = ackSem;

    /*
     * 拷贝用户数据
     */
    memcpy(sendPacket.data,userPacket->data,userPacket->len);

    /*
     * 添加时间戳
     */
    userGetMS(&timeMs);
    memcpy(&(sendPacket.data[userPacket->len]),&timeMs,sizeof(timeMs));

    /*
     * 修正长度(数据长度+len+type+src_addr)
     */
    sendPacket.len = userPacket->len + 4 + sizeof(timeMs);
    return Mailbox_post(pInst->userSendMbox, (Ptr *)&sendPacket, timeout);
}

/*****************************************************************************
 * 函数名称: Bool ZCPRecvPacket(ZCPInstance_t *pInst,
                                ZCPUserPacket_t *userPacket,
                                int32_t *timestamp,
                                UInt timeout)
 * 函数说明: ZCP报文接收函数
 * 输入参数:
 *          pInst: 实例指针
 *          userPacket: 用户接收报文指针
 *          timestamp: 时间戳指针
 *          timeout: Mailbox_pend的超时时间
 * 输出参数: 无
 * 返 回 值: FALSE(邮箱发送失败)/TRUE(邮箱发送成功)
 * 备注:
*****************************************************************************/
Bool ZCPRecvPacket(ZCPInstance_t *pInst,
        ZCPUserPacket_t *userPacket,
        int32_t *timestamp,
        UInt timeout)
{
    ZCPPacket_t recvPacket;
    Bool state;
    state = Mailbox_pend(pInst->userRecvMbox, (Ptr *)&recvPacket, timeout);

    userPacket->addr = (recvPacket.addrH<<8) + recvPacket.addrL;
    if(userPacket->addr != recvPacket.src_addr)
    {
        LogMsg("ZCP Source Address Error(%x->%x)\r\n",userPacket->addr,recvPacket.src_addr);
        userPacket->addr = recvPacket.src_addr;
    }

    userPacket->type = recvPacket.type;

    /*
     * 拷贝用户数据
     */
    memcpy(userPacket->data, recvPacket.data, recvPacket.len-8);
    userPacket->len = recvPacket.len - 8;

    /*
     * 获取时间戳
     */
    if(timestamp != NULL)
    {
        memcpy(timestamp, &(recvPacket.data[recvPacket.len-8]), 4);
    }

    return state;
}
