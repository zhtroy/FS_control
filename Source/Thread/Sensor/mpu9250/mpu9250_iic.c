#include "mpu9250_iic.h"
#include "i2c.h"
#include "interrupt.h"
#include <stddef.h>

void IICCfgInit(IICObj_t *insPtr,const IICConfig_t *cfgPtr)
{
    insPtr->baseAddr = cfgPtr->devAddr;
    insPtr->slaveAddr = 0x00;
    
    insPtr->sendBuffer.nextBytePtr = NULL;
    insPtr->sendBuffer.remainingBytes = 0;
    insPtr->sendBuffer.requestedBytes = 0;
    insPtr->recvBuffer.nextBytePtr = NULL;
    insPtr->recvBuffer.remainingBytes = 0;
    insPtr->recvBuffer.requestedBytes = 0;
    insPtr->handler = cfgPtr->handler;
    insPtr->callBackRef = cfgPtr->callBackRef;
    insPtr->intrEventID = cfgPtr->intrEventID;

    I2CMasterDisable(insPtr->baseAddr);
    I2CMasterInitExpClk(insPtr->baseAddr, cfgPtr->sysClk , cfgPtr->internalClk, cfgPtr->iicFreq);
    I2CMasterEnable(insPtr->baseAddr);
}

void IICSendBytes(IICObj_t *insPtr,uint8_t regAddr,uint8_t *bufPtr,uint8_t numBytes)
{
    insPtr->sendBuffer.requestedBytes = numBytes;
	insPtr->sendBuffer.remainingBytes = numBytes;
    insPtr->regAddr = regAddr;
	insPtr->sendBuffer.nextBytePtr = bufPtr;
    
    if(I2CMasterBusBusy(insPtr->baseAddr))
    {
    	return;
    }

    I2CMasterSlaveAddrSet(insPtr->baseAddr, insPtr->slaveAddr);
    
    I2CSetDataCount(insPtr->baseAddr, insPtr->sendBuffer.requestedBytes);

    I2CMasterControl(insPtr->baseAddr, I2C_CFG_MST_TX | I2C_CFG_STOP );

    I2CMasterIntEnableEx(insPtr->baseAddr, I2C_INT_TRANSMIT_READY | I2C_INT_STOP_CONDITION | I2C_INT_NO_ACK);

    I2CMasterStart(insPtr->baseAddr);
}

void IICRecvBytes(IICObj_t *insPtr,uint8_t *bufPtr,uint8_t numBytes)
{
    insPtr->recvBuffer.requestedBytes = numBytes;
	insPtr->recvBuffer.remainingBytes = numBytes;
	insPtr->recvBuffer.nextBytePtr = bufPtr;
    
    if(I2CMasterBusBusy(insPtr->baseAddr))
    {
    	return;
    }
    
    I2CMasterSlaveAddrSet(insPtr->baseAddr, insPtr->slaveAddr);

    I2CSetDataCount(insPtr->baseAddr, insPtr->recvBuffer.requestedBytes);

    I2CMasterControl(insPtr->baseAddr, I2C_CFG_MST_RX | I2C_CFG_STOP );

    I2CMasterIntEnableEx(insPtr->baseAddr, I2C_INT_DATA_READY | I2C_INT_STOP_CONDITION | I2C_INT_NO_ACK);

    I2CMasterStart(insPtr->baseAddr);
}

void IICSetSlaveAddr(IICObj_t *insPtr,uint8_t addr)
{
    insPtr->slaveAddr = addr;
}

void IICInterruptHandler(IICObj_t *insPtr)
{
    volatile unsigned int intCode = 0;

    // 获取中断状态
    intCode = I2CInterruptVectorGet(insPtr->baseAddr);

    while(intCode != 0)
    {
        // 清除中断事件
        IntEventClear(insPtr->intrEventID);

        if (intCode == I2C_INTCODE_TX_READY)
        {
            if(insPtr->sendBuffer.remainingBytes != 0)
            {
                if(insPtr->sendBuffer.remainingBytes == insPtr->sendBuffer.requestedBytes)
                {
                	/* 发送寄存器地址 */
                    I2CMasterDataPut(insPtr->baseAddr, insPtr->regAddr);
                    insPtr->sendBuffer.remainingBytes--;
                }
                else
                {
                	/* 发送数据 */
                    I2CMasterDataPut(insPtr->baseAddr, *(insPtr->sendBuffer.nextBytePtr));
                    insPtr->sendBuffer.nextBytePtr++;
                    insPtr->sendBuffer.remainingBytes--;
                }
            }
        }
        else if(intCode == I2C_INTCODE_RX_READY)
        {
            if(insPtr->recvBuffer.remainingBytes != 0)
            {
            	/* 接收数据并放入缓存中 */
                *(insPtr->recvBuffer.nextBytePtr) = I2CMasterDataGet(insPtr->baseAddr);
                insPtr->recvBuffer.remainingBytes--;
                insPtr->recvBuffer.nextBytePtr++;
            }
            else
            {
            	/* 多余/异常接收中断，获取并丢弃数据 */
                I2CMasterDataGet(insPtr->baseAddr);
            }
        }
        else if (intCode == I2C_INTCODE_STOP)
        {
            I2CMasterIntDisableEx(insPtr->baseAddr, I2C_INT_TRANSMIT_READY |
                                                    I2C_INT_DATA_READY |
                                                    I2C_INT_NO_ACK |
                                                    I2C_INT_STOP_CONDITION);
            
            if(insPtr->sendBuffer.remainingBytes != 0)
            {
            	insPtr->handler(insPtr,IIC_EVENT_SEND_ERROR,insPtr->sendBuffer.requestedBytes - insPtr->sendBuffer.remainingBytes);
            	insPtr->sendBuffer.remainingBytes = 0;
            }
            else if(insPtr->recvBuffer.remainingBytes != 0)
            {
            	insPtr->handler(insPtr,IIC_EVENT_RECV_ERROR,insPtr->recvBuffer.requestedBytes - insPtr->recvBuffer.remainingBytes);
            	insPtr->recvBuffer.remainingBytes = 0;
            }
            else
                insPtr->handler(insPtr,IIC_EVENT_TRANS_COMP,insPtr->recvBuffer.requestedBytes);
        }
        else;

        if (intCode == I2C_INTCODE_NACK)
        {
            I2CMasterIntDisableEx(insPtr->baseAddr, I2C_INT_TRANSMIT_READY |
                                                   I2C_INT_DATA_READY |
                                                   I2C_INT_NO_ACK |
                                                   I2C_INT_STOP_CONDITION);
            // 产生停止位
            I2CMasterStop(insPtr->baseAddr);

            I2CStatusClear(insPtr->baseAddr, I2C_CLEAR_STOP_CONDITION);

             // 清除中断
            IntEventClear(insPtr->intrEventID);
             
            insPtr->handler(insPtr,IIC_EVENT_SEND_ERROR,insPtr->sendBuffer.requestedBytes - insPtr->sendBuffer.remainingBytes);
        }
        else;

        if (I2CMasterIntStatus(insPtr->baseAddr) & I2C_ICSTR_NACKSNT)
        {
            I2CMasterIntDisableEx(insPtr->baseAddr, I2C_INT_TRANSMIT_READY |
                                                   I2C_INT_DATA_READY |
                                                   I2C_INT_NO_ACK |
                                                   I2C_INT_STOP_CONDITION);

            // 产生停止位
            I2CMasterStop(insPtr->baseAddr);

            I2CStatusClear(insPtr->baseAddr, (I2C_CLEAR_NO_ACK_SENT |
                                             I2C_CLEAR_STOP_CONDITION));

            // 清除中断
            IntEventClear(insPtr->intrEventID);
            
            insPtr->handler(insPtr,IIC_EVENT_RECV_ERROR,insPtr->recvBuffer.requestedBytes - insPtr->recvBuffer.remainingBytes);

        }
        else;
        intCode = I2CInterruptVectorGet(insPtr->baseAddr);
    }
}
