#include "mpu9250_iic.h"
#include "mpu9250_drv.h"
#include "soc_C6748.h" 
#include "interrupt.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Timestamp.h>

/* 函数定义 */
static void mpu9250IntrHandler(void *callBackRef, uint32_t event, uint32_t eventData);

/* 全局变量定义 */
IICObj_t mpu9250IICInst;
static Semaphore_Handle transCompSem;
static const IICConfig_t cfg = {
    .devAddr     = SOC_I2C_0_REGS,
    .sysClk      = 24000000,
    .internalClk = 8000000,
    .iicFreq     = 100000,
    .intrEventID = SYS_INT_I2C0_INT,
    .handler     = mpu9250IntrHandler,
    .callBackRef = NULL
};

/* 宏定义 */
#define I2C_TIMEOUT_MS  (2500)



/*****************************************************************************
 * 函数名称: void mpu9250I2CInit(void)
 * 函数说明: 初始化mpu9250连接的I2C接口
 * 输入参数: void
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/
void mpu9250I2CInit(void)
{
	/*初始化IIC配置*/
	IICCfgInit(&mpu9250IICInst,&cfg);

	/* 初始化信用量配置 */
    Semaphore_Params semParams;
	Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    transCompSem = Semaphore_create(0, &semParams, NULL);
}

/*****************************************************************************
 * 函数名称: int32_t mpu9250WriteBytes(uint8_t slvAddr, uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr)
 * 函数说明: mpu9250的IIC写入函数
 * 输入参数:
 * 		  slvAddr: 从设备地址
 * 		  regAddr: 寄存器起始地址
 * 		  numBytes: 写入字节数
 * 		  dataPtr: 数据缓冲的指针
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
int32_t mpu9250WriteBytes(uint8_t slvAddr, uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr)
{
	/* 设置从设备地址 */
    IICSetSlaveAddr(&mpu9250IICInst, slvAddr);

    /* 发送寄存器地址和数据 */
    IICSendBytes(&mpu9250IICInst, regAddr, dataPtr,numBytes+1);

    if(FALSE == Semaphore_pend(transCompSem,I2C_TIMEOUT_MS))
        return -1;
    return 0;
}

/*
 * 成功返回写入数量
 * 失败返回-1
 */
int32_t mpu9250WriteBytesFreeLength(uint8_t slvAddr, uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr)
{
	int len;
	while(numBytes > 0)
	{
		len = (regAddr/8 + 1)*8 - regAddr;
		if(len <= numBytes)
		{
			numBytes -= len;

		}
		else
		{
			len = numBytes;
			numBytes = 0;
		}
		if(mpu9250WriteBytes(EEPROM_SLV_ADDR,regAddr,len,dataPtr) == -1)
		{
			return -1;
		}
		dataPtr+=len;
		regAddr += len;
		Task_sleep(10);
	}
	return numBytes;
}

/*****************************************************************************
 * 函数名称: int32_t mpu9250ReadBytes(uint8_t slvAddr,uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr)
 * 函数说明: mpu9250的IIC读取函数
 * 输入参数:
 * 		  slvAddr: 从设备地址
 * 		  regAddr: 寄存器起始地址
 * 		  numBytes: 读取字节数
 * 输出参数: dataPtr: 接收数据缓冲的指针
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
int32_t mpu9250ReadBytes(uint8_t slvAddr,uint8_t regAddr, uint8_t numBytes, uint8_t *dataPtr)
{
	/* 设置从设备地址 */
    IICSetSlaveAddr(&mpu9250IICInst, slvAddr);
    
    /* 发送寄存器地址 */
    IICSendBytes(&mpu9250IICInst, regAddr,NULL, 1);
    if(FALSE == Semaphore_pend(transCompSem,I2C_TIMEOUT_MS))
    	return -1;
    
    /* 接收数据 */
    IICRecvBytes(&mpu9250IICInst, dataPtr, numBytes);
    if(FALSE == Semaphore_pend(transCompSem,I2C_TIMEOUT_MS))
        return -1;

    return 0;
}

/*****************************************************************************
 * 函数名称: int32_t mpu9250GetMS(uint32_t *countMS)
 * 函数说明: 获取毫秒时间戳
 * 输入参数: 无
 * 输出参数:
 *		  countMS: 毫秒计数指针
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
int32_t mpu9250GetMS(uint32_t *countMS)
{
	Types_FreqHz freq;
	Types_Timestamp64 timestamp;
	long long timecycle;
	long long freqency;
	Timestamp_getFreq(&freq);
	Timestamp_get64(&timestamp);
	timecycle = _itoll(timestamp.hi, timestamp.lo);
	freqency  = _itoll(freq.hi, freq.lo);
	countMS[0] = timecycle/(freqency/1000);
	return 0;
}

/*****************************************************************************
 * 函数名称: static void mpu9250IntrHandler(void *callBackRef, uint32_t event, uint32_t eventData)
 * 函数说明: 中断服务函数
 * 输入参数:
 * 		  callBackRef: 回调函数
 * 		  event: 事件
 * 		  eventData: 事件数据
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/
static void mpu9250IntrHandler(void *callBackRef, uint32_t event, uint32_t eventData)
{
	if(IIC_EVENT_RECV_ERROR == event)
	{
		/* TODO:接收错误，暂未做处理  */
		Semaphore_post(transCompSem);
	}
	else if(IIC_EVENT_SEND_ERROR == event)
	{
		/* TODO:发送错误，暂未做处理  */
		Semaphore_post(transCompSem);
	}
	else if(IIC_EVENT_TRANS_COMP == event)
	{
		Semaphore_post(transCompSem);
	}
	else;
}
