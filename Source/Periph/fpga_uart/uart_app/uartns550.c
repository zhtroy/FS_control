/***************************** Include Files ********************************/
#include "uartns550.h"
#include "fpga_periph_def.h"

/************************** Constant Definitions ****************************/

uartCfgTable_t uartCfgTable[] = {

     /*设备0资源配置*/
    {
         /*设备号,设备基地址ַ,设备驱动时钟<Hz>*/
         0,FPGA_RS232_DEV0,80000000,
         /*波特率,数据长度,校验位,ֹͣ停止位*/
         {115200,XUN_FORMAT_8_BITS,XUN_FORMAT_EVEN_PARITY,XUN_FORMAT_1_STOP_BIT},
         /*设备OPTIONS*/
         XUN_OPTION_DATA_INTR | XUN_OPTION_FIFOS_ENABLE,
         /*FIFO触发深度*/
         XUN_FIFO_TRIGGER_08
    },
     
     /*设备1资源配置*/
    {
         /*设备号,设备基地址ַ,设备驱动时钟<Hz>*/
         1,FPGA_RS232_DEV1,80000000,
         /*波特率,数据长度,校验位,ֹͣ停止位*/
         {115200,XUN_FORMAT_8_BITS,XUN_FORMAT_NO_PARITY,XUN_FORMAT_1_STOP_BIT},
         /*设备OPTIONS*/
         XUN_OPTION_DATA_INTR | XUN_OPTION_FIFOS_ENABLE,
         /*FIFO触发深度*/
         XUN_FIFO_TRIGGER_08
    },
     
     /*设备2资源配置*/
    {
         /*设备号,设备基地址ַ,设备驱动时钟<Hz>*/
         2,FPGA_RS232_DEV2,80000000,
         /*波特率,数据长度,校验位,ֹͣ停止位*/
         {115200,XUN_FORMAT_8_BITS,XUN_FORMAT_ODD_PARITY,XUN_FORMAT_1_STOP_BIT},
         /*设备OPTIONS*/
         XUN_OPTION_DATA_INTR | XUN_OPTION_FIFOS_ENABLE,
         /*FIFO触发深度*/
         XUN_FIFO_TRIGGER_08
    },
    
     /*设备3资源配置*/
    {
         /*设备号,设备基地址ַ,设备驱动时钟<Hz>*/
         3,FPGA_RS232_DEV3,80000000,
         /*波特率,数据长度,校验位,ֹͣ停止位*/
         {115200,XUN_FORMAT_8_BITS,XUN_FORMAT_NO_PARITY,XUN_FORMAT_1_STOP_BIT},
         /*设备OPTIONS*/
         XUN_OPTION_DATA_INTR | XUN_OPTION_FIFOS_ENABLE,
         /*FIFO触发深度*/
         XUN_FIFO_TRIGGER_08
    },
    
    /*设备4资源配置*/
    {
         /*设备号,设备基地址ַ,设备驱动时钟<Hz>*/
         4,FPGA_RS232_DEV4,80000000,
         /*波特率,数据长度,校验位,ֹͣ停止位*/
         {115200,XUN_FORMAT_8_BITS,XUN_FORMAT_NO_PARITY,XUN_FORMAT_1_STOP_BIT},
         /*设备OPTIONS*/
         XUN_OPTION_DATA_INTR | XUN_OPTION_FIFOS_ENABLE,
         /*FIFO触发深度*/
         XUN_FIFO_TRIGGER_08
    },
    
     /*设备5资源配置*/
    {
         /*设备号,设备基地址ַ,设备驱动时钟<Hz>*/
         5,FPGA_RS232_DEV5,80000000,
         /*波特率,数据长度,校验位,ֹͣ停止位*/
         {115200,XUN_FORMAT_8_BITS,XUN_FORMAT_NO_PARITY,XUN_FORMAT_1_STOP_BIT},
         /*设备OPTIONS*/
         XUN_OPTION_DATA_INTR | XUN_OPTION_FIFOS_ENABLE,
         /*FIFO触发深度*/
         XUN_FIFO_TRIGGER_08
    }
};

/*获取设备数量*/
#define UART_CFG_NUM (NELEMENTS(uartCfgTable)-1)

/*****************************************************************************
 * 函数名称: int32_t UartNs550Init(uint16_t deviceNum,XUartNs550_Handler funcPtr)
 * 函数说明: 初始化UART设备维护表 该表由项目开发人员必须调用 否则资源不能清空
 * 输入参数:
 *        deviceNum：设备号
 *        funcPtr：中断回调函数
 * 输出参数: 无
 * 返 回 值: 0(成功)/负数(失败)
 * 备注:
*****************************************************************************/
int32_t UartNs550Init(uint16_t deviceNum,XUartNs550_Handler funcPtr)
{
    int32_t status;
    XUartNs550 * instancePtr;
    
    if(deviceNum > UART_CFG_NUM)
    {
        return XST_FAILURE;
    }
    
    instancePtr = &(uartCfgTable[deviceNum].instance);
    
    /* 设备硬件复位 */
    UartNs550HardReset(deviceNum);
    
    /* 根据配置表初始化成员变量 */
	instancePtr->BaseAddress = uartCfgTable[deviceNum].baseAddress;
	instancePtr->InputClockHz = uartCfgTable[deviceNum].inputClockHz;

	/*
	 * Initialize the instance data to some default values and setup
	 * a handler and CallBackRef.
 	 */
 	
	instancePtr->Handler = funcPtr;
    //instancePtr->CallBackRef = instancePtr;
    instancePtr->CallBackRef = &(uartCfgTable[deviceNum].deviceNum);
    
	instancePtr->SendBuffer.NextBytePtr = NULL;
	instancePtr->SendBuffer.RemainingBytes = 0;
	instancePtr->SendBuffer.RequestedBytes = 0;
	instancePtr->ReceiveBuffer.NextBytePtr = NULL;
	instancePtr->ReceiveBuffer.RemainingBytes = 0;
	instancePtr->ReceiveBuffer.RequestedBytes = 0;

    /*
	 * Indicate the instance is now ready to use, initialized without error
	 */
	instancePtr->IsReady = XIL_COMPONENT_IS_READY;

    status = XUartNs550_SetDataFormat(instancePtr, 
        &(uartCfgTable[deviceNum].dataFormat));
    
	if (status != XST_SUCCESS) {
		instancePtr->IsReady = 0;
		return status;
	}

    status = XUartNs550_SetOptions(instancePtr, 
        uartCfgTable[deviceNum].options);
    
	if (status != XST_SUCCESS) {
		instancePtr->IsReady = 0;
		return status;
	}
    
    status = XUartNs550_SetFifoThreshold(instancePtr, 
        uartCfgTable[deviceNum].triggerLevel);
    
    if (status != XST_SUCCESS) {
		instancePtr->IsReady = 0;
		return status;
    }
    
    /* 清除历史状态 */
    XUartNs550_ClearStats(instancePtr);
    
    return XST_SUCCESS;
}

/*****************************************************************************
 * 函数名称: uint32_t UartNs550Send(uint16_t deviceNum, uint8_t *bufferPtr,uint32_t numBytes)
 * 函数说明: 串口发送
 * 输入参数:
 *        deviceNum：设备号
 *        bufferPtr：发送缓冲指针
 *        numBytes：发送数据长度，单位字节
 * 输出参数: 无
 * 返 回 值: 调用函数时发送的数据长度。
 * 备注：中断模式，非阻塞式发送
*****************************************************************************/
uint32_t UartNs550Send(uint16_t deviceNum, uint8_t *bufferPtr,uint32_t numBytes)
{
    
    XUartNs550 * instancePtr = &(uartCfgTable[deviceNum].instance);
    
    return XUartNs550_Send(instancePtr, bufferPtr, numBytes);
}

/*****************************************************************************
 * 函数名称: uint32_t UartNs550Recv(uint16_t deviceNum, uint8_t *bufferPtr,uint32_t numBytes)
 * 函数说明: 串口接收
 * 输入参数:
 *        deviceNum：设备号
 *        bufferPtr：接收缓冲指针
 *        numBytes：接收数据长度，单位字节
 * 输出参数: 无
 * 返 回 值: 调用函数时接收的数据长度
 * 备注：中断模式，非阻塞式接收
*****************************************************************************/
uint32_t UartNs550Recv(uint16_t deviceNum, uint8_t *bufferPtr,uint32_t numBytes)
{
    
    XUartNs550 * instancePtr = &(uartCfgTable[deviceNum].instance);
    
    return XUartNs550_Recv(instancePtr, bufferPtr, numBytes);
}

/*****************************************************************************
 * 函数名称: void UartNs550IntrHandler(uint16_t deviceNum)
 * 函数说明: 串口中断处理函数
 * 输入参数:
 *        deviceNum：设备号
 * 输出参数: 无
 * 返 回 值: 无
 * 备注：
*****************************************************************************/
void UartNs550IntrHandler(uint16_t deviceNum)
{
    XUartNs550 * instancePtr = &(uartCfgTable[deviceNum].instance);
    XUartNs550_InterruptHandler(instancePtr);
}

/*****************************************************************************
 * 函数名称: void UartNs550HardIntMask(uint16_t deviceNum)
 * 函数说明: 串口硬件中断Mask
 * 输入参数:
 *        deviceNum：设备号
 * 输出参数: 无
 * 返 回 值: 无
 * 备注：被Mask的设备才能有效上报中断
*****************************************************************************/
void UartNs550HardIntMask(uint16_t deviceNum)
{
    uint8_t Reg;
    Reg = *(volatile uint16_t *) (UART_INT_MASK_ADDR);
    *(volatile uint16_t *) (UART_INT_MASK_ADDR) =  Reg | (1 << deviceNum);
}

/*****************************************************************************
 * 函数名称: void UartNs550HardIntUnmask(uint16_t deviceNum)
 * 函数说明: 串口硬件中断Unmask
 * 输入参数:
 *        deviceNum：设备号
 * 输出参数: 无
 * 返 回 值: 无
 * 备注：取消Mask的设备无法有效上报中断
*****************************************************************************/
void UartNs550HardIntUnmask(uint16_t deviceNum)
{
    uint8_t Reg;
    Reg = *(volatile uint16_t *) (UART_INT_MASK_ADDR);
    *(volatile uint16_t *) (UART_INT_MASK_ADDR) =  Reg & (~(1 << deviceNum));
}

/*****************************************************************************
 * 函数名称: UartNs550GetLastErrors(uint16_t deviceNum)
 * 函数说明: 串口错误统计
 * 输入参数:
 *        deviceNum：设备号
 * 输出参数: 无
 * 返 回 值: 无
 * 备注：
*****************************************************************************/
uint8_t UartNs550GetLastErrors(uint16_t deviceNum)
{
    XUartNs550 * instancePtr = &(uartCfgTable[deviceNum].instance);
    
    return XUartNs550_GetLastErrors(instancePtr);
}

/*****************************************************************************
 * 函数名称: uint8_t UartNs550Getc(uint16_t deviceNum)
 * 函数说明: 字节获取(阻塞式)
 * 输入参数:
 *        deviceNum:设备号
 * 输出参数: 无
 * 返 回 值: 获取的数据
 * 备注：
*****************************************************************************/
uint8_t UartNs550Getc(uint16_t deviceNum)
{

    XUartNs550 * instancePtr = &(uartCfgTable[deviceNum].instance);

    return XUartNs550_RecvByte(instancePtr->BaseAddress);
}

/*****************************************************************************
 * 函数名称: void UartNs550Putc(uint16_t deviceNum, uint8_t data)
 * 函数说明: 发送字节(阻塞式)
 * 输入参数:
 *        deviceNum:设备号
 *        data:发送字节数据
 * 输出参数: 无
 * 返 回 值: 无
 * 备注：
*****************************************************************************/
void UartNs550Putc(uint16_t deviceNum, uint8_t data)
{

    XUartNs550 * instancePtr = &(uartCfgTable[deviceNum].instance);

    XUartNs550_SendByte(instancePtr->BaseAddress, data);
}

/*****************************************************************************
 * 函数名称: uint32_t UartNs550Puts(uint16_t deviceNum, int8_t *pTxBuffer, int32_t numBytesToWrite)
 * 函数说明: 发送字符串(阻塞式)
 * 输入参数:
 *        deviceNum:设备号
 *        pTxBuffer:发送字符串指针
 *        numBytesToWrite:发送数据长度
 * 输出参数: 无
 * 返 回 值: 实际发送的数据长度
 * 备注：
 *     发送结束的条件：
 *     a)numBytesToWrite为正数，发送计数count等于numBytesToWrite
 *     b)numBytesToWrite为正数，发送过程中检测到字符'\0'
 *     c)numBytesToWrite为负数时，依赖字符串结束符'\0'
*****************************************************************************/
uint32_t UartNs550Puts(uint16_t deviceNum,int8_t *pTxBuffer, int32_t numBytesToWrite)
{
     uint32_t count = 0;
     uint32_t flag = 0;

     if(numBytesToWrite < 0)
     {
          flag = 1;
     }

     while('\0' != *pTxBuffer)
     {
          /* Checks if data is a newline character. */
          if('\n' == *pTxBuffer)
          {
               /* Ensuring applicability to serial console.*/
              UartNs550Putc(deviceNum, '\r');
              UartNs550Putc(deviceNum, '\n');
          }
          else
          {
              UartNs550Putc(deviceNum, (uint8_t)*pTxBuffer);
          }
          pTxBuffer++;
          count++;

          if((0 == flag) && (count == numBytesToWrite))
          {
               break;
          }

     }
   /* Returns the number of bytes written onto the transmitter FIFO. */
   return count;
}

/*****************************************************************************
 * 函数名称: uint32_t UartNs550SendBlock(uint16_t deviceNum,const int8_t *pcBuf, uint32_t numBytesToWrite)
 * 函数说明: 发送数据(阻塞式)
 * 输入参数:
 *        deviceNum:设备号
 *        pcBuf:发送缓冲指针
 *        numBytesToWrite:发送数据长度
 * 输出参数: 无
 * 返 回 值: 发送数据长度
 * 备注：
*****************************************************************************/
uint32_t UartNs550SendBlock(uint16_t deviceNum,int8_t *pcBuf, uint32_t numBytesToWrite)
{
    uint32_t uIdx;

    for(uIdx = 0; uIdx < numBytesToWrite; uIdx++)
    {
        UartNs550Putc(deviceNum,pcBuf[uIdx]);
    }

    return(uIdx);
}

/*****************************************************************************
 * 函数名称: uint32_t UartNs550RecvBlock(uint16_t deviceNum,int8_t *recvBuf,uint32_t numBytesToRecv)
 * 函数说明: 接收指定长度数据(阻塞式)
 * 输入参数:
 *        deviceNum:设备号
 *        recvBuf:接收缓冲指针
 *        numBytesToWrite:发送数据长度
 * 输出参数: 无
 * 返 回 值: 接收数据长度
 * 备注：
*****************************************************************************/
uint32_t UartNs550RecvBlock(uint16_t deviceNum,int8_t *recvBuf, uint32_t numBytesToRecv)
{
    uint32_t uIdx;

    for(uIdx = 0; uIdx < numBytesToRecv; uIdx++)
    {
        recvBuf[uIdx] = UartNs550Getc(deviceNum);
    }

    return(uIdx);
}

/*****************************************************************************
 * 函数名称: uint32_t UartNs550DeviceIsExist(uint16_t deviceNum)
 * 函数说明: 判断设备是否被初始化
 * 输入参数:
 *        deviceNum:设备号
 * 输出参数: 无
 * 返 回 值: 0(设备不存在)/1(设备存在)
 * 备注：
*****************************************************************************/
uint32_t UartNs550DeviceIsExist(uint16_t deviceNum)
{
    XUartNs550 * instancePtr = &(uartCfgTable[deviceNum].instance);
    if(instancePtr == NULL)
        return 0;
    else
        return 1;
}
    
void UartNs550RS485TxDisable(uint16_t deviceNum)
{
    uint8_t mcrRegister;
    XUartNs550 * instancePtr = &(uartCfgTable[deviceNum].instance);

    mcrRegister = XUartNs550_ReadReg(instancePtr->BaseAddress,
						XUN_MCR_OFFSET);
    
    XUartNs550_WriteReg(instancePtr->BaseAddress, XUN_MCR_OFFSET,
			 	mcrRegister | XUN_MCR_RTS);
}

void UartNs550RS485TxEnable(uint16_t deviceNum)
{
    uint8_t mcrRegister;
    XUartNs550 * instancePtr = &(uartCfgTable[deviceNum].instance);

    mcrRegister = XUartNs550_ReadReg(instancePtr->BaseAddress,
						XUN_MCR_OFFSET);
    
    XUartNs550_WriteReg(instancePtr->BaseAddress, XUN_MCR_OFFSET,
			 	mcrRegister &(~XUN_MCR_RTS) );
}

