#ifndef __UARTNS550_H__
#define __UARTNS550_H__
#include "Xuartns550.h"
#include "Xparameters.h"
#include "common.h"

/* 宏定义 */
/* 计算结构体数量 */
#define NELEMENTS(array)		/* number of elements in an array */ \
		(sizeof (array) / sizeof ((array) [0]))

/* FPGA串口特殊寄存器地址 */
#define UART_INT_STATUS_ADDR    FPGA_RS232_INT
#define UART_RESET_ADDR         FPGA_RS232_RESET
#define UART_INT_ENABLE_ADDR    FPGA_RS232_INT_ENB
#define UART_INT_MASK_ADDR      FPGA_RS232_INT_MASK

/* 串口接收Buffer的大小 */
#define UART_REC_BUFFER_SIZE (64)

/* 串口设备硬件复位 */
#define UartNs550HardReset(deviceNum) do {\
    /* 串口控制器硬件复位 */ \
        *(volatile uint16_t *) (UART_RESET_ADDR) = (1 << deviceNum); \
        *(volatile uint16_t *) (UART_RESET_ADDR) = 0; \
    }while(0)

/* 串口设备硬件中断全局使能 */
#define UartNs550HardIntEnable() \
    (*(volatile uint16_t *) (UART_INT_ENABLE_ADDR) = 1)

/* 串口设备硬件中断全局关闭 */
#define UartNs550HardIntDisable() \
    (*(volatile uint16_t *) (UART_INT_ENABLE_ADDR) = 0)

/* 串口设备硬件中断全局Mask */
#define UartNs550HardIntMaskAll() \
    (*(volatile uint16_t *) (UART_INT_MASK_ADDR) = 0xff)

/* 串口设备硬件中断全局取消Mask */
#define UartNs550HardIntUnMaskAll() \
    (*(volatile uint16_t *) (UART_INT_MASK_ADDR) = 0)

/* 串口设备中断状态：返回产生中断的设备 */
#define UartNs550GetHardIntStatus() \
    (*(volatile uint16_t *) (UART_INT_STATUS_ADDR))

/* 数据类型定义 */
/* 串口配置表 */
typedef struct _HW_UART_CFG 
{
    uint16_t                     deviceNum;		/**< Unique Num  of device */
	uint32_t                     baseAddress;	/**< Base address of device */
	uint32_t                     inputClockHz;	/**< Input clock frequency */
    XUartNs550Format        dataFormat;     /**< Data Format */
    uint16_t                     options;        /**< Options */
    uint8_t                      triggerLevel;   /**< FIFO TriggerLevel */
    XUartNs550              instance;       /**Ns550 Instance */
}uartCfgTable_t;

/* 串口数据结构 */
typedef struct _UART_DATA_OBJ
{
    uint8_t                      length;
    uint8_t                      buffer[UART_REC_BUFFER_SIZE];
}uartDataObj_t;

/* 外部函数API声明 */
int32_t UartNs550Init(uint16_t deviceNum,XUartNs550_Handler funcPtr);
uint32_t UartNs550Send(uint16_t deviceNum, uint8_t *bufferPtr,uint32_t numBytes);
uint32_t UartNs550Recv(uint16_t deviceNum, uint8_t *bufferPtr,uint32_t numBytes);
void UartNs550IntrHandler(uint16_t deviceNum);
void UartNs550HardIntMask(uint16_t deviceNum);
void UartNs550HardIntUnmask(uint16_t deviceNum);
uint8_t UartNs550GetLastErrors(uint16_t deviceNum);
void UartNs550SetMode(uint16_t deviceNum,uint8_t mode);
void UartNs550RS485TxEnable(uint16_t deviceNum);
void UartNs550RS485TxDisable(uint16_t deviceNum);
uint8_t UartNs550Getc(uint16_t deviceNum);
void UartNs550Putc(uint16_t deviceNum, uint8_t data);
uint32_t UartNs550Puts(uint16_t deviceNum,int8_t *pTxBuffer, int32_t numBytesToWrite);
uint32_t UartNs550SendBlock(uint16_t deviceNum,int8_t *pcBuf, uint32_t numBytesToWrite);
uint32_t UartNs550RecvBlock(uint16_t deviceNum,int8_t *recvBuf, uint32_t numBytesToRecv);
uint32_t UartNs550DeviceIsExist(uint16_t deviceNum);
#endif




