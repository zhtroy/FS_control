/****************************************************************************/
/*                                                                          */
/*              ���� DSP6748 ��������غ���?                                 */
/*                                                                          */
/*              2014��07��12��                                              */
/*                                                                          */
/****************************************************************************/
#include "stdint.h"
#include "hw_types.h"

#include "TL6748.h"
#include "soc_C6748.h"
#include "hw_syscfg0_C6748.h"

#include "uart.h"
#include "psc.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

/****************************************************************************/
/*                                                                          */
/*            将库函数重定向到这里的代码，使用串口1    */
/*                                                                          */
/****************************************************************************/
#define UART_STDIO_INSTANCE             (1)
#define UART_CONSOLE_BASE               (SOC_UART_1_REGS)

static void UARTStdioInitExpClk(unsigned int baudRate,
                                unsigned int rxTrigLevel);
void UARTConsolePutc(uint8_t data);
uint8_t UARTConsoleGetc(void);
void UARTConsoleInit(void);

static unsigned char rxData = 0;
static Semaphore_Handle rxSem = NULL;

static void UARTStdioInitExpClk(unsigned int baudRate, unsigned int rxTrigLevel)
{
     UARTEnable(UART_CONSOLE_BASE);

     UARTConfigSetExpClk(UART_CONSOLE_BASE, 
                         SOC_UART_1_MODULE_FREQ,
                         baudRate, 
                         UART_WORDL_8BITS,
                         UART_OVER_SAMP_RATE_16);


     UARTFIFOEnable(UART_CONSOLE_BASE);

     UARTFIFOLevelSet(UART_CONSOLE_BASE, rxTrigLevel);

}

/****************************************************************************/
/*                                                                          */
/*              UART 中断初始化                                             */
/*                                                                          */
/****************************************************************************/
static void UARTInterruptInit(void)
{
    // 使能中断
    unsigned int intFlags = 0;
    intFlags |= (UART_INT_LINE_STAT  |  \
                 UART_INT_RXDATA_CTI);
    UARTIntEnable(SOC_UART_1_REGS, intFlags);
}

static void UART1InitSem()
{
    Semaphore_Params semParams;
    /* 初始化接收信用量 */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    rxSem = Semaphore_create(0, &semParams, NULL);
}

void UARTConsoleInit(void)
{
     #if (0 == UART_STDIO_INSTANCE)
     {
          PSCModuleControl(SOC_PSC_0_REGS,9, 0, PSC_MDCTL_NEXT_ENABLE);
          UARTPinMuxSetup(0, FALSE);
     }
     
     #elif (1 == UART_STDIO_INSTANCE)
     {
          PSCModuleControl(SOC_PSC_1_REGS,12, 0, PSC_MDCTL_NEXT_ENABLE);
          UARTPinMuxSetup(1, FALSE);
     }

     #else 
     {
          PSCModuleControl(SOC_PSC_1_REGS,13, 0, PSC_MDCTL_NEXT_ENABLE);
          UARTPinMuxSetup(2, FALSE);
     }
     #endif
     
     UARTStdioInitExpClk(BAUD_115200, UART_RX_TRIG_LEVEL_1);
     UARTInterruptInit();
     UART1InitSem();
}


void UARTConsolePutc(uint8_t data)
{
     UARTCharPut(UART_CONSOLE_BASE, data);
}


uint8_t UARTConsoleGetc(void)
{
    return ((uint8_t)UARTCharGet(UART_CONSOLE_BASE));
}

uint8_t UARTSemGetc(uint8_t * val,uint32_t timeout)
{
    if(FALSE == Semaphore_pend(rxSem,timeout))
        return FALSE;
    else
    {
        *val = rxData;
        return TRUE;
    }
}

/****************************************************************************/
/*                                                                          */
/*              硬件中断线程                                                */
/*                                                                          */
/****************************************************************************/
void UART1_ISR(UArg arg)
{
    unsigned int int_id = 0;

    // 确定中断源
    int_id = UARTIntStatus(SOC_UART_1_REGS);

    // 接收中断
    if(UART_INTID_RX_DATA == int_id)
    {
        rxData = UARTCharGetNonBlocking(SOC_UART_1_REGS);
        if(rxSem != NULL)
        {
            Semaphore_post(rxSem);
        }
    }

    // 接收错误
    if(UART_INTID_RX_LINE_STAT == int_id)
    {
        while(UARTRxErrorGet(SOC_UART_1_REGS))
        {
            // 从 RBR 读一个字节
            UARTCharGetNonBlocking(SOC_UART_1_REGS);
        }
    }
}


