/*
 * HWI_4_Isr.c
 *
 *  HWI 4 中断处理函数
 *  处理GPIO bank 0中断
 *  FPGA的UART 和 CAN中断都在这里处理
 *
 *  Created on: 2018-12-3
 *      Author: zhtro
 */

#include "uartns550.h"
#include "interrupt.h"
#include "gpio.h"
#include "soc_C6748.h"
#include "canModule.h"
#include "signal.h"

#define GPIO_UART_INT (1)
#define GPIO_CAN_INT  (2)
#define GPIO_SIGNAL_INT (6)
void HWI_4_Isr(void)
{

    unsigned char IntStatus,DeviceIndex;
	//Disable UART 中断
	UartNs550HardIntDisable ();
    CanHardIntDisable();
    SignalSetIntEnable(0);

    // 关闭 GPIO BANK 0 中断
    GPIOBankIntDisable(SOC_GPIO_0_REGS, 0);

    // 清除 GPIO BANK 0 中断事件
    IntEventClear(SYS_INT_GPIO_B0INT);

    if(GPIOPinIntStatus(SOC_GPIO_0_REGS, GPIO_UART_INT) == GPIO_INT_PEND)   //uart
    {

		GPIOPinIntClear(SOC_GPIO_0_REGS, GPIO_UART_INT);
        IntStatus = UartNs550GetHardIntStatus();
        for(DeviceIndex = 0;DeviceIndex < 6;DeviceIndex++)
        {
            if(IntStatus & (1 << DeviceIndex))
                UartNs550IntrHandler(DeviceIndex);
        }

    }


    if(GPIOPinIntStatus(SOC_GPIO_0_REGS, GPIO_CAN_INT) == GPIO_INT_PEND)
    {
		
		GPIOPinIntClear(SOC_GPIO_0_REGS, GPIO_CAN_INT);
        IntStatus = CanGetHardIntStatus();
        for(DeviceIndex = 0;DeviceIndex < 8;DeviceIndex++)
        {
            if(IntStatus & (1 << DeviceIndex))
                CanIsr(DeviceIndex);
        }
        
    }

    if(GPIOPinIntStatus(SOC_GPIO_0_REGS, GPIO_SIGNAL_INT) == GPIO_INT_PEND)
    {

        GPIOPinIntClear(SOC_GPIO_0_REGS, GPIO_SIGNAL_INT);
        IntStatus = SignalGetHardIntStatus();
        SignalCallBack(IntStatus);

    }
	// 使能 GPIO BANK 0 中断 */
    GPIOBankIntEnable(SOC_GPIO_0_REGS, 0);
    UartNs550HardIntEnable();
    CanHardIntEnable();
    SignalSetIntEnable(1);
}
