#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include "hw_types.h"				 
#include "hw_syscfg0_C6748.h"	     
#include "soc_C6748.h"			     
#include "psc.h"			         
#include "timer.h"          
#include <stdint.h>

/* 看门狗定时器周期 */
// 定时时间 5秒
// 低32位
#define TMR_PERIOD_LSB32  (0x07270E00)
// 高32位
#define TMR_PERIOD_MSB32  (0x0)

void timerWatchDogInit(void);

static uint8_t enableWatchDog = 1;
/****************************************************************************/
/*                                                                          */
/*              定时器 / 计数器初始化                                       */
/*                                                                          */
/****************************************************************************/
void timerWatchDogInit(void)
{
    // 配置 定时器 / 计数器 1 为 看门狗模式
	TimerConfigure(SOC_TMR_1_REGS, TMR_CFG_64BIT_WATCHDOG);
	
    // 设置周期 64位
    TimerPeriodSet(SOC_TMR_1_REGS, TMR_TIMER12, TMR_PERIOD_LSB32);
    TimerPeriodSet(SOC_TMR_1_REGS, TMR_TIMER34, TMR_PERIOD_MSB32);

    // 使能看门狗定时器
    TimerWatchdogActivate(SOC_TMR_1_REGS);
    

}


static void timerWatchdogTask(void)
{
    timerWatchDogInit();
    while(1)
    {
        // 复位看门狗定时器 “喂狗”

#if 1
		if(enableWatchDog == 1)
			TimerWatchdogReactivate(SOC_TMR_1_REGS);
		else;
#endif

        Task_sleep(2000);

    }
}

void stopWatchdogReactivate()
{
    enableWatchDog = 0;
}

void testWatchDogTaskInit()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;

	Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
	task = Task_create((ti_sysbios_knl_Task_FuncPtr)timerWatchdogTask, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}
}

