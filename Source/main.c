/*
 *  ======== main.c ========
 */

#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Cache.h>

#include "gpio_app.h"
#include "uartStdio.h"
#include "emifa/emifa_app.h"
#include "uartns550.h"

#include "Message/Message.h"
#include "Test_.h"
#include "canModule.h"

#include "DSP_Uart/dsp_uart2.h"
#include "fpga_ttl.h"
#include "common.h"
#include "TL6748.h"
#include "fpga_periph_def.h"
#include "mpu9250_drv.h"

void PeriphInit()
{
    /*
     * **********DSP外设初始化*****************
     * GPIO: 用户GPIO以及中断GPIO初始化 ;
     * IIC0: 用于9轴传感器(MPU9250)以及EEPROM通信;
     * UART1: 用作调试串口 (阻塞操作);
     * UART2: 用作4G通信  (阻塞操作);
     * EMIFA: 用于和FPGA之间通信;
     * ************************************
     */
	GPIOInit();

	I2CPinMuxSetup(0);

	UARTStdioInit();

	UART2StdioInit();

	mpu9250I2CInit();

	EMIFAInit();

	/*
	 * **********FPGA外设初始化***************
	 * 1. 等待FPGA加载成功(FPGA的DONE信号拉高);
	 * 2. 通过GPIO硬复位FPGA;
	 * 3. 判断EMIFA通信是否正常;
	 * 4. 初始化FPGA侧的CAN和串口中断配置;
	 *    a) 选中所有UART中断，开启使能;
	 *    b) 选中所有CAN中断，开启使能，并初始化CAN的
	 *       设备表;
	 * ************************************
	 */
	GPIOWaitFpgaDone();
	GPIOFpgaReset();

	EMIFAWriteWord(FPGA_TEST_REG,0,0xaa);
	if(0xaa != EMIFAReadWord(FPGA_TEST_REG,0))
	{
		UARTprintf("ERROR: EMIFA Test Failed!!!\r\n");
	}

    UartNs550HardIntMaskAll();
    UartNs550HardIntEnable ();

    CanHardIntMaskAll();
    CanHardIntEnable ();
    CanTableInit();
    TTLInit();

}

void ThreadInit()
{
	//task==========================
}

void SyncInit()
{
	//消息队列初始化
	Message_init();
}

/*
 *  ======== main ========
 */
Int main()
{

	/*
	 * **********初始化和任务启动***************
	 * 1. 外设初始化；
	 * 2. 同步模块（消息机制）初始化；
	 * 3. 线程初始化（暂未使用）；
	 * 4. Log任务初始化；
	 * 5. 测试任务入口；
	 * ************************************
	*/

	PeriphInit();

	SyncInit();

	ThreadInit();

	LogInit();

	TestEntry();

    BIOS_start();    /* does not return */
    return(0);
}
