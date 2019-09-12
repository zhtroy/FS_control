/*
 * signal.c
 *
 *  Created on: 2019-7-18
 *      Author: DELL
 */

#include "fpga_periph_def.h"
#include "emifa_app.h"
#include "logLib.h"


uint8_t SignalGetHardIntStatus()
{
    return EMIFAReadWord(FPGA_SIGNAL_INT,0);
}

void SignalCallBack(uint8_t intStatus)
{
    LogMsg("Signal Check %x",intStatus);
}
void SignalSetIntEnable(uint8_t value)
{
    EMIFAWriteWord(FPGA_SIGNAL_INT_ENB,0,value);
}
void SignalInit()
{
    /*
     * 均设置为上升沿中断
     */
    EMIFAWriteWord(FPGA_SIGNAL_TYPE,0,0x55);

    /*
     * 滤波均设置为2ms
     */
    EMIFAWriteWord(FPGA_SIGNAL0_FILTER,0,2);
    EMIFAWriteWord(FPGA_SIGNAL1_FILTER,0,2);
    EMIFAWriteWord(FPGA_SIGNAL2_FILTER,0,2);
    EMIFAWriteWord(FPGA_SIGNAL3_FILTER,0,2);
    /*
     * 清空中断状态
     */
    SignalGetHardIntStatus();

    /*
     * 打开中断使能
     */
    SignalSetIntEnable(1);
}


