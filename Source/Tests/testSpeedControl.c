#include "speed_Control.h"
#include "common.h"
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include "fpga_periph_def.h"

static int32_t vc = 0;
static int32_t ve = 0;
static int32_t te = 0;

void speedSet(int32_t slVc,int32_t slVe,int32_t slTe)
{
    vc = slVc;
    ve = slVe;
    te = slTe;
}

void taskTestSpeed(void)
{
    int32_t vg = 0,vgR = 0;
    uint16_t test = 0;
    uint16_t tmp = 0;
    while(1)
    {
        Task_sleep(100);
        test = EMIFAReadWord(FPGA_PCO_LOW,0);
        test += (EMIFAReadWord(FPGA_PCO_HIGH,0) << 8);

        if(test != tmp)
        {
            LogMsg("%d\r\n",test);
            tmp = test;
        }
#if 0
        vg = SpeedGenerate(vc, ve);
        if(vgR != vg)
        {
            LogMsg("%d\r\n",vg);
            vgR = vg;
        }
#endif
    }
}



void testSpeedControlTask(void)
{
    Task_Handle task;
    Error_Block eb;
    Task_Params taskParams;


    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.priority = 5;
    taskParams.stackSize = 2048;
    task = Task_create(taskTestSpeed, &taskParams, &eb);
    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

}
