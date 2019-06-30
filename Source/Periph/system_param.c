#include "common.h"
#include "mpu9250_drv.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>
#include "Moto/task_moto.h"

extern fbdata_t g_fbData;

systemParameter_t sysParam ={
        .carID = 0,
        .brakeDirection = 0,
        .brakeOffset = 100,
        .maxtThrottle = 100
};

static void ParamInitTask(UArg arg0, UArg arg1)
{
    mpu9250ReadBytes(EEPROM_PHY_ADDR,128, sizeof(systemParameter_t), (uint8_t*)(&sysParam));

    /*
     * 回传
     */
    g_fbData.myID = sysParam.carID;

}

void ParamInit()
{
    Task_Handle task;
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.priority = 5;
    taskParams.stackSize = 1024;
    task = Task_create(ParamInitTask, &taskParams, NULL);
    if (task == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }
}
