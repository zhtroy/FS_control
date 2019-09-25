#include "common.h"
#include "mpu9250_drv.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>
#include "Moto/task_moto.h"
#include "Zigbee/Zigbee.h"
#include "system_param.h"

extern fbdata_t g_fbData;

systemParameter_t g_sysParam ={
        .carID = 0,
        .brakeDirection = 0,
        .brakeOffset = 100,
        .maxtThrottle = 100
};

parameter_t g_var = {
		.cycleRoute = 0,
};

void ParamSendBack()
{
	ZigbeeSend(&g_sysParam, ZIGBEE_PACKET_PARAM, sizeof(g_sysParam)	);
}

/*
 * 参数写回flash
 * 0失败
 * 1成功
 */
uint8_t ParamWriteToEEPROM()
{
	int result;
	result = mpu9250WriteBytesFreeLength(EEPROM_PHY_ADDR, 128, sizeof(systemParameter_t),  (uint8_t*)(&g_sysParam));
	return (result == -1) ?0:1;
}

static void ParamInitTask(UArg arg0, UArg arg1)
{
    mpu9250ReadBytes(EEPROM_PHY_ADDR,128, sizeof(systemParameter_t), (uint8_t*)(&g_sysParam));

    //上电时发回一次参数
    ParamSendBack();

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
