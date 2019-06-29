#include "mpu9250_drv.h"
/*SYSBIOS includes*/
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu.h"
#include "soc_C6748.h"
#include "uart.h"
#include <mlmath.h>
#include "mpu9250_task.h"

/* 宏定义 */
#define DEFAULT_MPU_HZ  (10)
#define COMPASS_READ_MS (100)
#define MPU_Q30  (1073741824.0f)
#define PITCH_ERROR  (0)
#define ROLL_ERROR   (0)
#define YAW_ERROR    (0)
#define ACCEL_G		 (9.8)

/* 全局变量定义 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
static mpu9250DataCorrect_t compassDataCor = {
			.xMax = 1,
			.xMin = -1,
			.yMax = 1,
			.yMin = -1,
			.zMax = 1,
			.zMin = -1,
};

static uint8_t selfTestMode = 1;	/* 默认不进行自检 */
static uint8_t compassCorrect = 1;	/* 磁力计默认不校准 */
static uint8_t correctTrigger = 0;	/* 磁力计校准触发 */

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}


/*****************************************************************************
 * 函数名称: void MPU9250RunSelfTest(void)
 * 函数说明: 设备自检，通过则更新gyro和accel的偏移量。
 * 输入参数: 无
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/
static void MPU9250RunSelfTest(void)
{
   int result;
   long gyro[3], accel[3];
   float sens;
   unsigned short accel_sens;

   result = mpu_run_self_test(gyro, accel);

   if (result == 0x7)
   {
	   /* 测试通过，将gyro和accel的偏移量放入DMP中 */
       mpu_get_gyro_sens(&sens);
       gyro[0] = (long)(gyro[0] * sens);
       gyro[1] = (long)(gyro[1] * sens);
       gyro[2] = (long)(gyro[2] * sens);
       dmp_set_gyro_bias(gyro);
       mpu_get_accel_sens(&accel_sens);
       accel[0] *= accel_sens;
       accel[1] *= accel_sens;
       accel[2] *= accel_sens;
       dmp_set_accel_bias(accel);
       LogMsg("MPU9250 setting bias succesfully!!\r\n");
   }
	else
	{
		LogMsg("MPU9250 bias has not been modified ......\r\n");
	}
}

/*****************************************************************************
 * 函数名称: void MPU9250OpenDev(void)
 * 函数说明: 初始化MPU9250设备
 * 输入参数: 无
 * 输出参数: 无
 * 返 回 值: 0(成功)/负数(失败)
 * 备注:
*****************************************************************************/
static int32_t MPU9250OpenDev()
{
	struct int_param_s intParam;
	int32_t result = 0;
	uint8_t selfTestMode;

	/* 初始化I2C的相关配置 */
	//mpu9250I2CInit();

	/* 初始化MPU9250*/
	result = mpu_init(&intParam);
	if (result)
	{
		LogMsg("mpu_init Failed!!\r\n");
		return -1;
	}

	/* 唤醒传感器: 陀螺仪，加速度计和磁力计*/
	result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	if (result)
	{
		LogMsg("mpu_set_sensors Failed!!\r\n",0,0,0,0,0,0);
		return -1;
	}

	/* 陀螺仪(gyro)和加速度 (accel)数据都放到FIFO中 */
	result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	if (result)
	{
		LogMsg("mpu_configure_fifo Failed!!\r\n",0,0,0,0,0,0);
		return -1;
	}

	/* 设置陀螺仪和加速度计的采样速率，非DMP模式生效。(DMP模式下采样率固定为200Hz）*/
	result = mpu_set_sample_rate(DEFAULT_MPU_HZ);
	if (result)
	{
		LogMsg("mpu_set_sample_rate Failed!!\r\n");
		return -1;
	}

	/* 设置磁力计的采样速率 */
	result = mpu_set_compass_sample_rate(DEFAULT_MPU_HZ);
	if (result)
	{
		LogMsg("mpu_set_sample_rate Failed!!\r\n");
		return -1;
	}

	/* 设置加速度计的测量范围-2~2g */
	mpu_set_accel_fsr(2);

	/*
	mpu_get_compass_sample_rate(&compassRate);
	logMsg("MPU9250 Compass Sample Rate:%d\r\n",compassRate);
	*/

	/* 初始化DMP */
	result = dmp_load_motion_driver_firmware();
	if (result)
	{
		LogMsg("dmp_load_motion_driver_firmware Failed!!\r\n");
		return -1;
	}

	/* 设置DMP的gyro方向 */
	result = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
	if(result)
	{
		LogMsg("dmp_set_orientation Failed!!\r\n");
		return -1;
	}

	/* 设置DMP实现的功能 */
	result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT  | DMP_FEATURE_SEND_RAW_ACCEL |
					   DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
	if (result)
	{
		LogMsg("dmp_enable_feature Failed!!\r\n");
		return -1;
	}

	/* 设置DMP输出数据的频率 */
	result = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	if (result)
	{
		LogMsg("dmp_set_fifo_rate Failed!!\r\n");
		return -1;
	}

	/* 复位FIFO */
	result = mpu_reset_fifo();
	if (result)
	{
		LogMsg("mpu_reset_fifo Failed!!\r\n");
		return -1;
	}

	/* 自检:自检成功后，以当前位置作为参考原点 */
	MPU9250GetSelfTestMode(&selfTestMode);
	if(selfTestMode == 1)
	{
		MPU9250RunSelfTest();
	}

	/* 启动DMP */
	result = mpu_set_dmp_state(1);
	if (result)
	{
		LogMsg("mpu_reset_fifo Failed!!\r\n");
		return -1;
	}

	return 0;
}

/*****************************************************************************
 * 函数名称: void MPU9250CompassCorrectEnable(void)
 * 函数说明: 磁力计校准使能
 * 输入参数: 无
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/
void MPU9250CompassCorrectEnable(void)
{
	compassCorrect = 1;
	correctTrigger = 1;
}

/*****************************************************************************
 * 函数名称: void MPU9250CompassCorrectDisable(void)
 * 函数说明: 磁力计校准关闭
 * 输入参数: 无
 * 输出参数: 无
 * 返 回 值: 无
 * 备注:
*****************************************************************************/
void MPU9250CompassCorrectDisable(void)
{
	compassCorrect = 0;
}

/*****************************************************************************
 * 函数名称: static void MPU9250CompassCorrect(uint16_t *compass,mpu9250DataCorrect_t *dataCor)
 * 函数说明: 磁力计校准，获取各方向磁力的最大、最小值
 * 输入参数:
 * 		compass: 3轴磁力值指针；
 * 输出参数:
 * 		dataCor: 3轴磁力值的最大，最小值；
 * 返 回 值: 无
 * 备注:
*****************************************************************************/
static void MPU9250CompassCorrect(int16_t *compass,mpu9250DataCorrect_t *dataCor)
{
	if(correctTrigger)
	{
		dataCor->xMax = 1;
		dataCor->xMin = -1;
		dataCor->yMax = 1;
		dataCor->yMin = -1;
		dataCor->zMax = 1;
		dataCor->zMin = -1;
		correctTrigger = 0;
	}
	dataCor->xMax = MAX(compass[0],dataCor->xMax);
	dataCor->xMin = MIN(compass[0],dataCor->xMin);
	dataCor->yMax = MAX(compass[1],dataCor->yMax);
	dataCor->yMin = MIN(compass[1],dataCor->yMin);
	dataCor->zMax = MAX(compass[2],dataCor->zMax);
	dataCor->zMin = MIN(compass[2],dataCor->zMin);
}

/*****************************************************************************
 * 函数名称: static void MPU9250DataUpdate(mpu9250DataObj_t *dataObj)
 * 函数说明: 更新MPU9250数据。
 * 输入参数: 无
 * 输出参数:
 * 		dataObj:mpu9250数据对象，成员包括(姿态，加速度，磁力计和温度)
 * 返 回 值: 无
 * 备注:
*****************************************************************************/
static void MPU9250DataUpdate(mpu9250DataObj_t *dataObj)
{
	unsigned long sensor_timestamp;
	short gyro[3], accel_short[3], compass[3], sensors;
	unsigned char more,accelFsr;
	long quat[4], temperature;
	int32_t result;
	float q0,q1,q2,q3;
	int16_t compassX,compassY,compassZ;

	do
	{
		result = dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
	} while(more);

	if(0 == result)
	{
		/* 四元数转姿态计算 */
		if (sensors & INV_WXYZ_QUAT )
		{
			q0 = quat[0] / MPU_Q30;
			q1 = quat[1] / MPU_Q30;
			q2 = quat[2] / MPU_Q30;
			q3 = quat[3] / MPU_Q30;

			dataObj->pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 + PITCH_ERROR;
			dataObj->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 + ROLL_ERROR;
			dataObj->yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3 + YAW_ERROR;
		}

		/* 加速度归一化，单位g */
		mpu_get_accel_fsr(&accelFsr);
		dataObj->accelX = accel_short[0]/32768.0f;//*ACCEL_G*accelFsr;
		dataObj->accelY = accel_short[1]/32768.0f;//*ACCEL_G*accelFsr;
		dataObj->accelZ = accel_short[2]/32768.0f;//*ACCEL_G*accelFsr;

		dataObj->accelSpeed = sqrt(dataObj->accelX*dataObj->accelX +
									dataObj->accelY*dataObj->accelY +
									dataObj->accelZ*dataObj->accelZ);

		/* 温度计算
		 * TODO: 手册中的计算公式为21+xxx，而DMP中为35+xxx，需要在温度归一化之后-14.
		 * */
		mpu_get_temperature(&temperature, &sensor_timestamp);
		dataObj->temp = temperature/65536.0f - 14;

		/* 磁力计：电子罗盘方向计算 */

		mpu_get_compass_reg(compass, &sensor_timestamp);
		if(compassCorrect == 1)
		{
			MPU9250CompassCorrect(compass,&compassDataCor);
		}
		compassX = compass[0] - ((compassDataCor.xMax+compassDataCor.xMin)/2);
		compassY = (compassDataCor.xMax-compassDataCor.xMin)/(compassDataCor.yMax-compassDataCor.yMin)*compass[1] -
				((compassDataCor.yMax+compassDataCor.yMin)/2);
		compassZ = (compassDataCor.xMax-compassDataCor.xMin)/(compassDataCor.zMax-compassDataCor.zMin)*compass[1] -
						((compassDataCor.zMax+compassDataCor.zMin)/2);
		dataObj->direct =	atan2((double) (compassY),
						  (double) (compassX)
						 )*(180/M_PI)+180;

		dataObj->horizontal = compassZ;
	}
}

static void MPU9250Task(void)
{
	mpu9250DataObj_t dataObj;
	float test = 0;
	int16_t cnt = 0;

	int32_t result;
	/* 打开设备
	 * 若设备初始化失败，每隔1S尝试1次；
	 * */
	do{
		result = MPU9250OpenDev();
		if(result)
		{
			LogMsg("MPU9250 Initial Failed!!!\r\n");
			Task_sleep(1000);
		}
	}while(result);

	/* 主任务 */
    while(1)
    {
    	MPU9250DataUpdate(&dataObj);

    	LogMsg("加速度: %.3f,y-%.3f,z-%.3f\r\n%.3fm/s2\r\n",dataObj.accelX, dataObj.accelY, dataObj.accelZ,dataObj.accelSpeed);
    	LogMsg("俯仰: %3.3f,横滚: %3.3f,偏航: %3.3f\r\n",dataObj.pitch, dataObj.roll, dataObj.yaw);
    	LogMsg("温度: %.1f\r\n",dataObj.temp);
    	LogMsg("方向: %.2f,水平: %.2f\r\n",dataObj.direct, dataObj.horizontal);
        Task_sleep(1000/DEFAULT_MPU_HZ);
    } /* end while(1) */

}

void MPU9250GetSelfTestMode(uint8_t *mode)
{
	mode[0] = selfTestMode;
}

void MPU9250SetSelfTestMode(uint8_t mode)
{
	selfTestMode = mode;
}

void MPU9250TaskInit(void)
{
    Task_Handle task;
	Task_Params taskParams;
    Task_Params_init(&taskParams);
    
	taskParams.priority = 5;
	taskParams.stackSize = 4096;
    
	task = Task_create(MPU9250Task, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}
}

