#ifndef __TASK_CAN_H
#define __TASK_CAN_H

#include <stdint.h>
#include "Test_config.h"

//MOTO F CAN ID
#define MOTO_F_CANID1	0x10F8E3F3
#define MOTO_F_CANID2	0x10F8139A
#define MOTO_F_CANID3	0x10F8138D
#define MOTO_F_CANID4	0x10F8137B
//MOTO R CAN ID
#define MOTO_R_CANID1	0x10F8E4F3
#define MOTO_R_CANID2	0x10F8149A
#define MOTO_R_CANID3	0x10F8148D
#define MOTO_R_CANID4	0x10F8147B

#define CAN_DEV_0       (0)
#define MOTO_CAN_DEVNUM CAN_DEV_0

//Moto CAN控制数据结构
typedef struct _CAN_DATA
{
	uint8_t Gear;
	uint8_t ThrottleL;
	uint8_t ThrottleH;
	uint8_t Mode;
	uint8_t TorqueL;
	uint8_t TorqueH;
	uint8_t SpeedOrBreakL;
	uint8_t SpeedOrBreakH;
}canData_t;



enum motoSel
{
	FRONT_ONLY,
	REAR_ONLY,
	FRONT_REAR,
};
enum motoID
{
	MOTO_FRONT,
	MOTO_REAR,
};
enum motoMode
{
	MODE_THROTTLE,
	MODE_TORQUE,
	MODE_SPEED,
};
enum motoGear
{
	GEAR_NONE,
	GEAR_DRIVE,
	GEAR_REVERSE,
	GEAR_LOW,
};

/*
 * 电机控制量
 */
#pragma pack(1)
typedef struct moto_ctrl_t_Tag
{
	enum motoSel MotoSel;
	enum motoMode ControlMode;
	enum motoGear Gear;
	uint8_t Throttle;
    uint16_t GoalRPM;
    uint8_t PidOn;
}moto_ctrl_t;

/*通讯反馈数据结构*/
typedef struct
{
	uint8_t CarStatus;
	uint8_t MotoSel;
	uint8_t MotoId;
	uint8_t BreakMode;
	uint8_t Break;
	uint8_t Reserve1;
	uint8_t Reserve2;
	uint8_t Reserve3;
	//CAN ID=10F81X9A
	uint8_t Gear;
	uint8_t ThrottleL;
	uint8_t ThrottleH;
	uint8_t MotoMode;
	uint8_t RPML;
	uint8_t RPMH;
	uint8_t MotoTemp;
	uint8_t DriverTemp;
	//CAN ID=10F81X8D
	uint8_t VoltL;
	uint8_t VoltH;
	uint8_t CurrentL;
	uint8_t CurrentH;
	uint8_t DistanceL;
	uint8_t DistanceH;
	uint8_t ErrCodeL;
	uint8_t ErrCodeH;
	//CAN ID=10F81X7B
	uint8_t TorqueCtrlL;
	uint8_t TorqueCtrlH;
	uint8_t RPMCtrlL;
	uint8_t RPMCtrlH;
	uint8_t TorqueL;
	uint8_t TorqueH;
	uint8_t CanReserved1;
	uint8_t CanReserved2;
}motorData_t;

#pragma pack(1)
typedef struct{
	motorData_t motorDataF;
	motorData_t motorDataR;
	uint32_t rfid;
	uint8_t mode;
	uint8_t brake;
	uint8_t railstate;
	uint8_t FSMstate;
    uint8_t ErrorCode;  //急停态错误状态
    uint32_t circleNum;
	uint32_t rfidReadTime;
	uint32_t calcRPM;
 	uint32_t recvRPM;
 	uint32_t forwardCarDistance;
 	uint32_t forwardCarRPM;

}fbdata_t;

/*无错误*/
#define ERROR_NONE                  (0) 

/*超出安全轨道范围*/
#define ERROR_OUT_SAFE_TRACK        (1) 

/*网络连接超时，无心跳包*/
#define ERROR_CONNECT_TIMEOUT       (2) 

/*分轨失败*/
#define ERROR_SEPERATE_FAILED       (3)

/*主动急停*/
#define ERROR_MANUAL_STOP           (4)

/*超时未检测到进站ID*/
#define ERROR_WAIT_ENTER_STATION    (5)

/*超时未检测到停站ID*/
#define ERROR_WAIT_STOP_STATION     (6)

/*超时未检测到出站ID*/
#define ERROR_WAIT_LEAVE_STATION    (7)

/*超时未检测到预并轨ID*/
#define ERROR_WAIT_PRE_MERGE        (8)

/*超时未检测到并轨ID*/
#define ERROR_WAIT_MERGE            (9)

/*超时未检测到并轨光电对管*/
#define ERROR_WAIT_MERGE_PHOTON     (10)

/*并轨超时*/
#define ERROR_MERGE_FAILED          (11)

/*刹车通信异常*/
#define ERROR_BRAKE_TIMEOUT         (12)

/*变轨通信异常*/
#define ERROR_CHANGERAIL_TIMEOUT    (13)

/*RFID通信异常*/
#define ERROR_RFID_TIMEOUT    (14)

/*MOTO后轮通信异常*/
#define ERROR_MOTOR_TIMEOUT    (15)

/*MOTO后轮通信异常*/
#define ERROR_MOTOF_TIMEOUT    (16)

/*刹车控制器异常*/
#define ERROR_BRAKE_ERROR	(17)

/*前后轮转速异常*/
#define ERROR_RPM_ABNORMAL   (17)

#define DIFF_RPM_UPSCALE (4000)
#define DIFF_RPM_DWSCALE (-4000)

#define ADJ_THROTTLE_UPSCALE (20)
#define ADJ_THROTTLE_DWSCALE (-20)

#if CAR_VERSION == 20
//#define MAX_THROTTLE_SIZE (55)
#define MAX_THROTTLE_SIZE (110)
#elif CAR_VERSION == 21
#define MAX_THROTTLE_SIZE (100)

#elif CAR_VERSION == 22
#define MAX_THROTTLE_SIZE (180)

#endif

#define MIN_THROTTLE_SIZE (-355)

/*
 * 前后轮最大转速差
 */
#define MAX_DIFF_RPM (400)

/*
 * 刹车偏移量，由刹车部件导致，一部分行程可能无刹车效果
 */
#define BREAK_OFFSET (0)

/*
 * 安全距离范围定义,单位(10cm)
 */
#define MIN_SAFE_DISTANCE (200)
#define MAX_SAFE_DISTANCE (400)
/*
 * 远离距离范围定义,单位(10cm)
 */
#define MIN_AWAY_DISTANCE (50)
#define MAX_AWAY_DISTANCE (200)

#if CAR_VERSION == 20
#define MAX_BRAKE_SIZE (200)
#elif CAR_VERSION == 21
#define MAX_BRAKE_SIZE (250)
#elif CAR_VERSION == 22
#define MAX_BRAKE_SIZE (255)
#endif


#define FILTER_RPM (200)
#define DELTA_RPM (12)
#define RPM_LIMIT (2400)

#define BRAKE_THRO_RATIO (1)

uint16_t MotoGetRealRPM(void);
uint8_t MotoSetErrorCode(uint8_t code);
extern void MotoSetMotoSel(enum motoSel sel);
extern enum motoSel MotoGetMotoSel();
extern void MotoSetControlMode(enum motoMode mode);
extern enum motoMode MotoGetControlMode();
extern void MotoSetGear(enum motoGear gear);
extern enum motoGear MotoGetGear();
extern void MotoSetThrottle(uint8_t thr);
extern uint8_t MotoGetThrottle();
extern void MotoSetGoalRPM(uint16_t rpm);
extern uint16_t MotoGetGoalRPM();
extern void MotoSetPidOn(uint8_t mode);
extern uint8_t MotoGetPidOn();
uint16_t MotoGetCircles();
uint16_t MotoGetRpm();
uint8_t MotoGetCarMode();
#endif
