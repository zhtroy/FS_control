#ifndef __TASK_BRAKE_SERVO_H
#define __TASK_BRAKE_SERVO_H
#include <stdint.h>
#include "Test_config.h"

#define UART_DEV_2 (2)
#define SERVOR_MOTOR_UART UART_DEV_2

#define MODBUS_ACK_OK           (0)
#define MODBUS_ACK_NOTOK        (1)
#define MODBUS_ACK_CRC_ERR      (2)
#define MODBUS_ACK_FRAME_ERR    (3)
#define MODBUS_ACK_TIMEOUT		(4)
#define MODBUS_ACK_LOOPERR      (5)
#define MODBUS_ACK_SENDERR      (6)

#define UNKNOWNRAIL_0 (0)
#define UNKNOWNRAIL_3 (3)
#define LEFTRAIL    (2)
#define RIGHTRAIL   (1)

#define SLEEPTIME 3

#define PI 3.141592654											//定义常量PI
#define DM 0.375												//轮子直径
#define KM 3.285714												//速比
#define DELTA 1.0												//减速度差阈值
#define AMAX 10.0												//最大减速度10m/s2
#define BRAKETIME (20)											//刹车计算时间
#define MAXSTEP 100												//刹车行程总步数

#define STEP_EXIT (5)

#define BRAKE_TIMEOUT (100)

#define CHANGERAIL_TIMEOUT (600)


//#define RAIL_ENABLE (0)
#define RAIL_ENABLE (0)
#define RAIL_DIRECT (1)

typedef struct{
    uint8_t     id;
    uint8_t     cmd;
    uint8_t     addrH;
    uint8_t     addrL;
    uint8_t     dataH;
    uint8_t     dataL;
    uint16_t    crc;
} modbusCmd_t;

/*
 * 刹车控制量
 */
typedef struct brake_ctrl_tag{
	uint8_t Brake;
    uint8_t BrakeReady;
}brake_ctrl_t;

/*
 * 变轨控制
 */
typedef struct rail_ctrl_tag{
    uint8_t RailState;
}rail_ctrl_t;

/*
 * Rail 变轨
 */
extern void RailStartChangeRoutine();
extern void RailChangeStart();

extern void RailSetRailState(uint8_t state);
extern uint8_t RailGetRailState();
extern uint8_t RailRailStateUnknown();

/*
 * Brake 刹车
 */
extern void BrakeSetBrake(uint8_t value);
extern uint8_t BrakeGetBrake();
extern void BrakeSetReady(uint8_t value);
extern uint8_t BrakeGetReady();


/*
 * 停站
 */
extern void StartStationStopRoutine();

/*
 * 临时停靠
 */
extern void StartTempStopRoutine();

#endif

