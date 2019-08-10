/*
 * CarHsm.h
 *
 *  Created on: 2019-4-1
 *      Author: zhtro
 */

#ifndef CARHSM_H_
#define CARHSM_H_

#include "hsm.h"
#include "stdint.h"
#include "utils/Timeout.h"
#include "Sensor/RFID/EPCdef.h"
#include "Decision/route/Route.h"

typedef struct {
	Hsm super;
	State manual;
	State setting;
	State automode;
		State automode_idle;
		/*interjump的子状态根据RFID跳转*/
		State automode_running;
			State running_normal;
			State running_adjust;
			State running_seperate;


		State automode_changerail;
			State changerail_waitphoton;
			State changerail_changing;
			uint8_t prevrailstate;

		State automode_arrived;
		State automode_enterstation;
		State automode_stopstation;
		State autemode_stopstationleave;
		State automode_leavestation;
		State automode_premerge;
		State automode_merge;
			State merge_waitphoton;
			State merge_merging;
	State forcebrake;
}car_hsm_t;

enum CarEvents
{
	/*遥控事件 开始*/
	REMOTE_CHMODE_EVT,
	REMOTE_SELECT_MOTOR_EVT,
	REMOTE_SELECT_GEAR_EVT,
	REMOTE_SET_THROTTLE_EVT,
	REMOTE_CH_RAIL_EVT,
	REMOTE_SET_RAILSTATE_EVT,
	REMOTE_SET_BRAKE_EVT,
	REMOTE_SET_RPM_EVT,
	REMOTE_SET_KP_EVT,
	REMOTE_SET_KI_EVT,
	REMOTE_SET_KU_EVT,
	REMOTE_SET_KSP_EVT,
	REMOTE_SET_KSI_EVT,
	REMOTE_SET_KAP_EVT,
	REMOTE_SET_KAI_EVT,
	REMOTE_SET_ENABLE_CHANGERAIL_EVT,
	REMOTE_SET_STATION_ADDR,
	REMOTE_AUTO_START_EVT,
	REMOTE_HEARTBEAT_EVT,
	REMOTE_SET_ROUTENODE,
	REMOTE_NEW_ROUTE,

	/*RFID 开始*/
	RFID_EVT,

	/*timer 开始*/
	TIMER_EVT,

	/*光电对管开始*/
	PHOTON_EVT,

	/*错误事件*/
	ERROR_EVT,

	/*内部事件*/
	INTERNAL_EVT


};

/*
 * 以下是继承Msg的消息，加入需要的自定义数据，输入到HSM中
 *
 */

/*
 * 这个消息类型用于预先分配一块内存
 *
 * 其他的消息类型都不能大于这个类型
 */
typedef struct {
	Msg super;
	char mem[512];
}evt_placeholder_t;

/*
 *  遥控 (remote)
 *
 *  切换模式命令
 *
 */
typedef struct {
	Msg super;
	/*
	 * 模式跳转
	 * 0 手动模式
	 * 1 设置模式
	 * 2 自动模式
	 * 3 紧急制动模式
	 */
	uint8_t mode;
	uint8_t errorcode;
}evt_remote_chmode_t;

/*
 *  遥控 (remote)
 *
 *  电机选择
 *
 */
typedef struct {
	Msg super;
	/*
	 * 0 前轮
	 * 1 后轮
	 * 2 前后轮
	 */
	uint8_t mode;
}evt_remote_sel_motor_t;

/*
 * 遥控 (remote)
 *
 *	选择挡位
 */
typedef struct {
	Msg super;
	/*
	 * 0 空挡
	 * 1 前进
	 * 2 后退
	 * 3 慢速
	 */
	uint8_t gear;
}evt_remote_sel_gear_t;

/*
 * 遥控 (remote)
 *
 *	设置油门
 */
typedef struct {
	Msg super;
	uint8_t throttle;
}evt_remote_set_throttle_t;

/*
 * 遥控 (remote)
 *
 *	设置轨道状态
 */
typedef struct {
	Msg super;
	uint8_t state;
}evt_remote_set_railstate_t;

/*
 * 遥控 (remote)
 *
 *	设置刹车量
 */
typedef struct {
	Msg super;
	uint8_t brake;
}evt_remote_set_brake_t;

/*
 * 遥控 (remote)
 *
 *	设置RPM
 */
typedef struct {
	Msg super;
	uint16_t statecode;
	uint16_t rpm;
}evt_remote_set_rpm_t;

/*
 * 遥控 (remote)
 *
 *	设置float类型的参数
 */
typedef struct {
	Msg super;
	float value;
}evt_remote_set_float_param_t;

/*
 * 遥控 (remote)
 *
 *	设置u8类型的参数
 */
typedef struct {
	Msg super;
	uint8_t value;
}evt_remote_set_u8_param_t;

/*
 * 遥控 (remote)
 *
 *	设置u16类型的参数
 */
typedef struct {
	Msg super;
	uint16_t value;
}evt_remote_set_u16_param_t;

/*
 * rfid 读取到标签
 */
typedef struct{
	Msg super;
	/*
	 * 读到的EPC ID 暂时只用1个字节
	 */
	epc_t epc;
}evt_rfid_t;

/*
 * timer
 */
typedef struct{
	Msg super;
	timeout_type_t type;
}evt_timeout_t;

/*
 * error
 */
typedef struct{
	Msg super;
	uint8_t code;
}evt_error_t;


/*
 * internal
 */
typedef struct{
	Msg super;
	uint8_t eventcode;
}evt_internal_t;

/*
 * routenode
 */
typedef struct{
	Msg super;
	packet_routenode_t node;
}evt_routenode_t;

/*
 *
 */


/*
 * API
 */

extern void CarHsmCtor(car_hsm_t * me);
extern uint8_t CarHsmIsInStation();

#endif /* CARHSM_H_ */
