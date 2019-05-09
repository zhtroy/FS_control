/*
 * test4GControl_hsm.c
 *
 *	将test4GControl.c 中的switch case 状态机用hsm.h的方式实现
 *
 *
 *  Created on: 2019-3-27
 *      Author: zhtro
 */

#include "Message/Message.h"
#include "hsm.h"
#include "CarHsm.h"
#include "hsmUtil.h"
#include "stdlib.h"
#include "utils/Timeout.h"
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Task.h>
#include "Sensor/PhotoElectric/PhotoElectric.h"
#include "Zigbee/Zigbee.h"
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Clock.h>
#include "task_moto.h"
#include "logLib.h"
#include "Sensor/RFID/EPCdef.h"
#include "Sensor/ZCP/v2v_communication.h"


extern Void taskRFID(UArg a0, UArg a1);
/*
 * 全局变量
 */
uint8_t g_connectStatus;

static xdc_Void connectionClosed(xdc_UArg arg)
{
    p_msg_t sendmsg;
    g_connectStatus = 0;

    /*
    *TODO:添加断连处理，发送急停消息，进入急停模式，并设置ErrorCode
    */
    sendmsg = Message_getEmpty();
    sendmsg->type = error;
    sendmsg->data[0] = ERROR_CONNECT_TIMEOUT;
    Message_post(sendmsg);
    //setErrorCode(ERROR_CONNECT_TIMEOUT);
}

static void connectionOn()
{
	g_connectStatus = 1;
}

static Void taskZigbeeControlMain_hsm(UArg a0, UArg a1)
{
	p_msg_t pMsg;
	evt_placeholder_t hsmEvt;
	car_hsm_t carHsm;


	 // 使用一个Clock来检测通信心跳包
	Clock_Params clockParams;
	Clock_Handle heartClock;
	Error_Block  eb;

	Error_init(&eb);

	Clock_Params_init(&clockParams);
	clockParams.period = 0;       // one shot
	clockParams.startFlag = FALSE;
	heartClock = Clock_create(connectionClosed, 1000*60*3, &clockParams, &eb); //3min 后没有收到包就停止
	if ( heartClock == NULL )
	{
		System_abort("Clock create failed\n");
	}

	/*
	 * 初始化Timeout模块
	 */
	TimeoutInit();

	/*
	 * 状态机初始化
	 */
	CarHsmCtor(&carHsm);
	HsmOnStart((Hsm*) &carHsm);

	while(1)
	{
		pMsg = Message_pend();

		/*
		 * TODO: 将输入数据记录在StatusData模块中
		 */

		/*
		 * 将pMsg翻译成HSM的事件 hsmEvt
		 */
		switch(pMsg->type)
		{
			case zigbee:
			{
				/*
				 * 检测心跳包
				 */
				//连接状态 on
				connectionOn();
				//清零clock
				Clock_start(heartClock);

				int data = atoi(&pMsg->data[1]);
				switch(pMsg->data[0])
				{
					case 'z': /*模式切换命令*/
					{
						evt_remote_chmode_t * p;

						EVT_SETTYPE(&hsmEvt, REMOTE_CHMODE_EVT);
						p = EVT_CAST(&hsmEvt, evt_remote_chmode_t);
						p->mode = pMsg->data[1] - '0';
						if(p->mode == 3)
						{
							p->errorcode = (pMsg->data[2]-'0')*10 + (pMsg->data[3]-'0');
						}
						break;
					}

					case 'm': /*选择前驱，后驱，双驱*/
					{
						EVT_SETTYPE(&hsmEvt, REMOTE_SELECT_MOTOR_EVT);
						EVT_CAST(&hsmEvt, evt_remote_sel_motor_t)->mode = data;
						break;
					}

					case 'g': /*选择挡位*/
					{
						EVT_SETTYPE(&hsmEvt, REMOTE_SELECT_GEAR_EVT);
						EVT_CAST(&hsmEvt, evt_remote_sel_gear_t)->gear = data;
						break;
					}

					case 't': /*设置油门*/
					{
						EVT_SETTYPE(&hsmEvt, REMOTE_SET_THROTTLE_EVT);
						EVT_CAST(&hsmEvt, evt_remote_set_throttle_t) ->throttle = data;
						break;
					}

					case 'r': /*开始变轨命令*/
					{
						EVT_SETTYPE(&hsmEvt, REMOTE_CH_RAIL_EVT);
						break;
					}

//					case 'R': /*设置当前轨道状态*/
//					{
//						EVT_SETTYPE(&hsmEvt, REMOTE_SET_RAILSTATE_EVT);
//						EVT_CAST(&hsmEvt, evt_remote_set_railstate_t)->state = data;
//						break;
//					}

					case 'b': /*设置刹车量*/
					{
						EVT_SETTYPE(&hsmEvt, REMOTE_SET_BRAKE_EVT);
						EVT_CAST(&hsmEvt, evt_remote_set_brake_t)->brake = data;
						break;
					}

					case 'k': /*设置pid参数*/
					{
						switch(pMsg->data[1])
						{
							case 'i':
							{
								EVT_SETTYPE(&hsmEvt, REMOTE_SET_KI_EVT);
								break;
							}
							case 'p':
							{
								EVT_SETTYPE(&hsmEvt, REMOTE_SET_KP_EVT);
								break;
							}
							case 'u':
							{
								EVT_SETTYPE(&hsmEvt, REMOTE_SET_KU_EVT);
								break;
							}
							case 'v':
							{
								EVT_SETTYPE(&hsmEvt, REMOTE_SET_KSP_EVT);
								break;
							}
							case 'w':
							{
								EVT_SETTYPE(&hsmEvt, REMOTE_SET_KSI_EVT);
								break;
							}
						}

						EVT_CAST(&hsmEvt, evt_remote_set_float_param_t)->value = \
							((pMsg->data[2]-'0')*1000000 + (pMsg->data[3]-'0')*100000\
							+(pMsg->data[4]-'0')*10000 +(pMsg->data[5]-'0')*1000+(pMsg->data[6]-'0')*100\
							+(pMsg->data[7]-'0')*10 + (pMsg->data[8]-'0')) / 1000000.0;


						break;
					}

					case 'c': /*设置是否允许变轨*/
					{
						EVT_SETTYPE(&hsmEvt, REMOTE_SET_ENABLE_CHANGERAIL_EVT);
						EVT_CAST(&hsmEvt, evt_remote_set_u8_param_t)->value =(pMsg->data[1]-'0');
						break;
					}

					case 's': /*设置转速*/
					{
						evt_remote_set_rpm_t * p;
						EVT_SETTYPE(&hsmEvt, REMOTE_SET_RPM_EVT);
						p = EVT_CAST(&hsmEvt, evt_remote_set_rpm_t);
						p->statecode = (pMsg->data[1]-'0')*10 + (pMsg->data[2]-'0');
						p->rpm = (pMsg->data[3]-'0')*1000 \
								+ (pMsg->data[4]-'0')*100 \
								+(pMsg->data[5]-'0')*10   \
								+ (pMsg->data[6]-'0');
						break;
					}

					case 'S': /*在automode下开始启动*/
					{
						EVT_SETTYPE(&hsmEvt, REMOTE_AUTO_START_EVT);
						break;
					}

					case 'A': /*设置后车zigbee地址*/
					{
						EVT_SETTYPE(&hsmEvt, REMOTE_SET_BACKCAR_ADDR);
						EVT_CAST(&hsmEvt, evt_remote_set_u16_param_t)->value = \
								(pMsg->data[5]-'0') \
								+(pMsg->data[4]-'0') * 10 \
								+(pMsg->data[3]-'0') * 100 \
								+(pMsg->data[2]-'0') * 1000 \
								+(pMsg->data[1]-'0') * 10000;
						break;
					}

					case 'h': /*心跳包*/
					{
						EVT_SETTYPE(&hsmEvt, REMOTE_HEARTBEAT_EVT);
						break;
					}

				}
				break;
			} /* case cell: */

			case rfid:
			{
				EVT_SETTYPE(&hsmEvt, RFID_EVT);
				EVT_CAST(&hsmEvt, evt_rfid_t)->epc = *((epc_t*) pMsg->data);
				break;
			}/* case rfid: */

			case timer:
			{
				EVT_SETTYPE(&hsmEvt, TIMER_EVT);
				EVT_CAST(&hsmEvt, evt_timeout_t)->type = pMsg->data[0];
				break;
			}/*	case timer:*/

			case photon:
			{
				EVT_SETTYPE(&hsmEvt, PHOTON_EVT);
				break;
			}

			case error:
			{
				EVT_SETTYPE(&hsmEvt, ERROR_EVT);
				EVT_CAST(&hsmEvt, evt_error_t)->code = pMsg->data[0];
				break;
			}

		} /*	switch(pMsg->type) */



		/*
		 * 输入event，驱动HSM运行
		 */
		HsmOnEvent((Hsm *) &carHsm, (Msg *) &hsmEvt);
//		LogMsg("hsm state: %s\n", (STATE_CURR(&carHsm))->name);

		/*
		 * 回收消息
		 */
		Message_recycle(pMsg);
	}
}

void testZigbeeControlHSM_init()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;


	Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = 6;      //比5高
	taskParams.stackSize = 2048;
	task = Task_create(taskZigbeeControlMain_hsm, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//4G
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(taskZigbee, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}


	//驱动电机  priority = 5
	MototaskInit();
	//刹车       priority = 5
	ServoTaskInit();

	//变轨电机

	//RFID

	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(taskRFID, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//对管
	Task_Params_init(&taskParams);
	taskParams.priority = 3;
	taskParams.stackSize = 2048;
	taskParams.arg0 = 0;
	task = Task_create(taskPhotoElectric, &taskParams, &eb);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

	//ZCP
	V2VZCPInit();
}
