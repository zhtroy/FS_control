/*
 * CarHsm.c
 *
 * HSM定义，顶层状态
 *
 * 单个状态对应:
 * 		* car_hsm_t结构体中的的一个域
 * 		  命名方式为  父状态_状态 如automode_straight
 * 		* 一个状态处理函数
 * 		   状态处理函数命名方式为  AutomodeStraight
 *
 *  Created on: 2019-3-28
 *      Author: zhtro
 */

#include "CarHsm.h"
#include "hsmUtil.h"
#include "hsm.h"
#include "CarState.h"
#include "utils/Timeout.h"
#include "task_moto.h"
#include "task_brake_servo.h"
#include "Moto/Parameter.h"
#include "RFID/EPCdef.h"
#include "Decision/route/Route.h"



extern fbdata_t g_fbData;
/*
 * 状态机
 */
Msg const *Top(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case START_EVT:
		{
			STATE_START(me, &me->manual);
			/*
			 * 在状态处理函数中返回0表示消息已经被处理，否则返回msg
			 */
			return 0;
		}
		case ERROR_EVT:
		{
			MotoSetErrorCode(EVT_CAST(msg, evt_error_t)->code);
			STATE_TRAN(me, &me->forcebrake);
			return 0;
		}
		case REMOTE_CHMODE_EVT:
		{
			evt_remote_chmode_t * pEvt = EVT_CAST(msg, evt_remote_chmode_t);

			/*初始化motor数据*/
			MotoSetMotoSel(FRONT_REAR);  //两个电机都用
			MotoSetControlMode(MODE_THROTTLE);  //Throttle模式
			MotoSetGear(GEAR_NONE);    //空挡
			MotoSetThrottle(0);
			BrakeSetBrake(0);
			MotoSetGoalRPM(0);
			/*回传数据*/
			g_fbData.mode = pEvt->mode;

			switch(pEvt->mode)
			{
				case 0:
					STATE_TRAN(me, &me->manual);
					break;
				case 1:
					STATE_TRAN(me, &me->setting);
					break;
				case 2:
					STATE_TRAN(me, &me->automode);
					break;
				case 3:
					MotoSetErrorCode(pEvt->errorcode);
					STATE_TRAN(me, &me->forcebrake);
					break;
				default:
					break;
			}
			return 0;
		} /*case REMOTE_CHMODE_EVT:*/

        case RFID_EVT:
        {
            evt_rfid_t* p = EVT_CAST(msg, evt_rfid_t);

            if(EPC_FEAT_AUXILIARY_TRACK_START == p->epc.roadFeature)
            {
                if(RIGHTRAIL == RailGetRailState() && (GEAR_REVERSE == MotoGetGear()))
                {
                    MotoSetErrorCode(ERROR_OUT_SAFE_TRACK);
                    STATE_TRAN(me, &me->forcebrake);
                    return 0;
                }
            }
            else if(EPC_FEAT_AUXILIARY_TRACK_END == p->epc.roadFeature)
            {
                if(RIGHTRAIL == RailGetRailState() && (GEAR_DRIVE == MotoGetGear() || GEAR_LOW == MotoGetGear()))
                {
                    MotoSetErrorCode(ERROR_OUT_SAFE_TRACK);
                    STATE_TRAN(me, &me->forcebrake);
                    return 0;
                }
            }

            break;

        }/* case RFID_EVT: */




	}

	/*如果未处理该消息，返回msg*/
	return msg;
}



Msg const * TopManual(car_hsm_t * me, Msg* msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			//reset ctrl data
			return 0;
		}

		case REMOTE_SELECT_MOTOR_EVT:
		{
			MotoSetMotoSel( EVT_CAST(msg, evt_remote_sel_motor_t)->mode );
			return 0;
		}

		case REMOTE_SELECT_GEAR_EVT:
		{
			MotoSetGear(EVT_CAST(msg, evt_remote_sel_gear_t)->gear);
			return 0;
		}

		case REMOTE_SET_THROTTLE_EVT:
		{
			MotoSetThrottle( EVT_CAST(msg, evt_remote_set_throttle_t)->throttle);
			return 0;
		}

		case REMOTE_CH_RAIL_EVT:
		{
			RailChangeStart();
			return 0;
		}

		case REMOTE_SET_RAILSTATE_EVT:
		{
			RailSetRailState(EVT_CAST(msg, evt_remote_set_railstate_t)->state);
			return 0;
		}

		case REMOTE_SET_BRAKE_EVT:
		{
			BrakeSetBrake(EVT_CAST(msg, evt_remote_set_brake_t)->brake);
			return 0;
		}


	}


	return msg;
}

/*
 * 设置模式
 */
Msg const * TopSetting(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case REMOTE_SET_KI_EVT:
		{
			g_param.KI = EVT_CAST(msg, evt_remote_set_float_param_t)->value;
			return 0;
		}

		case REMOTE_SET_KP_EVT:
		{
			g_param.KP = EVT_CAST(msg, evt_remote_set_float_param_t)->value;
			return 0;
		}

		case REMOTE_SET_KU_EVT:
		{
			g_param.KU = EVT_CAST(msg, evt_remote_set_float_param_t)->value;
			return 0;
		}

		case REMOTE_SET_KSP_EVT:
		{
			g_param.KSP = EVT_CAST(msg, evt_remote_set_float_param_t)->value;
			return 0;
		}

		case REMOTE_SET_KSI_EVT:
		{
			g_param.KSI = EVT_CAST(msg, evt_remote_set_float_param_t)->value;
			return 0;
		}

		case REMOTE_SET_BACKCAR_ADDR:
		{
			g_param.REAR_CAR_ADDR = EVT_CAST(msg, evt_remote_set_u16_param_t)->value;
			return 0;
		}

		case REMOTE_SET_ENABLE_CHANGERAIL_EVT:
		{
			g_param.EnableChangeRail = EVT_CAST(msg, evt_remote_set_u8_param_t) ->value;
			return 0;
		}

		case REMOTE_SET_RPM_EVT:
		{
			evt_remote_set_rpm_t * p = EVT_CAST(msg, evt_remote_set_rpm_t);
			if(p->statecode < car_state_None)  //防止访问越界
			{
				g_param.StateRPM[ p->statecode] = p->rpm;
			}
			return 0;
		}

		case REMOTE_SET_ROUTENODE:
		{
			evt_routenode_t * p = EVT_CAST(msg, evt_routenode_t);
			packet_routenode_t node = p->node;
			RouteAddNode(node);
			return  0;
		}

		case REMOTE_NEW_ROUTE:
		{
			//如果是新设置路径，将原来的路径删除

			RouteFree();
			return 0;
		}
	}

	return msg;
}

/*
 * 自动模式
 */
Msg const * TopAutoMode(car_hsm_t * me, Msg* msg)
{
	switch(msg->evt)
	{
		case START_EVT: /*起始状态*/
		{
			STATE_START(me, &me->automode_idle);
			return 0;
		}
		case ENTRY_EVT:
		{
			MotoSetPidOn(1);
			MotoSetGear(GEAR_DRIVE);    /*前进档*/
			return 0;
		}
		case EXIT_EVT:
		{
			g_fbData.FSMstate = idle;
			MotoSetPidOn(0);
			return 0;
		}
	}

	return msg;
}

/*
 * 自动模式 - idle
 */
Msg const * AutoModeIdle(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			g_fbData.FSMstate =idle;
			return 0;
		}
		case EXIT_EVT:
		{
			return 0;
		}
		case REMOTE_AUTO_START_EVT:
		{
			STATE_TRAN(me, &me->cruising);
			return 0;
		}
	}

	return msg;
}

/*
 * 自动模式 - automode_interjump
 */
Msg const * AutoModeInterJump(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case START_EVT:
		{
			STATE_START(me, me->automode_interjump_hist);
			return 0;
		}
		case EXIT_EVT:
		{
			me->automode_interjump_hist = STATE_CURR(me);
			return 0;
		}
		case RFID_EVT:
		{
			/*
			 * 先进行路径EPC判断，如果是路径上的点,进行处理
			 */
			if(RouteHasOngoing())
			{
				evt_rfid_t * pEvt = EVT_CAST(msg, evt_rfid_t);
				packet_routenode_t curNode = RoutePeek();

				if(EPCgetShortID(&(pEvt->epc)) == curNode.nid)
				{

					RoutePop();    //如果路径点匹配，弹出当前节点

					if( RouteGetNodeNT(curNode) == ROUTE_NT_STOP ) //终点
					{
						STATE_TRAN(me, &me->automode_arrived);
						return 0;
					}
					else   //进入变轨流程
					{
						STATE_TRAN(me, &me->automode_changerail);
						return 0;
					}
				}
			}

			/*
			 * 如果不是路径上的点，再根据rfid的路段特性来调整速度
			 */
			switch(EVT_CAST(msg, evt_rfid_t)->epc.roadFeature)
			{
				case EPC_FEAT_HORIZONTAL_STRAIGHT:
				{
					STATE_TRAN(me, &me->straight);
					return 0;
				}

				case EPC_FEAT_HORIZONTAL_CURVE:
				{
					STATE_TRAN(me, &me->curving);
					return 0;
				}

				case EPC_FEAT_UPHILL_GRADUAL:
				{
					STATE_TRAN(me, &me->uphill);
					return 0;
				}


				case EPC_FEAT_DOWNHILL_GRADUAL:
				{
					STATE_TRAN(me, &me->downhill);
					return 0;
				}

			}
		}/*case RFID_EVT:*/
	}

	return msg;
}

/*
 * 自动模式-巡航
 */
Msg const * InterJumpCruising(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			g_fbData.FSMstate =cruising;

			MotoSetGoalRPM(g_param.StateRPM[cruising]);
			return 0;
		}
		case EXIT_EVT:
		{
			return 0;
		}
	}

	return msg;
}

/*
 * 自动模式- 直行
 */
Msg const * InterJumpStraight(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			g_fbData.FSMstate =straight;

			MotoSetGoalRPM(g_param.StateRPM[straight]);
			return 0;
		}
		case EXIT_EVT:
		{
			return 0;
		}
	}

	return msg;
}

/*
 * 自动模式- 预转弯
 */
Msg const * InterJumpPreCurve(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			g_fbData.FSMstate =pre_curve;

			MotoSetGoalRPM(g_param.StateRPM[pre_curve]);
			return 0;
		}
		case EXIT_EVT:
		{
			return 0;
		}
	}

	return msg;
}

/*
 * 自动模式- 转弯
 */
Msg const * InterJumpCurving(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			g_fbData.FSMstate =curving;

			MotoSetGoalRPM(g_param.StateRPM[curving]);
			return 0;
		}
		case EXIT_EVT:
		{
			return 0;
		}
	}

	return msg;
}

/*
 * 自动模式- 上坡
 */
Msg const * InterJumpUphill(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			g_fbData.FSMstate =uphill;

			MotoSetGoalRPM(g_param.StateRPM[uphill]);
			return 0;
		}
		case EXIT_EVT:
		{
			return 0;
		}
	}

	return msg;
}

/*
 * 自动模式- 下坡
 */
Msg const * InterJumpDownhill(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			g_fbData.FSMstate =downhill;

			MotoSetGoalRPM(g_param.StateRPM[downhill]);
			return 0;
		}
		case EXIT_EVT:
		{
			return 0;
		}
	}

	return msg;
}

/*
 * 自动模式- 预下坡
 */
Msg const * InterJumpPreDownhill(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			g_fbData.FSMstate =pre_downhill;

			MotoSetGoalRPM(g_param.StateRPM[pre_downhill]);
			return 0;
		}
		case EXIT_EVT:
		{
			return 0;
		}
	}

	return msg;
}

/*
 * 自动模式- 预分轨
 */
//Msg const * InterJumpPreSeperate(car_hsm_t * me, Msg * msg)
//{
//	switch(msg->evt)
//	{
//		case ENTRY_EVT:
//		{
//			g_fbData.FSMstate =pre_seperate;
//
//			MotoSetGoalRPM(g_param.StateRPM[pre_seperate]);
//			return 0;
//		}
//		case EXIT_EVT:
//		{
//			return 0;
//		}
//
//		case RFID_EVT:
//		{
//			if(EVT_CAST(msg, evt_rfid_t)->epc == EPC_SEPERATE)
//			{
//				STATE_TRAN(me, &me->automode_seperate);
//				return 0;
//			}
//		}
//	}
//
//	return msg;
//}

/*
 * 自动模式- 分轨
 */

Msg const * AutomodeChangeRail(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case START_EVT:
		{
			STATE_START(me, &me->changerail_waitphoton);
			return 0;
		}
		case ENTRY_EVT:
		{
			g_fbData.FSMstate = seperate;

			return 0;
		}
		case EXIT_EVT:
		{
			return 0;
		}
	}

	return msg;
}

/*
 * 分轨 - 等待光电对管
 */

Msg const * ChangeRailWaitPhoton(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			TimeoutSet(changerail_wait_photon);
			return 0;
		}

		case TIMER_EVT:
		{
			if(changerail_wait_photon == EVT_CAST(msg, evt_timeout_t)->type )
			{
				//没等到对管，不变轨
				MotoSetErrorCode(ERROR_SEPERATE_FAILED);
				STATE_TRAN(me, &me->forcebrake);
				return 0;
			}
			break;
		}
		case PHOTON_EVT:
		{
			me->prevrailstate = RailGetRailState();
			STATE_TRAN(me, &me->changerail_changing);
			return 0;
		}
	}

	return msg;
}


Msg const * ChangeRailChanging(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			TimeoutSet(changerail_wait_changerail);
			RailChangeStart();
			return 0;
		}

		case TIMER_EVT:
		{
			if(changerail_wait_changerail == EVT_CAST(msg, evt_timeout_t)->type)
			{
				if(me->prevrailstate != RailGetRailState())
				{
					STATE_TRAN(me, &me->automode_interjump);
					return 0;
				}
				else{
					MotoSetErrorCode(ERROR_SEPERATE_FAILED);
					STATE_TRAN(me, &me->forcebrake);
					return 0;
				}
			}
		}
	}

	return msg;
}

Msg const * AutoModeArrived(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case START_EVT:
		{
			return 0;
		}
		case ENTRY_EVT:
		{
			return 0;
		}
		case EXIT_EVT:
		{
			return 0;
		}
	}

	return msg;
}

//Msg const * AutomodeEnterStation(car_hsm_t * me, Msg * msg)
//{
//	switch(msg->evt)
//	{
//		case ENTRY_EVT:
//		{
//			g_fbData.FSMstate =enter_station;
//
//			MotoSetGoalRPM(g_param.StateRPM[enter_station]);
//			TimeoutSet(seperate_wait_stop_station);
//			return 0;
//		}
//		case TIMER_EVT:
//		{
//			if(seperate_wait_stop_station == EVT_CAST(msg, evt_timeout_t)->type)
//			{
//				MotoSetErrorCode(ERROR_WAIT_STOP_STATION);
//				STATE_TRAN(me, &me->forcebrake);
//				return 0;
//			}
//			break;
//		}
//		case RFID_EVT:
//		{
//			if(EPC_STOP_STATION == EVT_CAST(msg, evt_rfid_t)->epc)
//			{
//				STATE_TRAN(me, &me->automode_stopstation);
//				return 0;
//			}
//		}
//	}
//
//	return msg;
//}

Msg const * AutomodeStopStation(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			g_fbData.FSMstate =stop_station;

			/*进入开始离站状态*/
			STATE_TRAN(me, &me->autemode_stopstationleave);
			return 0;
		}
	}

	return msg;
}

//Msg const * AutomodeStopStationLeave(car_hsm_t * me, Msg * msg)
//{
//	switch(msg->evt)
//	{
//		case ENTRY_EVT:
//		{
//			MotoSetGoalRPM(g_param.StateRPM[stop_station]);
//			TimeoutSet(seperate_wait_leave_station);
//			return 0;
//		}
//		case TIMER_EVT:
//		{
//			if(seperate_wait_leave_station == EVT_CAST(msg, evt_timeout_t)->type)
//			{
//				MotoSetErrorCode(ERROR_WAIT_LEAVE_STATION);
//				STATE_TRAN(me, &me->forcebrake);
//				return 0;
//			}
//			break;
//		}
//
//		case RFID_EVT:
//		{
//			if(EPC_LEAVE_STATION == EVT_CAST(msg, evt_rfid_t)->epc)
//			{
//				STATE_TRAN(me, &me->automode_leavestation);
//				return 0;
//			}
//			break;
//		}
//
//	}
//
//	return msg;
//}

//Msg const * AutomodeLeaveStation(car_hsm_t * me, Msg * msg)
//{
//	switch(msg->evt)
//	{
//		case ENTRY_EVT:
//		{
//			g_fbData.FSMstate = leave_station;
//
//			MotoSetGoalRPM(g_param.StateRPM[leave_station]);
//			TimeoutSet(seperate_wait_pre_merge);
//			return 0;
//		}
//		case TIMER_EVT:
//		{
//			if(seperate_wait_pre_merge == EVT_CAST(msg, evt_timeout_t)->type)
//			{
//				MotoSetErrorCode(ERROR_WAIT_PRE_MERGE);
//				STATE_TRAN(me, &me->forcebrake);
//				return 0;
//			}
//			break;
//		}
//		case RFID_EVT:
//		{
//			if(EPC_PRE_MERGE == EVT_CAST(msg, evt_rfid_t)->epc)
//			{
//				STATE_TRAN(me, &me->automode_premerge);
//				return 0;
//			}
//			break;
//		}
//	}
//
//	return msg;
//}

//Msg const * AutomodePreMerge(car_hsm_t * me, Msg * msg)
//{
//	switch(msg->evt)
//	{
//		case ENTRY_EVT:
//		{
//			g_fbData.FSMstate = pre_merge;
//
//			MotoSetGoalRPM(g_param.StateRPM[pre_merge]);
//			TimeoutSet(seperate_wait_merge);
//			return 0;
//		}
//		case TIMER_EVT:
//		{
//			if(seperate_wait_merge == EVT_CAST(msg, evt_timeout_t)->type)
//			{
//				MotoSetErrorCode(ERROR_WAIT_MERGE);
//				STATE_TRAN(me, &me->forcebrake);
//				return 0;
//			}
//			break;
//		}
//		case RFID_EVT:
//		{
//			if(EPC_MERGE == EVT_CAST(msg, evt_rfid_t)->epc)
//			{
//				STATE_TRAN(me, &me->automode_merge);
//				return 0;
//			}
//			break;
//		}
//	}
//
//	return msg;
//}

Msg const * AutomodeMerge(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case START_EVT:
		{
			STATE_START(me, &me->merge_waitphoton);
			return 0;
		}
		case ENTRY_EVT:
		{
			g_fbData.FSMstate =merge;

			MotoSetGoalRPM(g_param.StateRPM[merge]);
			return 0;
		}
	}

	return msg;
}

Msg const * MergeWaitPhoton(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			TimeoutSet(merge_wait_photon);
			return 0;
		}
		case TIMER_EVT:
		{
			if(merge_wait_photon == EVT_CAST(msg, evt_timeout_t)->type)
			{
				MotoSetErrorCode(ERROR_WAIT_MERGE_PHOTON);
				STATE_TRAN(me, &me->forcebrake);
				return 0;
			}
			break;
		}
		case PHOTON_EVT:
		{

			STATE_TRAN(me, &me->merge_merging);
			return 0;
		}
	}

	return msg;
}

Msg const * MergeMerging(car_hsm_t * me, Msg * msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			RailChangeStart();
			TimeoutSet(merge_wait_changerail);
			return 0;
		}
		case TIMER_EVT:
		{
			if(merge_wait_changerail == EVT_CAST(msg, evt_timeout_t)->type)
			{
				if(LEFTRAIL==RailGetRailState())
				{
					STATE_TRAN(me, &me->cruising);
				}
				else
				{
					MotoSetErrorCode(ERROR_MERGE_FAILED);
					STATE_TRAN(me, &me->forcebrake);
				}
				return 0;
			}
			break;
		}

	}

	return msg;
}

/*
 * 紧急刹车
 */
Msg const * TopForceBrake(car_hsm_t * me, Msg* msg)
{
	switch(msg->evt)
	{
		case ENTRY_EVT:
		{
			g_fbData.mode = ForceBrake;


			BrakeSetBrake(MAX_BRAKE_SIZE);
			MotoSetGear(GEAR_NONE);
			MotoSetThrottle(0);
			return 0;
		}
		case EXIT_EVT:
		{
//			MotoSetErrorCode(ERROR_NONE);
			BrakeSetReady(1);
			return 0;
		}

	}

	return msg;
}

void CarHsmCtor(car_hsm_t * me)
{
	HsmCtor((Hsm *)me, "Car", (EvtHndlr)Top);
	StateCtor(&me->manual, "manual", &((Hsm *)me)->top, (EvtHndlr)TopManual);

	StateCtor(&me->setting, "setting", &((Hsm *)me)->top, (EvtHndlr)TopSetting);

	StateCtor(&me->automode, "automode", &((Hsm *)me)->top, (EvtHndlr)TopAutoMode);
		StateCtor(&me->automode_idle, "automode.idle",&me->automode, (EvtHndlr) AutoModeIdle);
		StateCtor(&me->automode_interjump, "automode.interjump",&me->automode, (EvtHndlr) AutoModeInterJump);
			StateCtor(&me->cruising, "interjump.cruising", &me->automode_interjump, (EvtHndlr) InterJumpCruising);
			StateCtor(&me->straight, "interjump.straight", &me->automode_interjump, (EvtHndlr) InterJumpStraight);
//			StateCtor(&me->pre_curve, "interjump.pre_curve", &me->automode_interjump, (EvtHndlr) InterJumpPreCurve);
			StateCtor(&me->curving, "interjump.curving", &me->automode_interjump, (EvtHndlr) InterJumpCurving);
			StateCtor(&me->uphill, "interjump.uphill", &me->automode_interjump, (EvtHndlr) InterJumpUphill);
			StateCtor(&me->downhill, "interjump.downhill", &me->automode_interjump, (EvtHndlr) InterJumpDownhill);
			me->automode_interjump_hist = &me->cruising;
//			StateCtor(&me->pre_downhill, "interjump.pre_downhill", &me->automode_interjump, (EvtHndlr) InterJumpPreDownhill);
//			StateCtor(&me->pre_seperate, "interjump.pre_seperate", &me->automode_interjump, (EvtHndlr) InterJumpPreSeperate);

		StateCtor(&me->automode_changerail, "automode_changerail",&me->automode, (EvtHndlr) AutomodeChangeRail);
			StateCtor(&me->changerail_waitphoton, "changerail_waitphoton",&me->automode_changerail, (EvtHndlr) ChangeRailWaitPhoton);
			StateCtor(&me->changerail_changing, "changerail_changing",&me->automode_changerail, (EvtHndlr) ChangeRailChanging);

		StateCtor(&me->automode_arrived, "automode_arrived", &me->automode, (EvtHndlr) AutoModeArrived);
//		StateCtor(&me->automode_enterstation, "automode_enterstation",&me->automode, (EvtHndlr) AutomodeEnterStation);
//		StateCtor(&me->automode_stopstation, "automode_stopstation",&me->automode, (EvtHndlr) AutomodeStopStation);
//		StateCtor(&me->autemode_stopstationleave, "autemode_stopstationleave",&me->automode, (EvtHndlr) AutomodeStopStationLeave);
//		StateCtor(&me->automode_leavestation, "automode_leavestation",&me->automode, (EvtHndlr) AutomodeLeaveStation);
//		StateCtor(&me->automode_premerge, "automode_premerge",&me->automode, (EvtHndlr) AutomodePreMerge);
//		StateCtor(&me->automode_merge, "automode_merge",&me->automode, (EvtHndlr) AutomodeMerge);
//			StateCtor(&me->merge_waitphoton, "merge_waitphoton",&me->automode_merge, (EvtHndlr) MergeWaitPhoton);
//			StateCtor(&me->merge_merging, "merge_merging",&me->automode_merge, (EvtHndlr) MergeMerging);

	StateCtor(&me->forcebrake, "forcebrake", &((Hsm *)me)->top, (EvtHndlr)TopForceBrake);
}


static car_hsm_t carHsm;

void CarHsmInit()
{
	CarHsmCtor(&carHsm);
	HsmOnStart((Hsm *) &carHsm);
}
