/*
 * test4GControl.c
 *
 *  用PC上的服务器发送指令，DSP上的4G模块接收后控制驱动电机，变轨电机，刹车电机
 *
 *  Created on: 2018-12-26
 *      Author: zhtro
 */


#include "Message/Message.h"
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include "stdio.h"
#include "stdlib.h"
#include "DSP_Uart/dsp_uart2.h"
#include "uartStdio.h"
#include <ti/sysbios/knl/Clock.h>
#include "task_moto.h"
#include "Test4GControl/test4GControl.h"
#include "Sensor/PhotoElectric/PhotoElectric.h"
#include "task_brake_servo.h"
#include "Parameter.h"
#include "Zigbee/Zigbee.h"



/*
 * 全局变量，给Motor 和Brake 用
 */
uint8_t g_connectStatus;
extern fbdata_t g_fbData;

static 	Clock_Handle _timer;


extern Void taskCellCommunication(UArg a0, UArg a1);
extern void ServoTaskInit();
extern void MototaskInit();
extern Void taskRFID(UArg a0, UArg a1);
extern void taskRFIDHeart(UArg a0, UArg a1);


static void FSprintf(const char *fmt, ...)
{
#if 0
	va_list vp;
    char func[128];
    char * str = func;
    va_start(vp,fmt);
    vsprintf(str,fmt,vp);
    UARTPuts(str,-1);
#endif
}

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

static void resetCarCtrlData()
{
	//初始化motor数据
		MotoSetMotoSel(FRONT_REAR);  //两个电机都用
		MotoSetControlMode(MODE_THROTTLE);  //Throttle模式
		MotoSetGear(GEAR_NONE);    //空挡
		MotoSetThrottle(0);
		BrakeSetBrake(0);
		MotoSetGoalRPM(0);

		FSprintf("reset motor\n");
}


static void sendTimeMsg(UArg arg)
{
	p_msg_t msg;

	msg = Message_getEmpty();
	msg->type = timer;
	msg->data[0] = (uint8_t) arg;

	Message_post(msg);
}

static void InitTimer()
{
	Clock_Params clockParams;

	Error_Block  eb;
	Error_init(&eb);

	Clock_Params_init(&clockParams);
	clockParams.period = 0;       // one shot
	clockParams.startFlag = FALSE;

	_timer = Clock_create(0, 1000, &clockParams, &eb);
}



static void setTimeout(uint32_t timeout,uint8_t msg)
{
	Clock_stop(_timer);
	Clock_setFunc(_timer, sendTimeMsg, (UArg) msg);
	Clock_setTimeout(_timer, timeout);
	Clock_start(_timer);
}


static Void task4GControlMain(UArg a0, UArg a1)
{
	p_msg_t msg;
	int tempint, paramNum;
    int errorCode;
	car_mode_t carMode = Manual; //手动0 设置参数1 自动2
	car_state_t carState = idle;
	car_seperate_state_t seperateState;
	car_merge_state_t mergeState;


    uint16_t RPMparam[car_state_None] = {0};

    InitTimer();

	 // 使用一个Timer来检测4G通信心跳包
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

	// Timer 配置结束


	while(1){
		msg= Message_pend();


		if(msg->type == zigbee)
		{
			/*
			 * 检测心跳包
			 */
			//连接状态 on
			connectionOn();
			//清零clock
			Clock_start(heartClock);

			/*
			 * 模式跳转
			 * 0 手动模式
			 * 1 设置模式
			 * 2 自动模式
			 * 3 紧急制动模式
			 */
			if(msg->data[0] == 'z'){
				tempint = msg->data[1]-'0';   //转换字符串为int
				carMode =(car_mode_t) tempint; //模式跳转
				resetCarCtrlData();  //清零电机控制
				MotoSetAutoMode(carMode);

				switch(carMode)   ///模式跳转动作
				{
                    case Manual:
                        MotoSetErrorCode(ERROR_NONE);
                        BrakeSetReady(1);
                        RailSetReady(1);
                        
                        break;
					case Auto:
						MotoSetGear(GEAR_DRIVE);   //进入自动模式后，默认挂前进档
						carState = idle;
                    
                        MotoSetErrorCode(ERROR_NONE);
                        BrakeSetReady(1);
                        RailSetReady(1);
						break;
                    case Setting:
                        MotoSetErrorCode(ERROR_NONE);
                        BrakeSetReady(1);
                        RailSetReady(1);

						break;
                    case ForceBrake:
                        errorCode = (msg->data[2]-'0')*10 + (msg->data[3]-'0');
                        MotoSetErrorCode(errorCode);
                        break;
				}
			}
		}

		if(msg->type == error)
		{
			carMode = ForceBrake;
			MotoSetErrorCode(msg->data[0]);
		}

		g_fbData.mode = (uint8_t) carMode;
		g_fbData.FSMstate = carState;

		if(carMode == Manual){ //手动模式
			switch(msg->type){
				case zigbee:
				{
					/*
					 *  TODO：简单的命令格式，后续需要改为和RFID协议类似的
					 *   第一位字符表示命令类型，后面的字符串转成uint8_t作为数据
					 *
					 */
					tempint = atoi(&(msg->data[1]));   //转换字符串为int

					switch(msg->data[0])
					{
						case 'm':    //motosel
							MotoSetMotoSel((enum motoSel) tempint);
							break;
						case 'c':    //controlmode
							MotoSetControlMode(tempint);
							break;
						case 'g':    //gear
							MotoSetGear(tempint);
							break;
						case 't':    //throttle
							MotoSetThrottle((uint8_t) tempint);
							break;
						case 'r':    //rail
							RailChangeStart();
							break;
						case 'R': //railstate
							/*setRailState((uint8_t) tempint);*/
							break;
						case 'b':    //brake
							BrakeSetBrake((uint8_t) tempint);
							break;
						case 'h' :   //心跳包
							break;
					}

					break;
				}   //case zigbee
				case rfid:
				{
                    switch(msg->data[0])
                    {
                        /*
                        *TODO:手动状态下防止辅轨出轨操作
                        *1）在辅轨上，处于倒挡，且检测到辅轨起始RFID，进入急停模式，并设置ErrorCode
                        *2）在辅轨上，处于前进挡或低速挡，且检测到辅轨结束RFID，进入急停模式，并设置ErrorCode
                        */
                        case EPC_AUXILIARY_TRACK_START:
                        {
                            if(RIGHTRAIL == RailGetRailState() && (GEAR_REVERSE == MotoGetGear()))
                            {
                                carMode = ForceBrake;
                                MotoSetErrorCode(ERROR_OUT_SAFE_TRACK);
                            }
                            break;
                        }
                        case EPC_AUXILIARY_TRACK_END:
                        {
                            if(RIGHTRAIL == RailGetRailState() && (GEAR_DRIVE == MotoGetGear() || GEAR_LOW == MotoGetGear()))
                            {
                                carMode = ForceBrake;
                                MotoSetErrorCode(ERROR_OUT_SAFE_TRACK);
                            }
                            break;
                        }
                        default:
                            break;
                        
                    }   // switch msg->data[0]
                    break;
				}   //case rfid
               

			}
		}////手动模式

		else if(carMode == Setting)  //设置模式
		{
			switch(msg->type){
				case zigbee:
				{
					tempint = atoi(&(msg->data[1]));   //转换字符串为int
					switch(msg->data[0])
					{
						case 's':
							//设置RPM参数  4位
							paramNum = (msg->data[1]-'0')*10 + (msg->data[2]-'0');

							RPMparam[paramNum] = (msg->data[3]-'0')*1000 + (msg->data[4]-'0')*100 +(msg->data[5]-'0')*10 + (msg->data[6]-'0');
							break;

						case 'k': //设置PID参数  1000000倍
							switch(msg->data[1]){
							case 'i':
								g_param.KI = (msg->data[2]-'0')*1000000 + (msg->data[3]-'0')*100000\
												+(msg->data[4]-'0')*10000 +(msg->data[5]-'0')*1000+(msg->data[6]-'0')*100\
												+(msg->data[7]-'0')*10 + (msg->data[8]-'0');
								break;
							case 'p':
								g_param.KP = (msg->data[2]-'0')*1000000 + (msg->data[3]-'0')*100000\
												+(msg->data[4]-'0')*10000 +(msg->data[5]-'0')*1000+(msg->data[6]-'0')*100\
												+(msg->data[7]-'0')*10 + (msg->data[8]-'0');
								break;
							case 'u':
								g_param.KU = (msg->data[2]-'0')*1000000 + (msg->data[3]-'0')*100000\
												+(msg->data[4]-'0')*10000 +(msg->data[5]-'0')*1000+(msg->data[6]-'0')*100\
												+(msg->data[7]-'0')*10 + (msg->data[8]-'0');
								break;

							}
							break;

						case 'c': // 是否允许变轨
							g_param.EnableChangeRail = (msg->data[1]-'0');
							break;

					}
					break;
				}  //case zigbee
			}
		}

		else if (carMode == Auto) //进入自动模式
		{

			switch(carState)
			{
				case idle:
				{
					if(msg->type == zigbee){
						if(msg->data[0] == 'S' )   //开始巡航
						{
							carState = cruising;
							MotoSetGoalRPM(RPMparam[carState]);
						}
					}
					break;
				}
				case cruising:
				case straight:
				case pre_curve:
				case curving:
				case uphill:
				case pre_downhill:
				case downhill:
                case pre_seperate:
				{
					switch(msg->type)
					{
						case rfid:
						{
							switch(msg->data[0])
							{
								case EPC_STRAIGHT:
									carState = straight;

									break;
								case EPC_PRE_CURVE:
									carState = pre_curve;

									break;
								case EPC_CURVING:
									carState = curving;

									break;
								case EPC_UPHILL:
									carState = uphill;

									break;
								case EPC_PRE_DOWNHILL:
									carState = pre_downhill;

									break;
								case EPC_DOWNHILL:
									carState = downhill;

									break;
								case EPC_PRE_SEPERATE:
									if(g_param.EnableChangeRail == 1){  //在上位机设置了变轨才进入变轨流程
										carState = pre_seperate;
									}
                                    break;
                                case EPC_SEPERATE:
                                    /*
                                    *TODO:特殊处理，解决预分轨状态无法跳出的问题
                                    */
                                    if(carState == pre_seperate)
                                    {
                                        carState = seperate;
						                setTimeout(g_timeout[seperate_wait_photon],seperate_wait_photon);
						                seperateState = s_wait_photon;
                                    }
									break;

							}
							MotoSetGoalRPM(RPMparam[carState]);
							break;
						}
					}

					break;
				} //case pre_seperate

				case seperate:
				{
					switch(seperateState)
					{
						case s_wait_photon:
						{
							if(msg->type == timer && msg->data[0] == seperate_wait_photon)
							{
								seperateState = seperate_cancel;
								FSprintf("seperate_wait_photon timeout \n");
							}

							if(msg->type == photon)
							{
								seperateState = seperating;
								//action
								setTimeout(g_timeout[seperate_wait_changerail], seperate_wait_changerail);
								RailChangeStart();
								FSprintf("begin change rail \n");
							}
							break;

						}

						case seperating:
						{
							if(msg->type == timer && msg->data[0] == seperate_wait_changerail)
							{
								FSprintf("seperate_wait_changerail timeout\n");
								if(RailIsChangeComplete())
								{
									seperateState = seperate_ok;
									FSprintf("seperate OK\n");
                                    /*
                                    *TODO:设置进站ID检测超时
                                    */
                                    setTimeout(g_timeout[seperate_wait_enter_station], seperate_wait_enter_station);
								}
								else{
									//TODO: 紧急制动，并发送ErroCode
									carMode = ForceBrake;
                                    MotoSetErrorCode(ERROR_SEPERATE_FAILED);
									FSprintf("seperate failed\n");
								}
							}
							break;
						}

						case seperate_ok:
						{
                            /*
						    *TODO:超时未检测到进站ID，进入急停模式，并设置ErrorCode
							*/
                            if(msg->type == timer && msg->data[0] == seperate_wait_enter_station)
							{
								carMode = ForceBrake;
                                MotoSetErrorCode(ERROR_WAIT_ENTER_STATION);
							}
                            
							//FIXME: 特殊处理
							if(msg->type == rfid && msg->data[0] ==EPC_ENTER_STATION)
							{
								carState = enter_station;
								MotoSetGoalRPM(RPMparam[carState]);
                                /*
                                *TODO:设置停站ID检测超时
                                */
                                setTimeout(g_timeout[seperate_wait_stop_station], seperate_wait_stop_station);
							}
							break;
						}

						case seperate_cancel:
						{
                            /*分轨失败进入巡航*/
                            carState = cruising;
                            MotoSetGoalRPM(RPMparam[carState]);
							break;
						}
					}

					break;
				}//case seperate:

				case enter_station:
				{
                    /*
				    *TODO:超时未检测到停站ID，进入急停模式，并设置ErrorCode
					*/
                    if(msg->type == timer && msg->data[0] == seperate_wait_stop_station)
					{
						carMode = ForceBrake;
                        MotoSetErrorCode(ERROR_WAIT_STOP_STATION);
					}
                    
					if(msg->type == rfid && msg->data[0] == EPC_STOP_STATION){
						carState = stop_station;
						MotoSetGoalRPM(0);
						setTimeout(g_timeout[station_stop], station_stop);
					}
					break;
				} //case enter_station:

				case stop_station:
				{
					if(msg->type==timer && msg->data[0] == station_stop){
						MotoSetGoalRPM(RPMparam[carState]);
						FSprintf("station_stop timeout\n");
                    
                        /*
                        *TODO:停站启动后，设置离站ID检测超时
                        */
                        setTimeout(g_timeout[seperate_wait_leave_station], seperate_wait_leave_station);
					}

                    /*
				    *TODO:超时未检测到离站ID，进入急停模式，并设置ErrorCode
					*/
                    if(msg->type == timer && msg->data[0] == seperate_wait_leave_station)
					{
						carMode = ForceBrake;
                        MotoSetErrorCode(ERROR_WAIT_LEAVE_STATION);
					}
                    
					if(msg->type == rfid && msg->data[0] == EPC_LEAVE_STATION)
					{
						carState = leave_station;
						MotoSetGoalRPM(RPMparam[carState]);
                        /*
                        *TODO:设置预并轨ID检测超时
                        */
                        setTimeout(g_timeout[seperate_wait_pre_merge], seperate_wait_pre_merge);
					}
					break;
				}

				case leave_station:
				{
                    /*
				    *TODO:超时未检测到预并轨ID，进入急停模式，并设置ErrorCode
					*/
                    if(msg->type == timer && msg->data[0] == seperate_wait_pre_merge)
					{
						carMode = ForceBrake;
                        MotoSetErrorCode(ERROR_WAIT_PRE_MERGE);
					}
                    
					if(msg->type == rfid && msg->data[0] == EPC_PRE_MERGE)
					{
						carState = pre_merge;
						MotoSetGoalRPM(RPMparam[carState]);
                        /*
                        *TODO:设置并轨ID检测超时
                        */
                        setTimeout(g_timeout[seperate_wait_merge], seperate_wait_merge);
					}
					break;
				}

				case  pre_merge:
				{
                    /*
				    *TODO:超时未检测到并轨ID，进入急停模式，并设置ErrorCode
					*/
                    if(msg->type == timer && msg->data[0] == seperate_wait_merge)
					{
						carMode = ForceBrake;
                        MotoSetErrorCode(ERROR_WAIT_MERGE);
					}
                    
					if(msg->type == rfid && msg->data[0] == EPC_MERGE)
					{
						carState = merge;
						mergeState = m_wait_photon;
						MotoSetGoalRPM(RPMparam[carState]);
						setTimeout(g_timeout[merge_wait_photon],merge_wait_photon);
					}
					break;
				}

				case merge:
				{
					switch(mergeState)
					{
						case m_wait_photon:
						{
							if(msg->type==timer && msg->data[0] == merge_wait_photon){
								//TODO: 制动
								carMode = ForceBrake;
								FSprintf("merge_wait_photon timeout\n");
                                MotoSetErrorCode(ERROR_WAIT_MERGE_PHOTON);
							}
							if(msg->type == photon)
							{
								mergeState = merging;
								//action
								setTimeout(g_timeout[merge_wait_changerail], merge_wait_changerail);
								RailChangeStart();
							}

							break;
						}

						case merging:
						{
							if(msg->type==timer && msg->data[0] == merge_wait_changerail){
								FSprintf("merge_wait_changerail timeout\n");
								if(RailIsChangeComplete()){
									mergeState = merge_ok;
									FSprintf("merge ok\n");
								}
								else{
									//紧急刹车
									carMode = ForceBrake;
                                    MotoSetErrorCode(ERROR_MERGE_FAILED);
									FSprintf("merge failed\n");
								}
							}
							break;
						}

						case merge_ok:
						{
                            /*并轨成功进入巡航*/
                            carState = cruising;
                            MotoSetGoalRPM(RPMparam[carState]);
//							//FIXME: 特殊处理
//							if(msg->type == rfid && msg->data[0] == EPC_PRE_CURVE){
//								carState = pre_curve;
//								setCarRPM(RPMparam[carState]);
//							}
							break;
						}
					}
				} //case merge:

			}


		}  //else if(autoMode == 2)  //自动

		if(carMode == ForceBrake)
		{
			MotoSetAutoMode(carMode);
			g_fbData.mode = (uint8_t) carMode;
			BrakeSetBrake(MAX_BRAKE_SIZE);
			MotoSetThrottle(0);
			switch(msg->type){
				case zigbee:
				{
					tempint = atoi(&(msg->data[1]));   //转换字符串为int

					switch(msg->data[0])
					{

						case 'R': //railstate
							/*setRailState((uint8_t) tempint);*/
							break;
					}
				}
			}

		}

		Message_recycle(msg);
	}
}



void test4GControl_init()
{
	Task_Handle task;
	Error_Block eb;
	Task_Params taskParams;


	Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = 6;      //比5高
	taskParams.stackSize = 2048;
	task = Task_create(task4GControlMain, &taskParams, &eb);
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
}
