#include "canModule.h"
#include "task_moto.h"
#include "Sensor/CellCommunication/CellDriver.h"

/*SYSBIOS includes*/
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include "Message/Message.h"
#include "stdio.h"
#include <math.h>
#include "common.h"
#include "task_brake_servo.h"
#include "Parameter.h"
#include "Zigbee/Zigbee.h"
#include "Sensor/RFID/RFID_task.h"
#include "Sensor/ZCP/V2V.h"
#include "Lib/vector.h"
#include "Lib/bitop.h"
#include "logLib.h"
#include "speed_control.h"

/* 宏定义 */
#define RX_MBOX_DEPTH (32)
#define MOTO_CONNECT_TIMEOUT (300)

/********************************************************************************/
/*          外部全局变量                                                              */
/********************************************************************************/
uint8_t g_calibFlag = 0;

#if 1
fbdata_t g_fbData;
extern uint8_t g_connectStatus;
#else 
motordata_t g_fbData.motorDataF, g_fbData.motorDataR;
static uint8_t g_connectStatus = 1;
#endif
static Clock_Handle clockMotoRearHeart;
static Clock_Handle clockMotoFrontHeart;
static Clock_Handle clockThrottle;

static float MotoPidCalc(int16_t expRpm,int16_t realRpm,float kp,float ki, float ku,uint8_t clear);
static uint16_t MotoGoalSpeedGen(uint16_t vc, float ksp, float ksi);
static uint8_t frontValid = 0;
static uint8_t rearValid = 0;
static uint16_t recvCircle = 0;
static float m_distance = 0;
static float lastDistance = 0;
static uint16_t maxSafeDistance = MAX_SAFE_DISTANCE;
static uint16_t minSafeDistance = MIN_SAFE_DISTANCE;
static float kap = 0.02;
static float kai = 0.08;
static float kp = 0.8;
static float ki = 0.009;
/********************************************************************************/
/*          静态全局变量                                                              */
/********************************************************************************/
static Semaphore_Handle txReadySem;
static Semaphore_Handle pidSem;
static Mailbox_Handle rxDataMbox = NULL;

static canDataObj_t canSendData;

/*电机控制量*/
static moto_ctrl_t m_motoCtrl = {
		.MotoSel = FRONT_REAR,
		.ControlMode = MODE_THROTTLE,
		.Gear = GEAR_NONE,
		.Throttle = 0,
		.GoalRPM=0,
		.PidOn = 0
};

extern Semaphore_Handle photoCalib;
extern Mailbox_Handle bSecMbox;
/********************************************************************************/
/*          外部CAN0的用户中断Handler                                                   */
/*                                                                              */
/********************************************************************************/
static void MotoCanIntrHandler(int32_t devsNum,int32_t event)
{
    canDataObj_t rxData;
	
	if (event == 1)         /* 收到一帧数据 */ 
    {
        CanRead(devsNum, &rxData);
        Mailbox_post(rxDataMbox, (Ptr *)&rxData, BIOS_NO_WAIT);
	}
    else if (event == 2)    /* 一帧数据发送完成 */ 
    {
        /* 发送中断 */
        Semaphore_post(txReadySem);
    }

}

static void MotoInitSem()
{
	Semaphore_Params semParams;
    Mailbox_Params mboxParams;

    /* 初始化发送和PID信用量 */
	Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_COUNTING;
    txReadySem = Semaphore_create(1, &semParams, NULL);
    pidSem = Semaphore_create(1, &semParams, NULL);

    /* 初始化接收邮箱 */
    Mailbox_Params_init(&mboxParams);
    rxDataMbox = Mailbox_create (sizeof (canDataObj_t),RX_MBOX_DEPTH, &mboxParams, NULL);
    
}
static xdc_Void MotoRearConnectClosed(xdc_UArg arg)
{
    p_msg_t msg;
    /*
    *TODO:添加后轮断连处理，发送错误码消息
    */
    msg = Message_getEmpty();
	msg->type = error;
	msg->data[0] = ERROR_MOTOR_TIMEOUT;
	msg->dataLen = 1;
	Message_post(msg);
	//LogMsg("Moto Rear Connect Failed!!\r\n");
	rearValid = 0;
}

static xdc_Void MotoFrontConnectClosed(xdc_UArg arg)
{
    p_msg_t msg;
    /*
    *TODO:添加前轮断连处理，发送错误码消息
    */
    msg = Message_getEmpty();
	msg->type = error;
	msg->data[0] = ERROR_MOTOF_TIMEOUT;
	msg->dataLen = 1;
	Message_post(msg);
	frontValid = 0;
	//LogMsg("Moto Front Connect Failed!!\r\n");
}

static xdc_Void ThrottleTimeout(xdc_UArg arg)
{
	if(MotoGetCarMode() == Manual)
	{
		m_motoCtrl.Throttle = 0;
	}
}

static void MotoInitTimer()
{
	Clock_Params clockParams;
	Clock_Params_init(&clockParams);
	clockParams.period = 0;       // one shot
	clockParams.startFlag = FALSE;
	clockMotoRearHeart = Clock_create(MotoRearConnectClosed, 3000, &clockParams, NULL);
	clockMotoFrontHeart = Clock_create(MotoFrontConnectClosed, 3000, &clockParams, NULL);
	clockThrottle = Clock_create(ThrottleTimeout, 2000, &clockParams, NULL);
	Clock_start(clockMotoRearHeart);
	Clock_start(clockMotoFrontHeart);
}

static void MotoSendTask(void)
{
    canData_t *canTx = (canData_t *)canSendData.Data;

    /*初始化帧类型*/
    canSendData.ID = 0;
    canSendData.SendType = 0;
    canSendData.RemoteFlag = 0;
    canSendData.ExternFlag = 1;
    canSendData.DataLen = 8;
    
    /*初始化电机控制数据*/
    canTx->Gear = 0x00;         //空挡
    canTx->ThrottleL = 0x00;
    canTx->ThrottleH = 0x00;
    canTx->Mode = 0x20;         //油门，前进
    canTx->TorqueL = 0x00;
    canTx->TorqueH = 0x00;
    canTx->SpeedOrBreakL = 0x00;
    canTx->SpeedOrBreakH = 0x00;
    
	while (1)
	{
        Semaphore_pend(pidSem, 100);        //100ms超时发送油门

		canTx->Gear = MotoGetGear();
        
		if (g_connectStatus)
		{
			canTx->ThrottleL = MotoGetThrottle();
			canTx->ThrottleH = 0;
		}
		else
		{
			canTx->ThrottleL = 0;
			canTx->ThrottleH = 0;
		}


		if ((MotoGetMotoSel() == FRONT_ONLY) || (MotoGetMotoSel() == FRONT_REAR))
		{
			canTx->Mode = 0x28;		                //前后电机方向相反
			canSendData.ID = MOTO_F_CANID1;
            //canSendData.DataLen = 8;
            Semaphore_pend(txReadySem, BIOS_WAIT_FOREVER);
			CanWrite(MOTO_CAN_DEVNUM, &canSendData);      
		}
        
		if ((MotoGetMotoSel() == REAR_ONLY) || (MotoGetMotoSel() == FRONT_REAR))
		{
			canTx->Mode = 0x20;		                //前后电机方向相反
			canSendData.ID = MOTO_R_CANID1;
            //canSendData.DataLen = 8;
            Semaphore_pend(txReadySem, BIOS_WAIT_FOREVER);
			CanWrite(MOTO_CAN_DEVNUM, &canSendData);      
		}

		
	}/* end while(1) */
}

/*
 * 根据RFID和轮子圈数，计算距离
 */
static rfidPoint_t *calibrationQueue;

/*
 * 最大校准误差10m
 */
#define MAX_CALIBRATION_DISTANCE (100)

static void MotoUpdateDistanceTask(void)
{
    uint16_t lastCircleNum = 0;
    uint16_t curCircleNum = 0;
    uint16_t circleDiff = 0;
    p_msg_t msg;
    uint8_t size = 0;
    uint32_t calibrationPoint = 0;
    uint32_t calibrationPointMin = 0;

    uint8_t lastbSection = 0;
    uint8_t bSection = 0;
    int32_t deltaDist = 0;
    rfidPoint_t calibRfid;
    car_mode_t mode = Manual;
    float step = 0;
    const int UPDATE_INTERVAL = 100;
    Mailbox_Handle RFIDV2vMbox = 0;
    epc_t epc;
    epc_t lastEpc;

    while(1)
    {
    	/*
    	 * 100ms更新一次距离
    	 */
        Task_sleep(UPDATE_INTERVAL);
        lastDistance = m_distance;

        /*
         * 根据车轮圈数更新距离
         */
        curCircleNum = MotoGetCircles();
        if(lastCircleNum > curCircleNum)
        {
            circleDiff = 65535 -lastCircleNum + curCircleNum;
        }
        else
        {
            circleDiff = curCircleNum - lastCircleNum;
        }

        lastCircleNum = curCircleNum;
        /*
         * 过滤掉CAN传输出错的圈数数据
         * 圈数差每100ms不超过10圈 , 认为0.1s跑不了3m
         */
        if(circleDiff<10)
        {
            step = circleDiff * (sysParam.wheelPerimeter/100.0) / WHEEL_SPEED_RATIO;
            if(GEAR_REVERSE == MotoGetGear())
            {
                if(m_distance > step)
                    m_distance -= step;
                else
                    m_distance = TOTAL_DISTANCE - step + m_distance;
            }
            else
            {
                m_distance += step;
                if(m_distance >TOTAL_DISTANCE )
                {
                	m_distance -= TOTAL_DISTANCE;
                }
            }
        }

#if 0
        mode = MotoGetCarMode();
        /*
         * 非Auto模式下，采用物理RFID校准距离
         */
        if(mode != Auto)
        {
            RFIDV2vMbox = RFIDGetV2vMailbox();
            if(RFIDV2vMbox == NULL)
            {
                Task_sleep(100);
                continue;
            }
            //读到RFID
            if(TRUE == Mailbox_pend(RFIDV2vMbox,(Ptr *)&epc,UPDATE_INTERVAL))
            {
                /*
                 * 进入B段，计算deltaS
                 */
                if(EPC_AB_A == lastEpc.ab && EPC_AB_B == epc.ab)
                {
                    V2VSetDeltaDistance((int32_t)(epc.distance)-(int32_t)m_distance);
                }
                else if(EPC_AB_A == epc.ab)
                {
                    V2VSetDeltaDistance(0);
                }

                //校正距离
                lastCircleNum = MotoGetCircles();
                m_distance = epc.distance;

                lastEpc = epc;
            }
            g_fbData.distance = m_distance;
            continue;
        }
#endif

        if(bSecMbox != NULL && Mailbox_pend(bSecMbox,&deltaDist,BIOS_NO_WAIT))
		{
        	/*
        	 * B段距离校准
        	 */
        	m_distance = m_distance + deltaDist;
        	V2VSetDeltaDistance(deltaDist);
		}

        size = vector_size(calibrationQueue);
        if(size == 0)
        {
            /*
             * 无校准点
             */
            g_fbData.distance = m_distance;
            continue;
        }

        calibrationPoint = (calibrationQueue[0].byte[8] << 16) +
                (calibrationQueue[0].byte[9] << 8) +
                calibrationQueue[0].byte[10];
        calibrationPointMin = calibrationPoint - (MAX_CALIBRATION_DISTANCE/2);

        //从0点翻转为281的情况也不要进入校准流程
        if(lastDistance < calibrationPointMin && m_distance >= calibrationPointMin
			&& (!(lastDistance < 50 && m_distance >TOTAL_DISTANCE - 50) )
			&& g_calibFlag == 0)
        {
            /*
             * 到达校准点
             * 设置校准启动标志，并清除光电对管信号量
             */
            g_calibFlag = 1;
            Semaphore_pend(photoCalib,BIOS_NO_WAIT);
            LogMsg("Calibration Start:%d\r\n",calibrationPoint);
        }

        if(g_calibFlag == 0)
        {
            /*
             * 尚未到达校准点
             */
            g_fbData.distance = m_distance;
            continue;
        }

        if(FALSE == Semaphore_pend(photoCalib,BIOS_NO_WAIT))
        {
            /*
             * 尚未检测到光电对管,且超出校准范围
             * TODO:尚未考虑环形翻转
             */
            if((m_distance - calibrationPointMin) > MAX_CALIBRATION_DISTANCE)
            {
                /*
                *超出校准点，清除该校准点，并发送错误
                */
                g_calibFlag = 0;
                if(g_param.cycleRoute == 1)
                {
                    calibRfid = calibrationQueue[0];
                    vector_erase(calibrationQueue,0);
                    vector_push_back(calibrationQueue,calibRfid);
                }
                else
                {
                    vector_erase(calibrationQueue,0);
                }


                msg = Message_getEmpty();
                msg->type = error;
                msg->data[0] = ERROR_CALIBRATION_OUTRANGE;
                msg->dataLen = 1;
                Message_post(msg);
            }
            g_fbData.distance = m_distance;
            continue;
        }

        /*
         * 检测到光电对管
         */
#if 0
        lastbSection = bSection;
        bSection = calibrationQueue[0].byte[6] >> 7;

        if(lastbSection == 0 && bSection == 1)
        {
            deltaDist = (calibrationQueue[0].byte[14] << 24) +
                        (calibrationQueue[0].byte[15] << 16) +
                        (calibrationQueue[0].byte[16] << 8) +
                        calibrationQueue[0].byte[17] ;
            V2VSetDeltaDistance(deltaDist);
            m_distance = calibrationPoint + deltaDist;
            LogMsg("Calibration(B) End:%d\r\n",calibrationPoint);
        }
        else
#endif
        {
            m_distance = calibrationPoint;
            LogMsg("Calibration(A) End:%d\r\n",calibrationPoint);
        }

        /*
         * 校准结束，清除校准点
         */
        g_calibFlag = 0;
        g_fbData.distance = m_distance;

        if(g_param.cycleRoute == 1)
        {
            calibRfid = calibrationQueue[0];
            vector_erase(calibrationQueue,0);
            vector_push_back(calibrationQueue,calibRfid);
        }
        else
        {
            vector_erase(calibrationQueue,0);
        }
    }
}

#define VOL_LOW_THRESHOLD (65.0)
static void MotoRecvTask(void)
{
    uint16_t recvRpm = 0;
    float adjThrottle = 0;
    float adjbrake = 0;
    static float hisThrottle = 0;
    uint16_t frontRpm = 0;
    uint16_t rearRpm = 0;
    uint16_t frontCircle = 0;
    uint16_t rearCircle = 0;
    uint16_t calcRpm = 0;
    uint32_t canID;
    uint32_t maxThrottle = 0;
    canDataObj_t canRecvData;
    uint16_t vg = 0;
    p_msg_t msg;
    uint8_t pidMode = 0;
    uint8_t pidModeOld = 0;
    int volLowCount = 0;
	float voltage;
	int RPMzeroCount = 0;
	uint8_t fixedBrakeGiven = 0;

	int frontReverseStartDistance = 0;
	int rearReverseStartDistance = 0;
	int distanceDiff = 0;

	const int REVERSE_THR = 10;  //1m

	g_fbData.motorDataF.MotoId = MOTO_FRONT;
	g_fbData.motorDataR.MotoId = MOTO_REAR;
	
	while (1)
	{
        Mailbox_pend(rxDataMbox, (Ptr *)&canRecvData, BIOS_WAIT_FOREVER);
        
        canID = (canRecvData.ID) & 0x1fffffff;
		switch (canID)
		{
    		//Front电机反馈信息打包
    		case MOTO_F_CANID2:
    			g_fbData.motorDataF.Gear       = (uint8_t)canRecvData.Data[0];
    			g_fbData.motorDataF.ThrottleL  = (uint8_t)canRecvData.Data[1];
    			g_fbData.motorDataF.ThrottleH  = (uint8_t)canRecvData.Data[2];
    			g_fbData.motorDataF.MotoMode   = (uint8_t)canRecvData.Data[3];
    			g_fbData.motorDataF.RPML       = (uint8_t)canRecvData.Data[4];
    			g_fbData.motorDataF.RPMH       = (uint8_t)canRecvData.Data[5];
    			g_fbData.motorDataF.MotoTemp   = (uint8_t)canRecvData.Data[6] - 40;
    			g_fbData.motorDataF.DriverTemp = (uint8_t)canRecvData.Data[7] - 40;

    			if( BF_GET(g_fbData.motorDataF.MotoMode,4,1) == 1)
    			{
    				//反转
    			}
    			else
    			{
    				frontReverseStartDistance = MotoGetCarDistance();
    			}

    			if(MotoGetCarDistance() < frontReverseStartDistance)
    			{
    				distanceDiff = TOTAL_DISTANCE - frontReverseStartDistance + MotoGetCarDistance();
    			}
    			else
    			{
    				distanceDiff = MotoGetCarDistance() - frontReverseStartDistance;
    			}


    			if(distanceDiff > REVERSE_THR)
    			{
    				frontReverseStartDistance = MotoGetCarDistance();
    				Message_postError(ERROR_REVERSING);
    			}

    			//LogMsg("moto turn front : %d\n", BF_GET(g_fbData.motorDataF.MotoMode,4,1));
    			/*
    			 * 计算前轮转速
    			 */
    			frontRpm = (g_fbData.motorDataF.RPMH << 8) + g_fbData.motorDataF.RPML;
    			break;
    		case MOTO_F_CANID3:
    			g_fbData.motorDataF.VoltL      = (uint8_t)canRecvData.Data[0];
    			g_fbData.motorDataF.VoltH      = (uint8_t)canRecvData.Data[1];
    			g_fbData.motorDataF.CurrentL   = (uint8_t)canRecvData.Data[2];
    			g_fbData.motorDataF.CurrentH   = (uint8_t)canRecvData.Data[3];
    			g_fbData.motorDataF.DistanceL  = (uint8_t)canRecvData.Data[4];
    			g_fbData.motorDataF.DistanceH  = (uint8_t)canRecvData.Data[5];
    			g_fbData.motorDataF.ErrCodeL   = (uint8_t)canRecvData.Data[6];
    			g_fbData.motorDataF.ErrCodeH   = (uint8_t)canRecvData.Data[7];
    			frontCircle = (g_fbData.motorDataF.DistanceH << 8) + g_fbData.motorDataF.DistanceL;
    			/* 收到心跳，重启定时器 */
				Clock_setTimeout(clockMotoFrontHeart,MOTO_CONNECT_TIMEOUT);
				Clock_start(clockMotoFrontHeart);

				if(g_fbData.motorDataF.ErrCodeL == 0 && g_fbData.motorDataF.ErrCodeH == 0)
					frontValid = 1;
				else
					frontValid = 0;

				/*
				 * 检测电压是否异常
				 */
				voltage = (g_fbData.motorDataF.VoltL + g_fbData.motorDataF.VoltH *256 ) * 0.1;
				if(voltage < VOL_LOW_THRESHOLD)
				{
					volLowCount++;
					if(volLowCount>60) //50ms*60 = 3s
					{
						volLowCount=0;
						Message_postEvent(internal, IN_EVTCODE_RAIL_POWER_DROP);
					}
				}
				else
				{
					volLowCount = 0;
				}
    			break;
    		case MOTO_F_CANID4:
    			g_fbData.motorDataF.TorqueCtrlL    = (uint8_t)canRecvData.Data[0];
    			g_fbData.motorDataF.TorqueCtrlH    = (uint8_t)canRecvData.Data[1];
    			g_fbData.motorDataF.RPMCtrlL       = (uint8_t)canRecvData.Data[2];
    			g_fbData.motorDataF.RPMCtrlH       = (uint8_t)canRecvData.Data[3];
    			g_fbData.motorDataF.TorqueL        = (uint8_t)canRecvData.Data[4];
    			g_fbData.motorDataF.TorqueH        = (uint8_t)canRecvData.Data[5];
    			g_fbData.motorDataF.CanReserved1   = (uint8_t)canRecvData.Data[6];
    			g_fbData.motorDataF.CanReserved2   = (uint8_t)canRecvData.Data[7];
    			break;
    		//Front电机反馈信息打包
    		case MOTO_R_CANID2:
    			g_fbData.motorDataR.Gear           = (uint8_t)canRecvData.Data[0];
    			g_fbData.motorDataR.ThrottleL      = (uint8_t)canRecvData.Data[1];
    			g_fbData.motorDataR.ThrottleH      = (uint8_t)canRecvData.Data[2];
    			g_fbData.motorDataR.MotoMode       = (uint8_t)canRecvData.Data[3];
    			g_fbData.motorDataR.RPML           = (uint8_t)canRecvData.Data[4];
    			g_fbData.motorDataR.RPMH           = (uint8_t)canRecvData.Data[5];
    			g_fbData.motorDataR.MotoTemp       = (uint8_t)canRecvData.Data[6] - 40;
    			g_fbData.motorDataR.DriverTemp     = (uint8_t)canRecvData.Data[7] - 40;


    			if( BF_GET(g_fbData.motorDataR.MotoMode,4,1) == 1)
    			{
    				//反转
    			}
    			else
    			{
    				rearReverseStartDistance = MotoGetCarDistance();
    			}

    			if(MotoGetCarDistance() < rearReverseStartDistance)
    			{
    				distanceDiff = TOTAL_DISTANCE - rearReverseStartDistance + MotoGetCarDistance();
    			}
    			else
    			{
    				distanceDiff = MotoGetCarDistance() - rearReverseStartDistance;
    			}


    			if(distanceDiff > REVERSE_THR)
    			{
    				rearReverseStartDistance = MotoGetCarDistance();
    				Message_postError(ERROR_REVERSING);
    			}

    			//LogMsg("moto turn rear : %d\n", BF_GET(g_fbData.motorDataR.MotoMode,4,1));
    			/*
                 * 计算后轮转速
                 */
    			rearRpm = (g_fbData.motorDataR.RPMH << 8) + g_fbData.motorDataR.RPML;

    			break;
    		case MOTO_R_CANID3:
    			g_fbData.motorDataR.VoltL          = (uint8_t)canRecvData.Data[0];
    			g_fbData.motorDataR.VoltH          = (uint8_t)canRecvData.Data[1];
    			g_fbData.motorDataR.CurrentL       = (uint8_t)canRecvData.Data[2];
    			g_fbData.motorDataR.CurrentH       = (uint8_t)canRecvData.Data[3];
    			g_fbData.motorDataR.DistanceL      = (uint8_t)canRecvData.Data[4];
    			g_fbData.motorDataR.DistanceH      = (uint8_t)canRecvData.Data[5];
    			g_fbData.motorDataR.ErrCodeL       = (uint8_t)canRecvData.Data[6];
    			g_fbData.motorDataR.ErrCodeH       = (uint8_t)canRecvData.Data[7];
    			rearCircle = (g_fbData.motorDataR.DistanceH << 8) + g_fbData.motorDataR.DistanceL;
    			/* 收到心跳，重启定时器 */
    			Clock_setTimeout(clockMotoRearHeart,MOTO_CONNECT_TIMEOUT);
    			Clock_start(clockMotoRearHeart);
    			if(g_fbData.motorDataR.ErrCodeL == 0 && g_fbData.motorDataR.ErrCodeH == 0)
					rearValid = 1;
				else
					rearValid = 0;

    			break;
    		case MOTO_R_CANID4:
    			g_fbData.motorDataR.TorqueCtrlL    = (uint8_t)canRecvData.Data[0];
    			g_fbData.motorDataR.TorqueCtrlH    = (uint8_t)canRecvData.Data[1];
    			g_fbData.motorDataR.RPMCtrlL       = (uint8_t)canRecvData.Data[2];
    			g_fbData.motorDataR.RPMCtrlH       = (uint8_t)canRecvData.Data[3];
    			g_fbData.motorDataR.TorqueL        = (uint8_t)canRecvData.Data[4];
    			g_fbData.motorDataR.TorqueH        = (uint8_t)canRecvData.Data[5];
    			g_fbData.motorDataR.CanReserved1   = (uint8_t)canRecvData.Data[6];
    			g_fbData.motorDataR.CanReserved2   = (uint8_t)canRecvData.Data[7];
    			break;
    		default:
    			break;
		}/* switch */

		/*
		 * PID计算条件：
		 * 若前轮有效，以收到前轮速度反馈时刻计算PID。否则，以后轮为准
		 */
		if((frontValid == 1 && canID == MOTO_F_CANID2) ||
		        (frontValid == 0 && rearValid == 1 && canID == MOTO_R_CANID2))
		{
		    /*
		     * 获取车辆转速: 增加了只有一个轮子正常的处理
		     */
			recvCircle = frontCircle;
		    if(frontValid == 1 && rearValid == 1)
		    {
		        /*
		         * 两轮驱动器均正常时，采用平均速度
		         * 限定最大油门
		         */
		        recvRpm = (frontRpm + rearRpm)/2;
		        maxThrottle = sysParam.maxtThrottle;
//		        recvCircle = (rearCircle + frontCircle)/2;

		        /*
                 * 前后轮转速差过大，视为异常，进入急停模式
                 */
		        /*
		        if(abs((int32_t)frontRpm - (int32_t)rearRpm) > MAX_DIFF_RPM)
		        {

                    msg = Message_getEmpty();
                    msg->type = error;
                    msg->data[0] = ERROR_RPM_ABNORMAL;
                    msg->dataLen = 1;
                    Message_post(msg);
		        }
		        */

		    }
		    else if(frontValid == 1)
		    {
		        recvRpm = frontRpm;
//		        recvCircle = frontCircle;
		        maxThrottle = sysParam.maxtThrottle;
		    }
		    else if(rearValid == 1)
		    {
		        recvRpm = rearRpm;
//		        recvCircle = rearCircle;
		        maxThrottle = sysParam.maxtThrottle;
		    }
		    else;

		    g_fbData.recvRPM = recvRpm;
            g_fbData.calcRPM = calcRpm;

            /*
             * PID运算
             *  a)根据加速曲线，拟合目标速度(goalRPM)，输出计算速度(calcRPM)
             *  b)根据calcRPM，进行PID调节
             */

            vg = MotoGoalSpeedGen(recvRpm, g_param.KSP, g_param.KSI);

            pidModeOld = pidMode;
            pidMode = MotoGetPidOn();

            if( pidMode  )
            {
//#if 1
//                /*
//                 * 拟合加速曲线（当前为固定加速度曲线）
//                 */
//                if ( abs((int)calcRpm - (int)(vg)) < DELTA_RPM )
//                {
//                    calcRpm = vg;
//                }
//                else
//                {
//                    if(calcRpm < vg)
//                    {
//                        calcRpm+=DELTA_RPM;
//                    }
//                    else if(calcRpm > vg)
//                    {
//                        calcRpm-=DELTA_RPM;
//                    }
//                }
//#else
//                calcRpm = SpeedGenerate(calcRpm, vg);
//#endif

            	calcRpm = vg;
                /*
                 * 限定最大速度
                 */
                calcRpm = (calcRpm > RPM_LIMIT) ? RPM_LIMIT : calcRpm;

                /*
                 * PID计算调节量
                 */
                adjThrottle = MotoPidCalc(calcRpm,recvRpm,kp,
                                        ki,0, 0);

                hisThrottle += adjThrottle;

                //如果速度为0，给一个固定刹车量,每次为0只给一次
                if( MotoGetRealRPM() == 0)
                {
                	RPMzeroCount ++;
                	if( RPMzeroCount >= 2)
                	{
                		RPMzeroCount = 0;
                		if( !fixedBrakeGiven)
                		{
							hisThrottle = -FORCE_BRAKE_SIZE;
							fixedBrakeGiven = 1;
                		}
                	}
                }
                else if (MotoGetRealRPM() > 0)
                {
                	RPMzeroCount = 0;
                	fixedBrakeGiven = 0;
                }


                if(hisThrottle < 0)
                {
                    /*
                     * 刹车模式:
                     *  a)关闭油门
                     *  b)增加刹车偏移量
                     *  c)设定刹车
                     */
                    MotoSetThrottle(0);

                    adjbrake = sysParam.brakeOffset - hisThrottle;
                    if(adjbrake > MAX_BRAKE_SIZE)
                    {
                        hisThrottle = sysParam.brakeOffset - MAX_BRAKE_SIZE;
                        adjbrake = MAX_BRAKE_SIZE;
                    }

                    BrakeSetBrake( round(adjbrake));

                }
                else
                {
                    /*
                     * 油门模式:
                     *  a)放开刹车
                     *  b)设置油门
                     */
                    BrakeSetBrake(0);

                    if(hisThrottle > maxThrottle)
                    {
                        hisThrottle = maxThrottle;
                    }
                    MotoSetThrottle(round(hisThrottle));
                }
            }
            else
            {
                /*
                 * 退出PID模式时，清除PID相关计算历史值
                 */
                calcRpm=0;
                hisThrottle = 0;
                MotoPidCalc(0,0,0,0,0,1);
                if(pidModeOld)
                {
                    MotoSetThrottle(0);
                }
            }

            /*
             * Post信用量到MotoSendTask
             */
            Semaphore_post(pidSem);
		}/*if(canID == ....)*/

	}
}
//TODO: 发送机车状态到4G
void MotoSetSafeDistance(uint16_t minValue,uint16_t maxValue)
{
    maxSafeDistance = maxValue;
    minSafeDistance = minValue;
}
#if 0
static uint16_t MotoGoalSpeedGen(uint16_t vc, float ksp, float ksi)
{
    uint16_t ds;
    uint16_t da;
    uint16_t dsa;
    uint16_t vf;
    uint32_t di;
    uint16_t vgs;
    static uint16_t vg;
    int32_t dis;
    int32_t vt;
    static float disSum = 0;
    static uint8_t disLess = 0;
    float vd;
    const float KS = 0.01;
    const float KA = 0.01;
    /*
     * 根据车速计算安全距离
     * ds: 安全距离
     * ks: 比例系数-安全距离
     * ka: 比例系数-远离距离
     * vc: 车辆速度
     * da: 远离距离（超出安全距离之后的远离量）
     * dsa: 安全距离+远离距离
     */
    ds = minSafeDistance + KS*vc;
    if(ds > maxSafeDistance)
    {
        ds = maxSafeDistance;
    }

    da = MIN_AWAY_DISTANCE + KA*vc;
    if(da > MAX_AWAY_DISTANCE)
    {
        da = MAX_AWAY_DISTANCE;
    }

    dsa = ds+da;

    /*
     * 根据安全距离调整车辆目标速度
     * di: 车辆距离
     * dis: 车辆距离-安全距离
     * ksp: 距离PID比例系数(p)
     * ksi: 距离PID积分系数(i)
     * vf: 前车速度
     * vg: 目标速度
     * vgs: 设定的目标速度
     * vt: 临时速度变量
     */
    vf = V2VGetFrontCarSpeed();
    di = V2VGetDistanceToFrontCar();
    vgs = MotoGetGoalRPM();

    if(di < ds)
    {
        dis = (int) di - (int) ds;

        disSum += ksi * dis;
        vd = ksp*dis + disSum;

        vt = vf+vd;

        if(vt < 0)
            vg = 0;
        else
            vg = vt;

        if(vg > vgs)
        {
            vg = vgs;
        }

        disLess = 1;
    }
    else if(di < dsa)
    {
        /*
         * (ds < di < dsa) vg保持不变
         */

        if(disLess == 0)
        {
            vg = vgs;
        }

        /*
         * 清除积分累计值
         */
        disSum = 0;
    }
    else
    {
        vg = vgs;
        disLess = 0;

        /*
         * 清除积分累计值
         */
        disSum = 0;
    }
    return vg;
}
#else
/*
 * 修改：
 * 1.删除远离距离；
 * 2.由位置PID改为增量式PID模式；
 * 3.增加调整区的特殊处理：采用另外一组参数；
 */
static uint16_t MotoGoalSpeedGen(uint16_t vc, float ksp, float ksi)
{
    uint16_t ds;
    uint16_t vf;
    uint32_t di;
    uint16_t vgs;
    static int16_t vg;
    static int32_t dis = 0;
    static int32_t disOld = 0;
    int32_t disDelta = 0;
    int32_t vt;
    float vd = 0;
    static float vdTmp = 0;
    const float KS = 0.2;

    /*
     * 根据车速计算安全距离
     * ds: 安全距离
     * ks: 比例系数-安全距离
     * vc: 车辆速度
     */
    ds = minSafeDistance + KS*vc;
    if(ds > maxSafeDistance)
    {
        ds = maxSafeDistance;
    }

    g_fbData.ds = ds;
    /*
     * 根据安全距离调整车辆目标速度
     * di: 车辆距离
     * dis: 车辆距离-安全距离
     * disDelta: 相邻两帧dis的差值
     * ksp: 距离PID比例系数(p)
     * ksi: 距离PID积分系数(i)
     * vf: 前车速度
     * vg: 目标速度
     * vgs: 设定的目标速度
     * vt: 临时速度变量
     */
    vf = V2VGetFrontCarSpeed();
    di = V2VGetDistanceToFrontCar();
    vgs = MotoGetGoalRPM();

    /*
     * 限定车辆之间的最大距离-200m
     */
    if(di > 2000)
        di = 2000;

    if(di>maxSafeDistance)
    {
    	vg = vgs;
    	return vg;
    }

    disOld = dis;
    dis = (int) di - (int) ds;
    disDelta = dis - disOld;

    if(V2VIsSameAdjustArea())
    {
        /*
         * 车辆处于同一调整区
         */
        vd = kap*disDelta + kai*dis;
    }
    else
    {
        vd =  ksp*disDelta + ksi*dis;
    }

    vg = vg+vd;

    if(vg > vgs)
    {
        /*
         *积分正向饱和
         */
        vg = vgs;
    }
    else if(vg < 0)
    {
        /*
         *积分负向饱和
         */
        vg = 0;
    }
    else
    {
        /*
         * 积分未饱和
         */

    }

    return vg;
}

#endif

/*****************************************************************************
 * 函数名称: static uint8_t crc8Calc(uint8_t *pData, uint8_t len)
 * 函数说明: CRC-8计算
 * 输入参数:
 *          pData:数据指针地址
 *          len:长度
 * 输出参数: 无
 * 返 回 值: 无
 * 备注: 多项式(x8+x5+x4+1)
*****************************************************************************/
static uint8_t crc8Calc(uint8_t *pData, uint8_t len)
{
    uint8_t i,j;
    uint8_t crc;
    crc = 0xff;

    for(j=0; j<len; j++)
    {
        crc = *pData ^ crc;  /* 每次先与需要计算的数据异或,计算完指向下一数据 */

        for (i=0; i<8; i++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc = (crc >> 1);
        }

        pData++;
    }


    return (crc);

}
void MotoSendFdbkToCellTask()
{
	/*
	 * 反馈数据包格式为 包头(0xAA 0x42 0x55) + 长度 + 数据 + CRC8 + 包尾(0x0D 0x99 0xCC)
	 */
	uint8_t  sendbuff[512];
	int32_t tms;
	int bufflen = sizeof(g_fbData)+8;

	sendbuff[0] = 0xAA;
	sendbuff[1] = 0x42;
	sendbuff[2] = 0x55;
	sendbuff[3] = sizeof(g_fbData);
	sendbuff[bufflen-3] = 0x0D;
	sendbuff[bufflen-2] = 0x99;
	sendbuff[bufflen-1] = 0xCC;

	while(1){
		/*
		fbData.motorDataR.ThrottleL++;
		fbData.motorDataR.RPMH++;
		fbData.motorDataF.ThrottleH++;
		fbData.motorDataF.RPML++;
		*/
		g_fbData.brake = BrakeGetBrake();
		g_fbData.railstate = RailGetRailState();
		userGetMS(&tms);
		g_fbData.rfidReadTime = tms;
		memcpy(sendbuff+4,(char*) &g_fbData, sizeof(g_fbData) );

		/*
		 * 插入CRC8
		 */
		sendbuff[bufflen-4] = crc8Calc(&sendbuff[4],sizeof(g_fbData));

		ZigbeeSend(sendbuff, bufflen);

		Task_sleep(100);
	}
}


void MototaskInit()
{
	Task_Handle task;
	Task_Params taskParams;

    /*初始化信用量*/
    MotoInitSem();

    /*初始化定时器*/
    MotoInitTimer();

    /*初始化CAN设备*/
    CanOpen(CAN_DEV_0, MotoCanIntrHandler, CAN_DEV_0);
    
    Task_Params_init(&taskParams);
	taskParams.priority = 5;
	taskParams.stackSize = 2048;
    
	task = Task_create(MotoSendTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

    task = Task_create(MotoRecvTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}

    task = Task_create(MotoUpdateDistanceTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}



	taskParams.priority = 2;
    task = Task_create(MotoSendFdbkToCellTask, &taskParams, NULL);
	if (task == NULL) {
		System_printf("Task_create() failed!\n");
		BIOS_exit(0);
	}


}

uint16_t MotoGetRealRPM(void)
{
	uint16_t uCarRPM;
	if (MotoGetMotoSel() == FRONT_ONLY)
	{
		uCarRPM = (g_fbData.motorDataF.RPMH << 8) + g_fbData.motorDataF.RPML;
	}
	else if (MotoGetMotoSel() == REAR_ONLY)
	{
		uCarRPM = (g_fbData.motorDataR.RPMH << 8) + g_fbData.motorDataR.RPML;
	}
	else if (MotoGetMotoSel() == FRONT_REAR)
	{
		uCarRPM = ((g_fbData.motorDataF.RPMH << 8) + g_fbData.motorDataF.RPML + (g_fbData.motorDataR.RPMH << 8) + g_fbData.motorDataR.RPML) / 2;
	}

	return uCarRPM;
}
#if 0
float MotoPidCalc(int16_t expRpm,int16_t realRpm,float kp,float ki, uint8_t clear)
{
    int32_t diffRpm;
    float adjThrottle;
    static float lastThrottle = 0;

    if(clear==1)
    {
    	lastThrottle =0;
    	return 0;
    }

    diffRpm = expRpm - realRpm;

    /*限定差值最大范围*/
    if(diffRpm > DIFF_RPM_UPSCALE)
        diffRpm = DIFF_RPM_UPSCALE;
    else if(diffRpm < DIFF_RPM_DWSCALE)
        diffRpm = DIFF_RPM_DWSCALE;
    else;


    adjThrottle = kp*diffRpm + ki*lastThrottle;


    if(adjThrottle > MAX_THROTTLE_SIZE)
        adjThrottle = MAX_THROTTLE_SIZE;
    else if(adjThrottle < MIN_THROTTLE_SIZE)
        adjThrottle = MIN_THROTTLE_SIZE;
    else;

    lastThrottle = adjThrottle;

    return adjThrottle;
}
#endif

float MotoPidCalc(int16_t expRpm,int16_t realRpm,float kp,float ki, float ku,uint8_t clear)
{
    int32_t diffRpm;
    float adjThrottle;
    static int32_t lastDiff = 0;
    static int32_t elastDiff = 0;

    /*
     * 清除历史值
     */
    if(clear==1)
    {
    	lastDiff =0;
    	elastDiff = 0;
    	return 0;
    }

    /*
     * 计算差值
     */
    diffRpm = expRpm - realRpm;

    /*
     * 限定差值最大范围，防止突变
     */
    if(diffRpm > DIFF_RPM_UPSCALE)
        diffRpm = DIFF_RPM_UPSCALE;
    else if(diffRpm < DIFF_RPM_DWSCALE)
        diffRpm = DIFF_RPM_DWSCALE;
    else;

    /*
     * 增量式PID，输出(油门/刹车)调节量
     */
    adjThrottle = kp*(diffRpm - lastDiff) + ki*diffRpm + ku*(diffRpm - 2*lastDiff + elastDiff);

    elastDiff = lastDiff;
    lastDiff = diffRpm;

    /*
     * 限制调节量最大范围，防止发生急加/急刹
     */
    if(adjThrottle > ADJ_THROTTLE_UPSCALE)
        adjThrottle = ADJ_THROTTLE_UPSCALE;
    else if(adjThrottle < ADJ_THROTTLE_DWSCALE)
        adjThrottle = ADJ_THROTTLE_DWSCALE;
    else;

    return adjThrottle;
}

uint8_t MotoSetErrorCode(uint8_t code)
{
    g_fbData.ErrorCode = code;
}

void MotoSetMotoSel(enum motoSel sel)
{
	m_motoCtrl.MotoSel = sel;
}
enum motoSel MotoGetMotoSel()
{
	return m_motoCtrl.MotoSel;
}
void MotoSetControlMode(enum motoMode mode)
{
	m_motoCtrl.ControlMode = mode;
}
enum motoMode MotoGetControlMode()
{
	return m_motoCtrl.ControlMode;
}
void MotoSetGear(enum motoGear gear)
{
	m_motoCtrl.Gear = gear;
}
enum motoGear MotoGetGear()
{
	return m_motoCtrl.Gear;
}
void MotoSetThrottle(uint8_t thr)
{
	m_motoCtrl.Throttle = thr;
	if(MotoGetCarMode() == Manual)
	{
		Clock_start(clockThrottle);
	}
}
uint8_t MotoGetThrottle()
{
	return m_motoCtrl.Throttle;
}
void MotoSetGoalRPM(uint16_t rpm)
{
	m_motoCtrl.GoalRPM = rpm;
}
uint16_t MotoGetGoalRPM()
{
	return m_motoCtrl.GoalRPM;
}

void MotoSetPidOn(uint8_t on)
{
	m_motoCtrl.PidOn = on;
}

uint8_t MotoGetPidOn()
{
	return m_motoCtrl.PidOn;
}

uint16_t MotoGetCircles()
{
    return recvCircle;
}

uint16_t MotoGetRpm()
{
    return g_fbData.recvRPM;
}

/*
 * 返回速度 (m/s)
 */
float MotoGetSpeed()
{
	float speed = (float)MotoGetRpm() / WHEEL_SPEED_RATIO * (sysParam.wheelPerimeter / 1000.0);

	return MotoGetGear() == GEAR_REVERSE ? -speed : speed;
}

/*
 * 速度单位(m/s)
 * 转速单位(圈/min)
 */
uint16_t RPMfromSpeed(float speed)
{
	return  (uint16_t) (speed * WHEEL_SPEED_RATIO / (sysParam.wheelPerimeter / 1000.0) * 60.0);
}

/*
 * 速度单位(m/s)
 * 转速单位(圈/min)
 */
float SpeedfromRPM(uint16_t rpm)
{
	return (float)rpm / 60.0 / WHEEL_SPEED_RATIO * (sysParam.wheelPerimeter / 1000.0);
}

uint8_t MotoGetCarMode()
{
    return g_fbData.mode;
}

/*
 * 返回车辆在轨道上的距离
 */
uint32_t MotoGetCarDistance()
{
	return m_distance;
}

/*
 * 设置车辆在轨道上的绝对位置
 */
void MotoSetCarDistance(uint32_t dist)
{
    m_distance = dist;
    lastDistance = dist;
}

void MotoUpdateCalibrationPoint(rfidPoint_t * calib)
{
	uint8_t size;
	uint8_t i;
	size = vector_size(calib);
	vector_free(calibrationQueue);
	calibrationQueue =0;
	for(i=0;i<size;i++)
	{
		vector_push_back(calibrationQueue,calib[i]);
	}
}

void MotoSetAdjustKAP(float kp)
{
    kap = kp;
}

void MotoSetAdjustKAI(float ki)
{
	kai = ki;
}


void MotoSetKP(float value)
{
    kp = value;
}

void MotoSetKI(float value)
{
    ki = value;
}
