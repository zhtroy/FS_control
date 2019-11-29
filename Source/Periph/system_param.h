/*
 * system_param.h
 *
 *  Created on: 2019-6-30
 *      Author: DELL
 */

#ifndef SYSTEM_PARAM_H_
#define SYSTEM_PARAM_H_


/*
 * EEPROM参数
 */
#pragma pack(1)
typedef struct{
    uint16_t carID;
    uint8_t brakeDirection;
    uint8_t brakeOffset;
    uint8_t maxtThrottle;
    uint8_t brakeReverseRatio;
    uint32_t wheelPerimeter;  //车轮周长(mm)
    uint32_t encoderWheelPerimeter; //编码器测距轮周长(0.1mm)
    uint8_t changeRailDirection;  //变轨方向
    uint8_t idleBrake;    //车辆在idle状态下的固定刹车量
    /*
     * 车速控制参数
     */
	float KI;
	float KP;
	float KU;
	//跟车参数
    float KSP;
    float KSI;
    float KAP;
    float KAI;
    //停站流程使用的pid参数
    float KP_station;
    float KI_station;

    //站台zigbee地址
    uint16_t station_addr;

    //急停刹车量
    uint8_t forcebrake;

}systemParameter_t;
#pragma pack()


/*
 * 全局变量
 */
#pragma pack(1)
typedef struct parameter_tag{
	uint32_t buildNumber;

	/*路线是否循环*/
    uint8_t cycleRoute;

}parameter_t;
#pragma pack()




#define EEPROM_PHY_ADDR (0x50)

//ROM参数
extern systemParameter_t g_sysParam;
//一些全局变量
extern parameter_t g_var;

//void ParmInit();
void ParamSendBack();
uint8_t ParamWriteToEEPROM();
void ParamInit();
#endif /* SYSTEM_PARAM_H_ */
