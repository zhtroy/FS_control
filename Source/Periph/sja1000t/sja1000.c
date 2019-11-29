#include "sja1000.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "emifa_app.h"

/************************************************************************
 *定义80M的晶体下，CAN总线波特率*
 ************************************************************************/
/*
sja1000波特率计算公式:
p:can总线波特率
f:外部晶振频率
SJW:BTR0高两位
BRP:BTR0低六位
TSEG2:BTR1第四五六位
TSEG1:BTR1第零到三位
p=f/(2*(SJW+TSEG1+TSEG2+3)*BRP)
*/
int8_t sja1000_timing_table_sample[13][2]=
{
	{0x53,0x2F},          /*0 ByteRate_20k*/  /*Not Supoort*/
    {0x31,0x7A},          /*1 ByteRate_40k*/
    {0x67,0x7A},          /*2 ByteRate_50k*/
    {0x18,0x7A},          /*3 ByteRate_80k*/
    {0x13,0x6B},          /*4 ByteRate_100k*/
    {0x0F,0x7A},          /*5 ByteRate_125k*/
    {0x09,0x7A},          /*6 ByteRate_200k*/
    {0x07,0x7A},          /*7 ByteRate_250k*/
    {0x44,0x7A},          /*8 ByteRate_400k*/
    {0x83,0x7A},          /*9 ByteRate_500k*/
    {0x02,0x7A},          /*a ByteRate_666k*/
    {0x04,0x34},          /*b ByteRate_800k*/
    {0x01,0x7A}          /*c ByteRate_1000k:NBT=6TQ*/
};


/*****************************************************************************
 * 函数名称: void sja1000RegWrite(sja1000Obj_t * sja100Obj,uint32_t addr,int8_t data)
 * 函数说明: 寄存器写操作
 * 输入参数:
 *          sja100Obj:sja1000设备数据结构体
 *			addr:操作的寄存器地址
 *			data:需要操作的数据
 * 输出参数: 无
 * 返 回 值: void
 * 备注: sja1000读写操作接口 目前DSP的EMIF接口是不能满足CAN的时序，必须借助逻辑实现
 *		由于存在读写操作的过程中中断打断的操作从而可能导致对CAN的操作失败，因此软件
 *		按照正常的EMIF时序进行访问 逻辑去解析地址线和数据线 然后按照CAN的时序转发出去
*****************************************************************************/
void sja1000RegWrite(sja1000Obj_t * sja100Obj,uint32_t addr,int8_t data)
{
	sja1000Params_t *sja1000_param=(sja1000Params_t *)&sja100Obj->sja1000_params_table;
	//xIoWrite(sja1000_param->sja1000Baseaddr ,addr,data);
	EMIFAWriteWord(sja1000_param->sja1000Baseaddr, addr,data);
}

/*****************************************************************************
 * 函数名称: void sja1000RegRead(sja1000Obj_t * sja100Obj,uint32_t addr,int8_t *data)
 * 函数说明: 寄存器读操作
 * 输入参数:
 *          sja100Obj:sja1000设备数据结构体
 *			addr:操作的寄存器地址
 *			data:需要操作的数据
 * 输出参数: 无
 * 返 回 值: void
 * 备注: sja1000读写操作接口 目前DSP的EMIF接口是不能满足CAN的时序，必须借助逻辑实现
 *		由于存在读写操作的过程中中断打断的操作从而可能导致对CAN的操作失败，因此软件
 *		按照正常的EMIF时序进行访问 逻辑去解析地址线和数据线 然后按照CAN的时序转发出去
*****************************************************************************/
void sja1000RegRead(sja1000Obj_t * sja100Obj,uint32_t addr,int8_t *data)
{
	sja1000Params_t *sja1000_param=(sja1000Params_t *)&sja100Obj->sja1000_params_table;
	//xIoRead(sja1000_param->sja1000Baseaddr, addr,data);
    *data = EMIFAReadWord(sja1000_param->sja1000Baseaddr, addr);
}

/*****************************************************************************
 * 函数名称: int32_t sja1000IsExit(sja1000Obj_t * sja1000Obj)
 * 函数说明: sja1000测试寄存器操作 主要目的是测试sja1000设备读写访问时序是否正常
 * 输入参数:
 *          sja100Obj:sja1000设备数据结构体
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
static int32_t sja1000IsExit(sja1000Obj_t * sja1000Obj)
{
	sja1000Reg_t* sja1000Regs = (sja1000Reg_t*)SJA1000_MODULE_BASE_ADDR;
	int8_t regValue;
	uint32_t regAddr;
	regAddr = (uint32_t)&sja1000Regs->regs_p.test_p;
	sja1000RegWrite(sja1000Obj,regAddr,0xAA);
	//asm(" NOP 8");
	sja1000RegRead(sja1000Obj,regAddr,&regValue);
	if(regValue!=(int8_t)0xAA)
	{
		return -1;
	}
	sja1000RegWrite(sja1000Obj,regAddr,0x55);
	//asm(" NOP 8");
	sja1000RegRead(sja1000Obj,regAddr,&regValue);
	if(regValue!=(int8_t)0x55)
	{
		return -1;
	}
	return 0;
}

/*****************************************************************************
 * 函数名称: int32_t sja1000EnterReset(sja1000Obj_t * sja1000Obj)
 * 函数说明: sja1000进入复位模式
 * 输入参数:
 *          sja100Obj:sja1000设备数据结构体
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
static int32_t sja1000EnterReset(sja1000Obj_t * sja1000Obj)
{
	sja1000Reg_t* sja1000Regs = (sja1000Reg_t*)SJA1000_MODULE_BASE_ADDR;
	uint32_t i;
	int8_t regValue;
	uint32_t regAddr;
	regAddr = (uint32_t)&sja1000Regs->regs_p.mode_p;
	sja1000RegRead(sja1000Obj,regAddr,&regValue);
	regValue |= 1;
	sja1000RegWrite(sja1000Obj,regAddr,regValue);
	for(i=0;i<0xffff;i++);
	sja1000RegRead(sja1000Obj,regAddr,&regValue);
	if((regValue&0x1) == 0x1)
	{
		return 0;
	}
	return -1;
}

/*****************************************************************************
 * 函数名称: int32_t sja1000ExitReset(sja1000Obj_t * sja1000Obj)
 * 函数说明: sja1000退出复位模式
 * 输入参数:
 *          sja100Obj:sja1000设备数据结构体
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
static int32_t sja1000ExitReset(sja1000Obj_t * sja1000Obj)
{
	sja1000Reg_t* sja1000Regs = (sja1000Reg_t*)SJA1000_MODULE_BASE_ADDR;
	uint32_t i;
	int8_t regValue;
	uint32_t regAddr;
	regAddr = (uint32_t)&sja1000Regs->regs_p.mode_p;
	sja1000RegRead(sja1000Obj,regAddr,&regValue);
	regValue &= ~1;
	sja1000RegWrite(sja1000Obj,regAddr,regValue);
	for(i=0;i<0xffff;i++);
	sja1000RegRead(sja1000Obj,regAddr,&regValue);
	if((regValue&0x1) != 0x1)
	{
		return 0;
	}
	return -1;
}

/*****************************************************************************
 * 函数名称: void sja1000Isr(int32_t arg)
 * 函数说明: sja1000中断接口 (需要注意的是此处可能存在多个中断事件挂接在一个中断服务函数)
 * 输入参数:
 *          sja100Obj:sja1000设备数据结构体
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
void sja1000Isr(int32_t arg)
{
	sja1000Obj_t *sja1000Obj=(sja1000Obj_t *)arg;
	sja1000Reg_t* sja1000Regs = (sja1000Reg_t*)SJA1000_MODULE_BASE_ADDR;
	int8_t regValue;
	sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.interrupt_p,&regValue);
	sja1000Obj->sja1000_diagnose_table.nIntCount++;
	if((regValue&0x1)==0x1)/*接收中断*/
	{
		sja1000Obj->sja1000_diagnose_table.nIntRecvCount++;
		if(sja1000Obj->sja1000_int_table.irqCall!=NULL)
		{
			sja1000Obj->sja1000_int_table.irqCall(sja1000Obj->sja1000_int_table.arg,0x1);
		}
	}
	if((regValue&0x2)==0x2)/*发送中断*/
	{
		sja1000Obj->sja1000_diagnose_table.nIntSendCount++;
		if(sja1000Obj->sja1000_int_table.irqCall!=NULL)
		{
			sja1000Obj->sja1000_int_table.irqCall(sja1000Obj->sja1000_int_table.arg,0x2);
		}
	}
	if((regValue&0x4)==0x4)/*错误报警中断*/
	{
		sja1000Obj->sja1000_diagnose_table.nEICount++;
		#if 0
		/*进入复位工作模式:开启总线*/
		if(sja1000EnterReset(sja1000Obj)!=0)
		{
			SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000进入复位模式失败",DEV_ENTER_RESET_FAIL,0,0,0,0);
		}		
		/*设置发送错误计数器*/
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.txerr_p,0x0);	
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.rxerr_p,0x0);	
		/*退出复位工作模式*/
		if(sja1000ExitReset(sja1000Obj)!=0)
		{
			SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000退出复位模式失败",DEV_EXIT_RESET_FAIL,0,0,0,0);
		}	
		#endif
	}
	if((regValue&0x8)==0x8)/*数据溢出中断*/
	{
		sja1000Obj->sja1000_diagnose_table.nDOICount++;
	}
	if((regValue&0x20)==0x20) /*错误消极中断*/
	{
		sja1000Obj->sja1000_diagnose_table.nEPICount++;
	}
	if((regValue&0x40)==0x40)/*仲裁丢失中断*/
	{
		sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.alc_p,&regValue);
		sja1000Obj->sja1000_diagnose_table.nALErrValue=regValue;
		sja1000Obj->sja1000_diagnose_table.nALIICount++;
	}
	if((regValue&0x80)==0x80)/*总线错误中断*/
	{
		sja1000Obj->sja1000_diagnose_table.nBEICount++;
		sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.ecc_p,&regValue);
		sja1000Obj->sja1000_diagnose_table.nErrValue=regValue;
		#if 0
		/*进入复位工作模式:开启总线*/
		if(sja1000EnterReset(sja1000Obj)!=0)
		{
			SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000进入复位模式失败",DEV_ENTER_RESET_FAIL,0,0,0,0);
		}		
		/*设置发送错误计数器*/
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.txerr_p,0x0);	
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.rxerr_p,0x0);	
		/*退出复位工作模式*/
		if(sja1000ExitReset(sja1000Obj)!=0)
		{
			SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000退出复位模式失败",DEV_EXIT_RESET_FAIL,0,0,0,0,0);
		}	
		#endif
	}
}

/*****************************************************************************
 * 函数名称: int32_t sja1000Open(void * obj)
 * 函数说明: sja1000初始化函数
 * 输入参数:
 *          obj:sja1000设备数据结构体
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
int32_t sja1000Open(void * obj)
{
	sja1000Obj_t *sja1000Obj=(sja1000Obj_t *)obj;
	sja1000Reg_t* sja1000Regs = (sja1000Reg_t*)SJA1000_MODULE_BASE_ADDR;
	int8_t *acr,*amr;
	int8_t regValue;
	uint32_t index=0;
	uint32_t tmp=0;
	if(sja1000Obj == NULL)
	{
		SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000设备输入结构体为空",DEV_HAND_NULL,0,0,0,0);
		return DEV_HAND_NULL;
	}
	/*上电复位CAN控制器*/
	//sja1000HardWareReset(sja1000Obj);
	
	/*检查sja1000是否存在*/
	if(sja1000IsExit(sja1000Obj)!=0)
	{
		SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000测试访问出错,请检查sja1000访问时序",DEV_NO_EXIST,0,0,0,0);
		return DEV_NO_EXIST;
	}
	/*进入复位工作模式*/
	if(sja1000EnterReset(sja1000Obj)!=0)
	{
		SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000进入复位模式失败",DEV_ENTER_RESET_FAIL,0,0,0,0);
		return DEV_ENTER_RESET_FAIL;
	}
	/*设置工作于peli or basic 模式并且只激活tx0和rx0*/
	sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.cdr_p,0xc0);
	/*初始化定时器既波特率*/
	if((sja1000Obj->sja1000_params_table.bandRate > BAUDTATE_20K) && (sja1000Obj->sja1000_params_table.bandRate<= BAUDTATE_1000K))
	{
		index=sja1000Obj->sja1000_params_table.bandRate;
	}
	else
	{
		index=BAUDTATE_20K;
		sja1000Obj->sja1000_params_table.bandRate=BAUDTATE_20K;
		SJA1000_DEBUG_LOG(DEBUG_WARNING,"波特率%d不支持,默认配置成20K",sja1000Obj->sja1000_params_table.bandRate,0,0,0,0);
	}

	sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.btr0_p,sja1000_timing_table_sample[index][0]);
	sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.btr1_p,sja1000_timing_table_sample[index][1]);
	/*初始化滤波器acr*/
	/*按照大端方式存储高16bit(低4bit未用)用于标准帧，32bit(低2bit未用)用于扩展帧*/
	if((sja1000Obj->sja1000_params_table.frameMode&0xffff)==CAN_FRAME_TYPE_STD)
	{
		if((sja1000Obj->sja1000_params_table.frameMode&0xffff0000)==FILTER_SINGLE)
		{
			tmp=sja1000Obj->sja1000_params_table.acr<<5;
			acr=(int8_t*)&(tmp);	
			/*单滤波*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx0_rx0_acr0_p,acr[1]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,acr[0]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,0x0);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,0x0);
		}
		else
		{
			tmp=(sja1000Obj->sja1000_params_table.acr&0xffff)<<5;
			acr=(int8_t*)&(tmp);
			/*双滤波*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx0_rx0_acr0_p,acr[1]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,acr[0]);
			tmp=((sja1000Obj->sja1000_params_table.acr&0xffff0000)>>16)<<5;
			acr=(int8_t*)&(tmp);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,acr[1]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,acr[0]);
		}
	}
	else
	{	
		if((sja1000Obj->sja1000_params_table.frameMode&0xffff0000)==FILTER_SINGLE)
		{
			tmp=sja1000Obj->sja1000_params_table.acr<<3;
			acr=(int8_t*)&(tmp);	
			/*单滤波*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx0_rx0_acr0_p,acr[3]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,acr[2]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,acr[1]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,acr[0]);
		}
		else
		{
			tmp=(sja1000Obj->sja1000_params_table.acr&0xffff);
			acr=(int8_t*)&(tmp);	
			/*扩展帧的单滤波可以过滤整个ID 双滤波只能过滤高16位的ID*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx0_rx0_acr0_p,acr[1]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,acr[0]);
			tmp=((sja1000Obj->sja1000_params_table.acr&0xffff0000)>>16);
			acr=(int8_t*)&(tmp);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,acr[1]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,acr[0]);
		}
	}
	/*初始化滤波器amr*/
	if((sja1000Obj->sja1000_params_table.frameMode&0xffff)==CAN_FRAME_TYPE_STD)
	{
		if((sja1000Obj->sja1000_params_table.frameMode&0xffff0000)==FILTER_SINGLE)
		{
			if(sja1000Obj->sja1000_params_table.amr!=0xffffffff)
			{
				tmp=sja1000Obj->sja1000_params_table.amr<<5;
			}
			else
			{
				tmp=sja1000Obj->sja1000_params_table.amr;
			}
			amr=(int8_t*)&(tmp);
			/*单滤波*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx4_rx4_amr0_p,amr[1]);/*默认只接收屏蔽寄存器中的值*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx5_rx5_amr1_p,amr[0]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx6_rx6_amr2_p,0xff);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx7_rx7_amr3_p,0xff);
		}
		else
		{
			if(sja1000Obj->sja1000_params_table.amr!=0xffffffff)
			{
				tmp=(sja1000Obj->sja1000_params_table.amr&0xffff)<<5;
			}
			else
			{
				tmp=sja1000Obj->sja1000_params_table.amr;
			}
			amr=(int8_t*)&(tmp);
			/*双滤波*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx4_rx4_amr0_p,amr[1]);/*默认只接收屏蔽寄存器*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx5_rx5_amr1_p,amr[0]|0xf);
			if(sja1000Obj->sja1000_params_table.amr!=0xffffffff)
			{
				tmp=((sja1000Obj->sja1000_params_table.amr&0xffff0000)>>16)<<5;
			}
			else
			{
				tmp=sja1000Obj->sja1000_params_table.amr;
			}
			amr=(int8_t*)&(tmp);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx6_rx6_amr2_p,amr[1]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx7_rx7_amr3_p,amr[0]|0xf);
		}
		
	}
	else
	{		
		if((sja1000Obj->sja1000_params_table.frameMode&0xffff0000)==FILTER_SINGLE)
		{
			if(sja1000Obj->sja1000_params_table.amr!=0xffffffff)
			{
				tmp=sja1000Obj->sja1000_params_table.amr<<3;
			}
			else
			{
				tmp=sja1000Obj->sja1000_params_table.amr;
			}
			amr=(int8_t*)&(tmp);
			/*单滤波*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx4_rx4_amr0_p,amr[3]);/*默认只接收屏蔽寄存器中的值*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx5_rx5_amr1_p,amr[2]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx6_rx6_amr2_p,amr[1]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx7_rx7_amr3_p,amr[0]);
		}
		else
		{
			if(sja1000Obj->sja1000_params_table.amr!=0xffffffff)
			{
				tmp=(sja1000Obj->sja1000_params_table.amr&0xffff);
			}
			else
			{
				tmp=sja1000Obj->sja1000_params_table.amr;
			}
			amr=(int8_t*)&(tmp);
			/*扩展帧的单滤波可以过滤整个ID 双滤波只能过滤高16位的ID*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx4_rx4_amr0_p,amr[1]);/*默认只接收屏蔽寄存器中的值*/
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx5_rx5_amr1_p,amr[0]);
			if(sja1000Obj->sja1000_params_table.amr!=0xffffffff)
			{
				tmp=((sja1000Obj->sja1000_params_table.amr&0xffff0000)>>16);
			}
			else
			{
				tmp=sja1000Obj->sja1000_params_table.amr;
			}
			amr=(int8_t*)&(tmp);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx6_rx6_amr2_p,amr[3]);
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx7_rx7_amr3_p,amr[2]);
		}
	}
	sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.ocr_p,0x1a);
	/*接收基地址*/
	sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.rxr_p,0x00);
	/*设置模式*/
	/*检查滤波方式*/
	if((sja1000Obj->sja1000_params_table.frameMode&0xffff0000)==FILTER_SINGLE)
	{
		/*单滤波*/
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.mode_p,(1<<3|1));
	}
	else
	{
		/*双滤波*/
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.mode_p,1);
	}
	/*清中断*/
	sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.interrupt_p,&regValue);
	/*初始化中断*/
	/* |1<<0 接收中断 |1<<1 发送中断 |1<<2 错误报警中断 |1<<3 数据溢出中断
	   |1<<4 唤醒中断 |1<<5 错误消极中断 |1<<6 仲裁丢失中断 |1<<7 总线错误中断*/
	regValue = sja1000Obj->sja1000_int_table.intTxRx|1<<2|1<<3|1<<4|1<<5|1<<6|1<<7;
	sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.intenable_p,regValue); 
    sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.intenable_p,&regValue);
	/*释放报文缓冲,CMD寄存器钟RRB位*/
	sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.command_p,0x04);
	if(sja1000ExitReset(sja1000Obj)!=0)
	{
		SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000退出复位模式失败",DEV_EXIT_RESET_FAIL,0,0,0,0);
		return DEV_EXIT_RESET_FAIL;
	}	
    
	return DEV_OK;
}

/*****************************************************************************
 * 函数名称: int32_t sja1000Close(void * obj)
 * 函数说明: sja1000关闭
 * 输入参数:
 *          obj:sja1000设备数据结构体
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
int32_t sja1000Close(void * obj)
{
	sja1000Obj_t *sja1000Obj=(sja1000Obj_t *)obj;
	if(sja1000Obj == NULL)
	{
		SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000设备输入结构体为空",0,0,0,0,0);
		return DEV_HAND_NULL;
	}
	return DEV_OK;
}

/*****************************************************************************
 * 函数名称: int32_t sja1000State(sja1000Obj_t * sja1000Obj)
 * 函数说明: sja1000状态
 * 输入参数:
 *          obj:sja1000设备数据结构体
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
int32_t sja1000State(sja1000Obj_t * sja1000Obj)
{
	sja1000Reg_t* sja1000Regs = (sja1000Reg_t*)SJA1000_MODULE_BASE_ADDR;	
	int8_t regValue;
//	uint32_t cnt = 0;
	sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.status_p,&regValue);
	if((regValue&0x80) == 0x80)	/*总线关闭:发送错误次数255*/
	{
		/*进入复位工作模式:开启总线*/
		if(sja1000EnterReset(sja1000Obj)!=0)
		{
			SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000进入复位模式失败",DEV_ENTER_RESET_FAIL,0,0,0,0);
			return DEV_ENTER_RESET_FAIL;
		}		
		/*设置发送错误计数器*/
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.txerr_p,0x0);	
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.rxerr_p,0x0);	
		/*退出复位工作模式*/
		if(sja1000ExitReset(sja1000Obj)!=0)
		{
			SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000退出复位模式失败",DEV_EXIT_RESET_FAIL,0,0,0,0);
			return DEV_EXIT_RESET_FAIL;
		}	
	}

	sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.status_p,&regValue);
	if((regValue&0x40) == 0x40)	/*计数器超过阀门值*/
	{
		/*进入复位工作模式:开启总线*/
		if(sja1000EnterReset(sja1000Obj)!=0)
		{
			SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000进入复位模式失败",DEV_ENTER_RESET_FAIL,0,0,0,0);
			return DEV_ENTER_RESET_FAIL;
		
		}		
		/*设置发送错误计数器*/
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.txerr_p,0x0);			
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.rxerr_p,0x0);			
		/*退出复位工作模式*/
		if(sja1000ExitReset(sja1000Obj)!=0)
		{
			SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000退出复位模式失败",DEV_EXIT_RESET_FAIL,0,0,0,0);
			return DEV_EXIT_RESET_FAIL;
		}	
	}

	return 0;
}

/*****************************************************************************
 * 函数名称: int32_t sja1000Write(void * obj,void * data)
 * 函数说明: sja1000发送函数
 * 输入参数:
 *          obj:sja1000设备数据结构体
 *			data:发送的数据
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
int32_t sja1000Write(void * obj,void * data)
{
	sja1000Obj_t *sja1000Obj=(sja1000Obj_t *)obj;
	sja1000DataObj_t *sja1000Data=(sja1000DataObj_t *)data;
	sja1000Reg_t* sja1000Regs = (sja1000Reg_t*)SJA1000_MODULE_BASE_ADDR;
	uint32_t i;
	int8_t regValue;
//	uint32_t timeOut=0;
	uint32_t idTmp=0;
	if(sja1000Obj == NULL)
	{
		SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000设备输入结构体为空",DEV_HAND_NULL,0,0,0,0);
		return DEV_HAND_NULL;
	}
	/*检查发送缓冲区状态是否锁定*/
	sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.status_p,&regValue);
	if((regValue&0x04) != 0x04)
	{
		SJA1000_DEBUG_LOG(DEBUG_WARNING,"发送缓冲区被锁定",DEV_SEND_BUSY,0,0,0,0);
		return DEV_SEND_BUSY;
	}
	if(sja1000Data->DataLen > 8)
	{
	   sja1000Data->DataLen = 0x8;
	}
	regValue = sja1000Data->DataLen;
	if(sja1000Data->RemoteFlag != 0) /*remote frame*/
	{
		regValue|=0x40;
	}		
	if(sja1000Data->ExternFlag != 0) /*extened 帧*/
	{
		regValue|=0x80; 
	}
	sja1000Obj->sja1000_diagnose_table.nWriteCount++;
	/*帧信息canAddr=0x10*/
	sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx0_rx0_acr0_p,regValue);
	if(sja1000Data->ExternFlag != 0)
	{
		/*识别码canAddr=0x11\0x12\0x13\0x14*/
		idTmp= sja1000Data->ID << 3;		/*低3bit无效*/
		regValue = (idTmp)&0xFF;	
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx4_rx4_amr0_p,regValue);  /*ID4~ID0\X\X\X*/
		regValue = (idTmp>>8)&0xFF;
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,regValue);  /*ID12~ID5*/
		regValue = (idTmp>>16)&0xFF;
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,regValue);  /*ID20~ID13*/
		regValue = (idTmp>>24)&0xFF;
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,regValue);  /*ID28~ID21*/
		for(i=0;i<sja1000Data->DataLen;i++)
		{
			/*发送数据canAddr=0x15~0x1f*/
			sja1000RegWrite(sja1000Obj,i+(uint32_t)&sja1000Regs->regs_p.tx5_rx5_amr1_p,sja1000Data->Data[i]);  
		}
	}
	else
	{
		/*识别码canAddr=0x11\0x12*/
		idTmp = sja1000Data->ID << 5;	/*低5bit无效*/
		regValue = (idTmp)&0xFF;
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,regValue);	/*ID20~ID18\X\X\X\X\X*/
		regValue = (idTmp>>8)&0xFF;	
		sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,regValue);	/*ID28~ID21*/
		for(i=0;i<sja1000Data->DataLen;i++)
		{
			/*发送数据canAddr=0x13~0x1d*/
			sja1000RegWrite(sja1000Obj,i+(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,sja1000Data->Data[i]);  
		}
	}
	switch(sja1000Data->SendType)
	{
		case 0: /*正常发送*/
			regValue = 0x01;
			break;
		case 1:  /*单次发送*/
			regValue = 0x03;
			break;
		case 2: /*自发自收*/
			regValue = 0x10;
			break;
		case 3:  /*单次自发自收*/
			regValue = 0x12;
			break;
		default:
			SJA1000_DEBUG_LOG(DEBUG_ERROR,"发送模式未知",DEV_SEBD_TYPE_ERROR,0,0,0,0);
			return DEV_SEBD_TYPE_ERROR;
	}		
	sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.command_p,regValue);
	/*等待发送完成*/
    /*
	do
	{
		sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.status_p,&regValue);
		moduleDelayUs(100,20);
		timeOut++;
		if(timeOut>SJA1000_WRITE_TIME_OUT)
		{
			SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000发送超时",DEV_SEND_TIME_OUT,0,0,0,0,0);
			return DEV_SEND_TIME_OUT;
		}
	}while((regValue&0x08) != 0x08);
	*/
	return sja1000Data->DataLen;
}

/*****************************************************************************
 * 函数名称: int32_t sja1000Read(void * obj,void * data)
 * 函得? sja1000接收函数
 * 输入参数:
 *          obj:sja1000设备数据结构体
 *			data:接收的数据
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
int32_t sja1000Read(void * obj,void * data)
{
	sja1000Obj_t *sja1000Obj=(sja1000Obj_t *)obj;
	sja1000DataObj_t *sja1000Data=(sja1000DataObj_t *)data;
	sja1000Reg_t* sja1000Regs = (sja1000Reg_t*)SJA1000_MODULE_BASE_ADDR;
	uint32_t i;
	int8_t regValue;
	int32_t regValue1,regValue2;
	if(sja1000Obj == NULL)
	{
		SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000设备输入结构体为空",DEV_HAND_NULL,0,0,0,0);
		return DEV_HAND_NULL;
	}
	/*检查接收缓冲区中是否有信息*/
	sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.status_p,&regValue);
	if((regValue&0x01) != 0x01)
	{
		SJA1000_DEBUG_LOG(DEBUG_WARNING,"缓冲区中没有数据",DEV_REC_FIFO_EMPTY,0,0,0,0);
		return DEV_REC_FIFO_EMPTY;
	}
	sja1000Obj->sja1000_diagnose_table.nReadCount++;
	/*判断帧类型*/
	sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx0_rx0_acr0_p,&regValue); /*帧信息canAddr=0x10*/
	if((regValue &0x80)==0x80)  
	{
		sja1000Data->ExternFlag=CAN_FRAME_TYPE_EXT;	/*扩展帧*/
	}
	else
	{
		sja1000Data->ExternFlag=CAN_FRAME_TYPE_STD;
	}
	if((regValue &0x40)==0x40)  
	{
		sja1000Data->RemoteFlag=CAN_FRAME_TYPE_RTR;	/*远程帧*/
	}
	else
	{
		sja1000Data->RemoteFlag=CAN_FRAME_TYPE_DATA;
	}
	sja1000Data->DataLen = (uint32_t)(regValue & 0xF);
	if(sja1000Data->DataLen > 8)
	{
		sja1000Data->DataLen = 0x8;
	}
	/*接收数据*/
	if(sja1000Data->ExternFlag != 0)
	{
		sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,&regValue);	/*ID28~ID21*/
		regValue1 =regValue&0xff;
		regValue1 <<= 8;
		sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,&regValue); /*ID20~ID13*/
		regValue2=regValue&0xff;
		regValue1|=regValue2;
		regValue1 <<= 8;
		sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,&regValue); /*ID12~ID5*/
		regValue2=regValue&0xff;
		regValue1|=regValue2;
		regValue1 <<= 8;
		sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx4_rx4_amr0_p,&regValue); /*ID20~ID18\X\X\X\X\X*/
		regValue2=regValue&0xff;
		regValue1|=regValue2;
		regValue1 >>= 3;
		sja1000Data->ID = regValue1;
		for(i=0;i<sja1000Data->DataLen;i++)
		{
			sja1000RegRead(sja1000Obj,i+(uint32_t)&sja1000Regs->regs_p.tx5_rx5_amr1_p,&sja1000Data->Data[i]);  
		}
	}
	else
	{
		sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,&regValue);	/*ID28~ID21*/
		regValue1 =regValue&0xff;
		regValue1<<= 8;
		sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,&regValue);	/*ID20~ID18\X\X\X\X\X*/
		regValue2=regValue&0xff;
		regValue1|=regValue2;
		regValue1>>= 5;
		sja1000Data->ID = regValue1;
		for(i=0;i<sja1000Data->DataLen;i++)
		{
			sja1000Data->Data[i] = 0x0;
			sja1000RegRead(sja1000Obj,i+(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,&sja1000Data->Data[i]);  
		}		
	}
	/*释放报文缓冲 使另外一条数据信息立即有效,CMD寄存器钟RRB位*/
	sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.command_p,0x04);
  	return sja1000Data->DataLen;	
}

/*****************************************************************************
 * 函数名称: int32_t sja1000IoCtl(void * obj,uint32_t funcNo,uint32_t *arg)
 * 函数说明: sja1000 IO控制函数
 * 输入参数:
 *          obj:sja1000设备数据结构体
 *			funcNo:控制选项
 *			arg:控制参数
 * 输出参数: 无
 * 返 回 值: 0(成功)/-1(失败)
 * 备注:
*****************************************************************************/
int32_t sja1000IoCtl(void * obj,uint8_t funcNo,uint32_t *arg)
{
	sja1000Obj_t *sja1000Obj=(sja1000Obj_t *)obj;
	sja1000Reg_t* sja1000Regs = (sja1000Reg_t*)SJA1000_MODULE_BASE_ADDR;
	int8_t *acr,*amr;
	int8_t regValue;
	uint32_t tmp=0;
	if(sja1000Obj == NULL)
	{
		SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000设备输入结构体为空",DEV_HAND_NULL,0,0,0,0);
		return DEV_HAND_NULL;
	}
	switch (funcNo)
    {
        case CAN_BAUD_SET: 
			tmp=*arg;
            if ((tmp > BAUDTATE_20K) && (tmp<= BAUDTATE_1000K))
            {
            	/*进入复位工作模式*/
				if(sja1000EnterReset(sja1000Obj)!=0)
				{
					SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000进入复位模式失败",DEV_ENTER_RESET_FAIL,0,0,0,0);
					return DEV_ENTER_RESET_FAIL;
				}
				/*初始化定时器既波特率*/
				sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.btr0_p,sja1000_timing_table_sample[tmp][0]);
				sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.btr1_p,sja1000_timing_table_sample[tmp][1]); 
				if(sja1000ExitReset(sja1000Obj)!=0)
				{
					SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000退出复位模式失败",DEV_EXIT_RESET_FAIL,0,0,0,0);
					return DEV_EXIT_RESET_FAIL;
				}	
            }
            else
            {
            	SJA1000_DEBUG_LOG(DEBUG_ERROR,"不支持的波特率",DEV_BAND_RATE_NONSUPPORT,0,0,0,0);
				return DEV_BAND_RATE_NONSUPPORT;
			}
            break;  
        case CAN_BAUD_GET:
            *arg = sja1000Obj->sja1000_params_table.bandRate;
            break;
        case CAN_ACCCODE:  
        	/*进入复位工作模式*/
			if(sja1000EnterReset(sja1000Obj)!=0)
			{
				SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000进入复位模式失败",DEV_ENTER_RESET_FAIL,0,0,0,0);
				return DEV_ENTER_RESET_FAIL;
			}
        	/*初始化滤波器acr*/
			if((sja1000Obj->sja1000_params_table.frameMode&0xffff)==CAN_FRAME_TYPE_STD)
			{
				
				if((sja1000Obj->sja1000_params_table.frameMode&0xffff0000)==FILTER_SINGLE)
				{
					tmp=(*arg)<<5;
					acr=(int8_t*)&(tmp);
					/*单滤波*/
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx0_rx0_acr0_p,acr[1]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,acr[0]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,0);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,0); 
				}
				else
				{
					tmp=((*arg)&0xffff)<<5;
					acr=(int8_t*)&(tmp);
					/*双滤波*/
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx0_rx0_acr0_p,acr[1]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,acr[0]);
					tmp=(((*arg)&0xffff0000)>>16)<<5;
					acr=(int8_t*)&(tmp);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,acr[1]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,acr[0]); 
				}					 
			}
			else
			{	
				if((sja1000Obj->sja1000_params_table.frameMode&0xffff0000)==FILTER_SINGLE)
				{
					tmp=(*arg)<<3;
					acr=(int8_t*)&(tmp);
					/*单滤波*/
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx0_rx0_acr0_p,acr[3]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,acr[2]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,acr[1]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,acr[0]); 
				}
				else
				{
					tmp=((*arg)&0xffff);
					acr=(int8_t*)&(tmp);
					/*扩展帧的单滤波可以过滤整个ID 双滤波只能过滤高16位的ID*/
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx0_rx0_acr0_p,acr[1]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx1_rx1_acr1_p,acr[0]);
					tmp=(((*arg)&0xffff0000)>>16);
					acr=(int8_t*)&(tmp);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx2_rx2_acr2_p,acr[1]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx3_rx3_acr3_p,acr[0]);  
				}	 
			}
			if(sja1000ExitReset(sja1000Obj)!=0)
			{
				SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000退出复位模式失败",DEV_EXIT_RESET_FAIL,0,0,0,0);
				return DEV_EXIT_RESET_FAIL;
			}
            break;
        case CAN_ACCMASK:
            /*进入复位工作模式*/
			if(sja1000EnterReset(sja1000Obj)!=0)
			{
				SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000进入复位模式失败",DEV_ENTER_RESET_FAIL,0,0,0,0);
				return DEV_ENTER_RESET_FAIL;
			}
			if((sja1000Obj->sja1000_params_table.frameMode&0xffff)==CAN_FRAME_TYPE_STD)
			{	
				if((sja1000Obj->sja1000_params_table.frameMode&0xffff0000)==FILTER_SINGLE)
				{
					if((*arg)!=0xffffffff)
					{
						tmp=(*arg)<<5;
					}
					else
					{
						tmp=(*arg);
					}
					amr=(int8_t*)&(tmp);
					/*单滤波*/
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx4_rx4_amr0_p,amr[1]);/*默认只接收屏蔽寄存器中的值*/
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx5_rx5_amr1_p,amr[0]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx6_rx6_amr2_p,0xff);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx7_rx7_amr3_p,0xff);
				}
				else
				{
					if((*arg)!=0xffffffff)
					{
						tmp=((*arg)&0xffff)<<5;
					}
					else
					{
						tmp=(*arg);
					}
					amr=(int8_t*)&(tmp);
					/*双滤波*/
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx4_rx4_amr0_p,amr[1]);/*默认只接收屏蔽寄存器中的值*/
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx5_rx5_amr1_p,amr[0]|0xf);
					if((*arg)!=0xffffffff)
					{
						tmp=(((*arg)&0xffff0000)>>16)<<5;
					}
					else
					{
						tmp=(*arg);
					}
					amr=(int8_t*)&(tmp);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx6_rx6_amr2_p,amr[1]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx7_rx7_amr3_p,amr[0]|0xf);
				}
			}
			else
			{
				if((sja1000Obj->sja1000_params_table.frameMode&0xffff0000)==FILTER_SINGLE)
				{
					if((*arg)!=0xffffffff)
					{
						tmp=(*arg)<<3;
					}
					else
					{
						tmp=(*arg);
					}
					amr=(int8_t*)&(tmp);
					/*单滤波*/
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx4_rx4_amr0_p,amr[3]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx5_rx5_amr1_p,amr[2]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx6_rx6_amr2_p,amr[1]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx7_rx7_amr3_p,amr[0]); 
				}
				else
				{
					if((*arg)!=0xffffffff)
					{
						tmp=((*arg)&0xffff);
					}
					else
					{
						tmp=(*arg);
					}
					amr=(int8_t*)&(tmp);
					/*扩展帧的单滤波可以过滤整个ID 双滤波只能过滤高16位的ID*/
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx4_rx4_amr0_p,amr[1]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx5_rx5_amr1_p,amr[0]);
					if((*arg)!=0xffffffff)
					{
						tmp=(((*arg)&0xffff0000)>>16);
					}
					else
					{
						tmp=(*arg);
					}
					amr=(int8_t*)&(tmp);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx6_rx6_amr2_p,amr[1]);
					sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.tx7_rx7_amr3_p,amr[0]); 
				}
			}
			if(sja1000ExitReset(sja1000Obj)!=0)
			{
				SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000退出复位模式失败",DEV_EXIT_RESET_FAIL,0,0,0,0);
				return DEV_EXIT_RESET_FAIL;
			}
            break;
        case CAN_FILER:/*SELECT处理*/
			if((*arg)>1)
			{
				SJA1000_DEBUG_LOG(DEBUG_ERROR,"滤波参数错误",DEV_FILTER_PARAM_ERROR,0,0,0,0);
				return DEV_FILTER_PARAM_ERROR;
			}
             /*进入复位工作模式*/
			if(sja1000EnterReset(sja1000Obj)!=0)
			{
				SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000进入复位模式失败",DEV_ENTER_RESET_FAIL,0,0,0,0);
				return DEV_ENTER_RESET_FAIL;
			}
			sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.mode_p,&regValue);
			regValue&=0xf7;/*清空滤波方式设置位*/
			regValue|=(*arg)<<3;
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.mode_p,regValue);
			sja1000Obj->sja1000_params_table.frameMode|=(*arg)<<16;
			if(sja1000ExitReset(sja1000Obj)!=0)
			{
				SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000退出复位模式失败",DEV_EXIT_RESET_FAIL,0,0,0,0);
				return DEV_EXIT_RESET_FAIL;
			}             
            break;
		case CAN_MODE:
			tmp=*arg;
			if(tmp>SJA100_OPERATION_LOOPBACK)
			{
				SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000不支持的工作模式",DEV_WORK_MODE_NONSUPPORT,0,0,0,0);
				return DEV_WORK_MODE_NONSUPPORT;
			}
			if(sja1000EnterReset(sja1000Obj)!=0)
			{
				SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000进入复位模式失败",DEV_ENTER_RESET_FAIL,0,0,0,0);
				return DEV_ENTER_RESET_FAIL;
			}
			sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.mode_p,&regValue);
			regValue|=tmp<<1;
			sja1000RegWrite(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.mode_p,regValue);
			if(sja1000ExitReset(sja1000Obj)!=0)
			{
				SJA1000_DEBUG_LOG(DEBUG_ERROR,"sja1000退出复位模式失败",DEV_EXIT_RESET_FAIL,0,0,0,0);
				return DEV_EXIT_RESET_FAIL;
			} 
			break;
		case CAN_FRAM_MODE:
			tmp=*arg;
			sja1000Obj->sja1000_params_table.frameMode=tmp;
			break;
		case CAN_DEVINFO_GET:
			sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.txerr_p,&regValue);
			sja1000Obj->sja1000_diagnose_table.nTxErrValue=regValue;
			sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.rxerr_p,&regValue);
			sja1000Obj->sja1000_diagnose_table.nRxErrValue=regValue;
			LogMsg("\r中断总和: %d次\n",sja1000Obj->sja1000_diagnose_table.nIntCount);
			LogMsg("\r发送中断: %d次\n",sja1000Obj->sja1000_diagnose_table.nIntSendCount);
			LogMsg("\r接收中断: %d次\n",sja1000Obj->sja1000_diagnose_table.nIntRecvCount);
			LogMsg("\r总线错误: %d次\n",sja1000Obj->sja1000_diagnose_table.nBEICount);
			LogMsg("\r错误报警: %d次\n",sja1000Obj->sja1000_diagnose_table.nEICount);
			LogMsg("\r数据溢出: %d次\n",sja1000Obj->sja1000_diagnose_table.nDOICount);
			LogMsg("\r错误消极: %d次\n",sja1000Obj->sja1000_diagnose_table.nEPICount);
			LogMsg("\r仲裁丢失: %d次\n",sja1000Obj->sja1000_diagnose_table.nALIICount);
			LogMsg("\r发送错误: %d次\n",sja1000Obj->sja1000_diagnose_table.nTxErrValue);
			LogMsg("\r接收错误: %d次\n",sja1000Obj->sja1000_diagnose_table.nRxErrValue);
			LogMsg("\r发送次数: %d次\n",sja1000Obj->sja1000_diagnose_table.nWriteCount);
			LogMsg("\r接收次数: %d次\n",sja1000Obj->sja1000_diagnose_table.nReadCount);
			LogMsg("\r最后一次仲裁丢失值: %d次\n",sja1000Obj->sja1000_diagnose_table.nALErrValue);
			LogMsg("\r最后一次错误代码值: %d次\n",sja1000Obj->sja1000_diagnose_table.nErrValue);
			break;
		case CAN_DEBUG:
			/*sja1000_dbg_level=*arg;*/
			break;
        /*
		case CAN_RESET:
			sja1000HardWareReset(sja1000Obj);
			break;
		*/
		case CAN_BUS_STATUS:
			sja1000RegRead(sja1000Obj,(uint32_t)&sja1000Regs->regs_p.status_p,&regValue);
			*arg=regValue;
			break;
        default:
            break;
    }
    return DEV_OK;
}


