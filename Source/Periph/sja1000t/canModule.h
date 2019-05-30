#ifndef __CAN_MODULE__
#define __CAN_MODULE__

#ifdef __cplusplus
extern "C"{
#endif

/* 文件头 */
#include "common.h"
#include "soc_C6748.h"
#include "fpga_periph_def.h"
/* 宏定义 */
#define ZMD_SJA1000_MODULE 1

#define NELEMENTS(array)		/* number of elements in an array */ \
		(sizeof (array) / sizeof ((array) [0]))
		
/*调试信息控制*/
#define CAN_DEBUG_LOG(type,fmt,arg1,arg2,arg3,arg4,arg5) do{\
			FSZX_DEBUG_LOG(type,"\r0:can:%d\n",arg1, 0, 0, 0, 0, 0); \
			FSZX_DEBUG_LOG(type,"\r1:%s(%d):"fmt"\n",__FUNCTION__,__LINE__,arg2,arg3,arg4,arg5);  \
	}while(0)
/*目前支持的CAN波特率*/
#define	BAUDTATE_20K        (0)            /*0 ByteRate_20k*/  /*Not Supoort*/           
#define	BAUDTATE_40K        (1)            /*1 ByteRate_40k*/           
#define	BAUDTATE_50K        (2)            /*2 ByteRate_50k*/           
#define	BAUDTATE_80K        (3)            /*3 ByteRate_80k*/           
#define	BAUDTATE_100K       (4)            /*4 ByteRate_100k*/          
#define	BAUDTATE_125K       (5)            /*5 ByteRate_125k*/          
#define	BAUDTATE_200K       (6)            /*6 ByteRate_200k*/          
#define	BAUDTATE_250K       (7)            /*7 ByteRate_250k*/          
#define	BAUDTATE_400K       (8)            /*8 ByteRate_400k*/          
#define	BAUDTATE_500K       (9)            /*9 ByteRate_500k*/          
#define	BAUDTATE_666K       (10)           /*a ByteRate_666k*/          
#define	BAUDTATE_800K       (11)           /*b ByteRate_800k*/          
#define	BAUDTATE_1000K      (12)           /*c ByteRate_1000k:NBT=6TQ*/ 

/*CAN模块IO控制*/
#define  CAN_BAUD_SET		(0)/*波特率设置*/
#define  CAN_BAUD_GET		(1)/*获取波特率*/
#define  CAN_ACCCODE		(2)/*滤波设置*/
#define  CAN_ACCMASK		(3)/*滤波掩码设置*/
#define  CAN_FILER			(4)/*滤波方式设置*/
#define	 CAN_MODE			(5)/*CAN(sja1000独有)模式选择(basicCAN peliCAN)*/
#define  CAN_FRAM_MODE		(6)/*CAN帧模式(标准帧 扩展帧)*/
#define  CAN_DEVINFO_GET	(7)/*can模块信息打印*/
#define  CAN_DEBUG			(8)/*can模块信息打印级别控制*/
#define  CAN_RESET			(9)/*CAN芯片硬件复位*/
#define  CAN_BUS_STATUS		(10)/*can总线状态*/

/*CAN滤波方式宏定义*/
#define FILTER_DUAL          (0<<16)        /*双滤波*/
#define FILTER_SINGLE        (1<<16)        /*单滤波*/

/*CAN工作方式宏定义。用于open函数中的flags参数配置*/
#define OPERATION_NORMAL     0        /*正常模式*/
#define OPERATION_RDONLY     1        /*只听模式*/
#define OPERATION_LOOPBACK   2        /*自测模式*/

/*CAN帧格式定义*/
#define CAN_FRAME_TYPE_STD	 (0)	/*标准帧*/
#define CAN_FRAME_TYPE_EXT	 (1)	/*扩展帧*/
#define CAN_FRAME_TYPE_DATA	 (0)	/*数据帧*/
#define CAN_FRAME_TYPE_RTR	 (1)	/*远程帧*/

/*CAN中断类型定义*/
#define RIE			(1<<0) 			  /*接收中断*/
#define TIE			(1<<1)			  /*发送中断*/	

/*设备状态*/
#define CAN_DEV_OPENED			(0xffff)
#define CAN_DEV_CLOSED			(0)

#define CAN_INT_STATUS_ADDR         FPGA_CAN_INT
#define CAN_RESET_ADDR              FPGA_CAN_RESET
#define CAN_INT_ENABLE_ADDR         FPGA_CAN_INT_ENB
#define CAN_INT_MASK_ADDR           FPGA_CAN_INT_MASK


#define CanHardIntEnable() \
    (*(volatile uint16_t *) (CAN_INT_ENABLE_ADDR) = 1)
    
#define CanHardIntDisable() \
    (*(volatile uint16_t *) (CAN_INT_ENABLE_ADDR) = 0)

#define CanHardIntMaskAll() \
    (*(volatile uint16_t *) (CAN_INT_MASK_ADDR) = 0xff)
    
#define CanHardIntUnMaskAll() \
    (*(volatile uint16_t *) (CAN_INT_MASK_ADDR) = 0)

#define CanGetHardIntStatus() \
    (~(*(volatile uint16_t *) (CAN_INT_STATUS_ADDR)))


/*typedef*/
/* 函数指针 */
typedef int32_t (*canOpenFuncPtr_t)(void *canObj);	

typedef int32_t (*canWriteFuncPtr_t)(void *canObj,void * canData);

typedef int32_t (*canReadFuncPtr_t)(void *canObj,void * canData);

typedef int32_t (*canCloseFuncPtr_t)(void *canObj);

typedef int32_t (*canCtrlFuncPtr_t)(void *canObj,uint8_t funcNo,uint32_t *arg);

typedef void (*canIsrFuncPtr_t)(int32_t arg);

typedef void (*hwIsrFuncPtr_t)(int32_t arg,int32_t event);

#pragma pack(1)/*考虑字节对齐操作*/

/*can诊断项定义*/
typedef struct _can_diagnose
{
	uint32_t   nIntCount;	/*产生的中断次数*/
	uint32_t   nIntRecvCount;	/*产生的接收中断次数*/
	uint32_t   nIntSendCount;	/*产生的发送中断次数*/
	uint32_t   nWriteCount;	/*应用调用写的次数*/
	uint32_t   nReadCount;   /*应用调用读的次数*/
	uint32_t   nBEICount; /*总线错误中断次数*/
	uint32_t   nEICount;	/*错误报警中断次数*/
	uint32_t   nDOICount;	/*数据溢出中断次数*/
	uint32_t   nEPICount;	/*错误消极中断次数*/
	uint32_t   nALIICount;	/*仲裁丢失中断次数*/
	uint8_t    nALErrValue;  /*最后一次仲裁丢失的错误代码值*/
	uint8_t    nErrValue;  /*最后一次的错误代码值*/	
	uint8_t    nTxErrValue;  /*发送错误计数*/
	uint8_t    nRxErrValue;  /*接收错误计数*/		
} canDiagnose_t;

/*can数据结构定义*/
typedef struct _can_data_obj
{
	uint32_t  ID;/*报文ID*/
	uint8_t SendType;/*发送帧类型，=0时为正常发送，=1时为单次发送，=2时为自发自收，
	                        =3时为单次自发自收，只在此帧为发送帧时有意*/
	uint8_t RemoteFlag;/*是否是远程帧；1/远程帧 0/非远程帧*/
	uint8_t ExternFlag;/*是否是扩展帧,1/扩展帧 0/标准帧*/
	uint8_t DataLen;/*数据长度(<=8)，即Data 的长度*/
	int8_t  Data[8];/*报文的数据；*/
}canDataObj_t;

 typedef struct _hw_int_params 
{
	uint32_t eventId;/*中断事件号*/
	uint32_t intTxRx;/*CAN接收发送中断使能*/
	hwIsrFuncPtr_t irqCall;/*中断回调*/
	int32_t  arg;/*中断回调函数参数*/
}canIntTable_t;

typedef struct _hw_can_params 
{
	uint32_t addr_index;/*SJA1000 EMIF总线基地址 ECAN表示模块内索引*/
	uint32_t width_gpioArray;/*SJA1000 EMIF总线位宽 ECAN使用那组GPIO*/
	uint32_t frameMode;/*CAN控制器的帧模式 对于sja1000来说帧模块还包括滤波方式 高16位存滤波方式 低16位存帧模式*/
	uint32_t acr;	/*验收屏蔽寄存器*/
	uint32_t amr;	/*验收掩码寄存器*/
	uint32_t rate;/*波特率设置*/
}canParamsTable_t;

typedef struct _hw_can_params_int 
{
	canParamsTable_t can_params_table;/*CAN参数表*/
	canIntTable_t    can_int_cfg_table;/*CAN中断配置表*/
	canDiagnose_t	can_diagnose_table;/*CAN诊断表*/
}canParamsIntTable_t;

typedef struct _hw_can_cfg 
{
	uint8_t*      	 		hwName;			/*设备名*/
	uint8_t 				devsNum;		/*设备序列号*/
	canParamsIntTable_t 	hwCfg;			/*设备资源信息*/
}canCfgTable_t;

typedef struct _can_func
{
	uint8_t*       	    hwName;			/*设备名*/
	canOpenFuncPtr_t  	hwCanOpen;		/**< chip CAN open*/
	canWriteFuncPtr_t 	hwCanWrite;		/**< chip CAN write*/
	canReadFuncPtr_t  	hwCanRead;		/**< chip CAN read*/
	canCloseFuncPtr_t 	hwCanClose;		/**< chip CAN close*/
	canCtrlFuncPtr_t  	hwCanCtl;		/**< chip CAN ctrl*/
    canIsrFuncPtr_t       hwcanisr;       /**< chip CAN ISR*/
}canFuncTable_t;

typedef struct _can_dev
{
	uint32_t devStatus;		/*设备状态*/
	uint8_t	devsNum;	/*设备序列号*/
	uint8_t   hwName[20];	/*设备名 NOTE:设备名长度不能超过20*/
	uint32_t drvIndex;		/*设备驱动索引*/
	uint32_t paramIndex;	/*设备资源索引*/
}canDevTable_t;
#pragma pack()

/*CAN API接口定义*/
int32_t CanTableInit(void);
int32_t CanOpen(uint8_t devsNum,hwIsrFuncPtr_t canIsrCall,int32_t arg);
int32_t CanClose(uint8_t devsNum);
int32_t CanWrite(uint8_t devsNum,canDataObj_t * canData);
int32_t CanRead(uint8_t devsNum,canDataObj_t * canData);
int32_t CanIoCtl(uint8_t devsNum,uint8_t funcNo,uint32_t *arg);
void CanIsr(int32_t devsNum);
void CanHardIntMask(uint8_t devsNum);
void CanHardIntUnmask(uint8_t devsNum);


#ifdef __cplusplus
}
#endif

#endif
