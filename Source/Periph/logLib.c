/*头文件声明*/
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <stdio.h>
#include "uartStdio.h"
#include "logLib.h"

/* 宏定义 */
#define MAILBOX_DEPTH	(64)	           	 	/* 邮箱深度 */
#define MAX_CHAR_NUMS	(256)					/* 单次打印的最大字符数 */

/* 静态全局变量声明 */
static Task_Handle logTaskId = NULL;            /* 任务句柄*/
static Semaphore_Handle logSem = NULL;          /* 信号量句柄*/
static Mailbox_Handle logMailbox = NULL;        /* 邮箱句柄*/
static int logMsgsLost = 0;	                    /* 丢失消息计数*/
static char strArray[MAILBOX_DEPTH][MAX_CHAR_NUMS];
static uint8_t strIndex = 0;

/* 静态函数声明 */
static void LogTask (void);

/*****************************************************************************
* 函数名称: logInit
* 函数说明: 初始化log的任务、邮箱和信号量
* 输入参数: 无
* 输出参数: 无
* 返 回 值: 0(成功)/-1(失败)
* 备注:
*****************************************************************************/
int LogInit(void)
{
    Mailbox_Params mboxParams;
    Task_Params taskParams;
    Semaphore_Params semParams;
    
    if (logTaskId != NULL)
    {
        /*任务已经被初始化了*/
        return (ERROR);         
    }
    
    Mailbox_Params_init(&mboxParams);
    logMailbox = Mailbox_create (sizeof (char *),MAILBOX_DEPTH, &mboxParams, NULL);
    if (logMailbox == NULL)
    {
        /*邮箱创建失败*/
        return (ERROR);
    }

    Task_Params_init(&taskParams);
	taskParams.priority = LOG_TASK_PRIORITY;
	taskParams.stackSize = LOG_TASK_STACK_SIZE;
    logTaskId = Task_create((ti_sysbios_knl_Task_FuncPtr)LogTask, &taskParams, NULL);

    if (logTaskId == NULL)
    {
        /*任务创建失败*/
        return (ERROR);
    }
    
	Semaphore_Params_init(&semParams);
	semParams.mode = Semaphore_Mode_BINARY;
	logSem = Semaphore_create(1, &semParams, NULL);

    if (logSem == NULL)
    {
        /*信用量创建失败*/
        return (ERROR);
    }

    return (OK);
}

/*****************************************************************************
* 函数名称: logMsg
* 函数说明: 初始化log的任务、邮箱和信号量
* 输入参数:
*           fmt:参数格式化方式
*           arg1-arg6:待格式化参数
* 输出参数: 无
* 返 回 值: 消息长度/-1(失败)
* 备注:
*****************************************************************************/
int LogMsg(const char *fmt, ...)
{

	va_list vp;
	char *strAddr;
    va_start(vp,fmt);
    vsprintf(strArray[strIndex],fmt,vp);
    strAddr = strArray[strIndex];

    if(FALSE == Mailbox_post(logMailbox,(Ptr *)&strAddr,BIOS_NO_WAIT))
	{
        /*邮箱已满，消息丢失*/
	    ++logMsgsLost;
	    return (ERROR);
	}

    /* 消息发送成功，字符串数组地址累加 */
    if(strIndex < (MAILBOX_DEPTH-1))
    	strIndex++;
    else
    	strIndex = 0;

    return 0;
}

/*****************************************************************************
* 函数名称: logTask
* 函数说明: 静态函数，log任务回调函数
* 输入参数: 无
*           
* 输出参数: 无
* 返 回 值: 无
* 备注:
*****************************************************************************/
static void LogTask (void)
{
    static int oldMsgsLost;

    int newMsgsLost;	/* used in case logMsgsLost is changed during use */
    int32_t strAddr;

    while(1)
	{
    	if (FALSE == Mailbox_pend (logMailbox, (Ptr *) &strAddr, BIOS_WAIT_FOREVER))
    	{
            /*获取消息失败*/
    		LogPuts ("logTask: error reading log messages.\n",MAX_CHAR_NUMS);
    	}
    	else
    	{
    	    if (strAddr == NULL)
    	    {
                /*无格式化信息*/
    	    	LogPuts ("<null \"fmt\" parameter>\n",MAX_CHAR_NUMS);
    	    }
    	    else
    		{
    		    LogPuts ((char *)strAddr,MAX_CHAR_NUMS);
    		}
        }

    	/* check for any more messages lost */

    	newMsgsLost = logMsgsLost;

    	if (newMsgsLost != oldMsgsLost)
    	{
    		LogPrintf ("logTask: %d log messages lost.\n",
    		     newMsgsLost - oldMsgsLost);

    	    oldMsgsLost = newMsgsLost;
    	}
	}/*end while(1)*/
}


/*****************************************************************************
* 函数名称: void LogPuts(char *txBuffer, int numBytesToWrite)
* 函数说明: 静态函数，通过串口打印字符串
* 输入参数:
*           txBuffer: 发送数据指针
*           numBytesToWrite: 发送数据长度
* 输出参数: 无
* 返 回 值: 无
* 备注:
*****************************************************************************/
void LogPuts(char *txBuffer, int numBytesToWrite)
{
    /*添加信号量锁，保证打印信息的顺序*/
    Semaphore_pend (logSem, BIOS_WAIT_FOREVER);
    UARTPuts(txBuffer, numBytesToWrite);
    Semaphore_post (logSem);
}

/*****************************************************************************
* 函数名称: LogPrintf(char *fmt, ...)
* 函数说明: 静态函数，通过串口打印字符串
* 输入参数:
*           fmt:参数格式化方式
*           ...:待格式化参数
* 输出参数: 无
* 返 回 值: 无
* 备注:
*****************************************************************************/
void LogPrintf(char *fmt, ...)
{
	va_list vp;
	va_start(vp,fmt);
	/*添加信号量锁，保证打印信息的顺序*/
	Semaphore_pend (logSem, BIOS_WAIT_FOREVER);
	UARTprintf(fmt,vp);
	Semaphore_post (logSem);
}
