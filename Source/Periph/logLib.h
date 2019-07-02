#ifndef _LOG_LIB_H
#define _LOG_LIB_H
#ifdef __cplusplus
extern "C" {
#endif



#define LOG_TASK_PRIORITY (5)       /* log任务的优先级 */
#define LOG_TASK_STACK_SIZE  (4096) /* log任务占用的栈大小 */

#ifndef OK
#define OK (0)
#endif

#ifndef ERROR
#define ERROR (-1)
#endif

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#if 0
/* 类型定义 */
typedef struct				    /* log消息 */
{
    char *	fmt;			    /* 格式化字符串指针 */
    int		arg [MAX_LOGARGS];	/* 格式化字符串的参数 */
} logMsg_t;
#endif


/* 函数声明 */
int LogInit(void);
int LogMsg(const char *fmt, ...);
void LogPuts(char *txBuffer, int numBytesToWrite);
void LogPrintf(char *fmt, ...);

#define LogDebug(fmt,...)  LogMsg("%s(%d)-<%s>: "##fmt, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif
