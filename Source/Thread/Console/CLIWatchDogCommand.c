/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

extern void stopWatchdogReactivate();
BaseType_t prvResetCPU( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

const CLI_Command_Definition_t xResetCPU =
{
	"rst",
	"\r\nrst/<ctrl-c>:Reset CPU by Watchdog Timer\r\n \
Notes:Wait most 5s.\r\n",
	prvResetCPU, /* The function to run. */
	0 /* Three parameters are expected, which can take any value. */
};

BaseType_t prvResetCPU( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    memset( pcWriteBuffer, 0x00, xWriteBufferLen );
    strncpy( pcWriteBuffer, "\r\nWaiting CPU Reset......\r\n", xWriteBufferLen );
    stopWatchdogReactivate();
    return pdFALSE;
}


