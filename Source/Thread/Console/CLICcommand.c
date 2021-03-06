
/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"


extern const CLI_Command_Definition_t xMemeoryWrite;
extern const CLI_Command_Definition_t xMemeoryRead;
extern const CLI_Command_Definition_t xResetCPU;
extern const CLI_Command_Definition_t xLTESend;
extern const CLI_Command_Definition_t xLTESenda;
extern const CLI_Command_Definition_t xLTESetMode;
extern const CLI_Command_Definition_t xLTEGetMode;
extern const CLI_Command_Definition_t xMessageSend;
extern const CLI_Command_Definition_t xSpeedSet;

/*-----------------------------------------------------------*/

void vRegisterSampleCLICommands( void )
{
	/* Register all the command line commands defined immediately above. */
    FreeRTOS_CLIRegisterCommand( &xResetCPU );
    FreeRTOS_CLIRegisterCommand( &xMemeoryWrite );
    FreeRTOS_CLIRegisterCommand( &xMemeoryRead );
    FreeRTOS_CLIRegisterCommand( &xLTESend );
    FreeRTOS_CLIRegisterCommand( &xLTESenda );
    FreeRTOS_CLIRegisterCommand( &xLTESetMode );
    FreeRTOS_CLIRegisterCommand( &xLTEGetMode );
    FreeRTOS_CLIRegisterCommand( &xMessageSend );
    FreeRTOS_CLIRegisterCommand( &xSpeedSet );
}
/*-----------------------------------------------------------*/

