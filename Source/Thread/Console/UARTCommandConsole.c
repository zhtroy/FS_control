/* Standard includes. */
#include "string.h"
#include "stdio.h"
#include <stdint.h>

/* SysBios includes. */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/runtime/Error.h>

#include "CLICommon.h"
#include "FreeRTOS_CLI.h"

/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE		50
#define cmdMAX_HISTORY 16
/* DEL acts as a backspace. */
#define cmdASCII_DEL		( 0x7F )
#define cmdASCII_CTRL_C     (0x03)
#define INVALID_COMBKEY (0)
#define COMBKEY_UPARROW (1)
#define COMBKEY_DOWNARROW (2)
#define COMBKEY_LEFTARROW (3)
#define COMBKEY_RIGHTARROW (4)

//typedef struct {
//    int8_t charArray[3];
//    uint8_t keyIndex;
//} combKey_t;
//
//static combKey_t combKeyTable[] = {
//    {{27,'[','A'},COMBKEY_UPARROW},
//    {{27,'[','B'},COMBKEY_DOWNARROW},
//    {{27,'[','C'},COMBKEY_LEFTARROW},
//    {{27,'[','D'},COMBKEY_RIGHTARROW}
//};



/*-----------------------------------------------------------*/

/*
 * The task that implements the command console processing.
 */
static void prvUARTCommandConsoleTask( void *pvParameters );
//void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority );
extern void vRegisterSampleCLICommands( void );
/*-----------------------------------------------------------*/

/* Const messages output by the command console. */
static const char * const pcWelcomeMessage = \
"\r\n             飞梭智行设备有限公司                    \
 \r\n            FeiSuoZhiXing Equipment Co., Ltd         \
 \r\n-----------------------------------------------------\
 \r\n--- OS: SYS/BIOS         \
 \r\n--- System Clock: 456MHz \
 \r\n--- Memory: DDR2-156MHz  \
 \r\n--- Boot: NandFlash    \
 \r\n--- Software: v1.0   \
 \r\n--- HardWare: v.10   \
 \r\n--- FPGA: v.10   \
 \r\n--- Date: 2018-12-13   \
 \r\n\r\n>";

static const char * const pcNewLine = "\r\n";
static const char * const pcEndOfOutputMessage = "\r\n>";
static const char * const pcDelchar = "\b \b";

/* Used to guard access to the UART in case messages are sent to the UART from
more than one task. */
/*static Semaphore_Handle xTxMutex = NULL;*/
 
/*-----------------------------------------------------------*/

void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority )
{
//    Semaphore_Params semParams;
    Error_Block eb;
    Task_Params taskParams;
    Task_Handle taskHdl;

    vRegisterSampleCLICommands();
	/* Create the semaphore used to access the UART Tx. */
//    Semaphore_Params_init(&semParams);
//    semParams.mode = Semaphore_Mode_BINARY;
//    xTxMutex = Semaphore_create(1, &semParams, NULL);
//	configASSERT( xTxMutex );

    
    Error_init(&eb);
    Task_Params_init(&taskParams);
	taskParams.priority = uxPriority;
	taskParams.stackSize = usStackSize;
	/* Create that task that handles the console itself. */
    taskHdl = Task_create((ti_sysbios_knl_Task_FuncPtr)prvUARTCommandConsoleTask, &taskParams, &eb);
	configASSERT( taskHdl );
}
/*-----------------------------------------------------------*/

static void prvUARTCommandConsoleTask( void *pvParameters )
{
	unsigned char cRxedChar;
	uint8_t ucInputIndex = 0;
	uint8_t ucHisIndex = 0;
	uint8_t ucHisCnt = 0;
	uint8_t ucHisPos = 0;
	uint8_t i = 0;
	uint8_t cCombKeyFlag = 0;
	uint8_t cCombKeyIndex = 0;
	uint8_t cCombKey = INVALID_COMBKEY;
	unsigned char cCombKeyArray[2];

	char *pcOutputString; 
	static char cInputString[ cmdMAX_INPUT_SIZE ], cLastInputString[cmdMAX_HISTORY][ cmdMAX_INPUT_SIZE ];
	BaseType_t xReturned;

	( void ) pvParameters;

	/* Obtain the address of the output buffer.  Note there is no mutual
	exclusion on this buffer as it is assumed only one command console interface
	will be used at any one time. */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	/* Initialise the UART. */
	//xPort = xSerialPortInitMinimal( configCLI_BAUD_RATE, cmdQUEUE_LENGTH );

	/* Send the welcome message. */
	vSerialPutString((char * ) pcWelcomeMessage, ( unsigned short ) strlen( pcWelcomeMessage ) );

	for( ;; )
	{
		/* Wait for the next character.  The while loop is used in case
		INCLUDE_vTaskSuspend is not set to 1 - in which case portMAX_DELAY will
		be a genuine block time rather than an infinite block time. */
		//while( xSerialGetChar( xPort, &cRxedChar, portMAX_DELAY ) != pdPASS );
		while(xSerialGetChar(&cRxedChar,BIOS_WAIT_FOREVER) != TRUE);

        /* Echo the character back. */
        if( cRxedChar == 27 )
        {
            cCombKeyFlag = 1;
            cCombKeyIndex = 0;
            continue;
        }
        else if (cCombKeyFlag == 1)
        {
            if(cCombKeyIndex == 0)
            {
                cCombKeyArray[cCombKeyIndex] = cRxedChar;
                cCombKeyIndex = 1;
                continue;
            }
            else
            {
                cCombKeyArray[cCombKeyIndex] = cRxedChar;
                cCombKeyIndex = 0;
                cCombKeyFlag = 0;
                if(cCombKeyArray[0] == '[' && cCombKeyArray[1] == 'A')
                    cCombKey = COMBKEY_UPARROW;
                else if(cCombKeyArray[0] == '[' && cCombKeyArray[1] == 'B')
                    cCombKey = COMBKEY_DOWNARROW;
                else 
                {
                    cCombKey = INVALID_COMBKEY;
                    continue;
                }
            } 
        }
        else
            cCombKey = INVALID_COMBKEY;
		/* Ensure exclusive access to the UART Tx. */
        
//		if( Semaphore_pend( xTxMutex, BIOS_WAIT_FOREVER ) == pdPASS )
        if(1)
		{
            /*取消CTRL-C导致的复位，防止由于串口接收错误导致的误复位*/
#if 0
            if(cRxedChar == cmdASCII_CTRL_C)    /*CTRL-C:rst command*/
            {   
                memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );
                ucInputIndex = 3;    
                strncpy(cInputString,"rst",3);
                cRxedChar = '\r';
            }
            else;
#endif
            
            if(cCombKey == COMBKEY_UPARROW || cCombKey == COMBKEY_DOWNARROW)
            {
                
                if(cCombKey == COMBKEY_UPARROW && ucHisCnt < cmdMAX_HISTORY)
                    ucHisCnt++;
                else if(cCombKey == COMBKEY_DOWNARROW && ucHisCnt > 1)
                    ucHisCnt--;
                else;
                        
                if(ucHisIndex >= ucHisCnt)
                    ucHisPos = ucHisIndex - ucHisCnt;
                else
                    ucHisPos = cmdMAX_HISTORY + ucHisIndex - ucHisCnt;

                for(i = 0;i<ucInputIndex;i++)
                {
                    vSerialPutString((char * ) pcDelchar, ( unsigned short ) strlen( pcDelchar ) );
                }
                
				memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );

                strcpy( cInputString, cLastInputString[ucHisPos] );
                ucInputIndex = strlen( cInputString );
                vSerialPutString((char * ) cInputString,ucInputIndex);
               
            }
			else if( cRxedChar == '\n' || cRxedChar == '\r' )
			{
				/* Just to space the output from the input. */
				vSerialPutString((char * ) pcNewLine, ( unsigned short ) strlen( pcNewLine ) );

				/* Pass the received command to the command interpreter.  The
				command interpreter is called repeatedly until it returns
				pdFALSE	(indicating there is no more output) as it might
				generate more than one string. */
				if(ucInputIndex != 0)
				{
					do
					{
						/* Get the next output string from the command interpreter. */
						xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

						/* Write the generated string to the UART. */
						vSerialPutString( (char * ) pcOutputString, ( unsigned short ) strlen( pcOutputString ) );

					} while( xReturned != pdFALSE );

					strcpy( cLastInputString[ucHisIndex],cInputString);
					ucHisIndex = (ucHisIndex+1)%cmdMAX_HISTORY;
					ucHisCnt = 0;

					ucInputIndex = 0;
					memset( cInputString, 0x00, cmdMAX_INPUT_SIZE );
					memset( pcOutputString, 0x00, configCOMMAND_INT_MAX_OUTPUT_SIZE );
				}
				else;

                vSerialPutString((char * ) pcEndOfOutputMessage, ( unsigned short ) strlen( pcEndOfOutputMessage ) );
			}
			else
			{
				if( ( cRxedChar == '\b' ) || ( cRxedChar == cmdASCII_DEL ) )
				{
					/* Backspace was pressed.  Erase the last character in the
					string - if any. */
					if( ucInputIndex > 0 )
					{
                        vSerialPutString((char * ) pcDelchar, ( unsigned short ) strlen( pcDelchar ) );
						ucInputIndex--;
						cInputString[ ucInputIndex ] = '\0';
					}
				}
				else
				{
					/* A character was entered.  Add it to the string entered so
					far.  When a \n is entered the complete	string will be
					passed to the command interpreter. */
					if( ( cRxedChar >= ' ' ) && ( cRxedChar <= '~' ) )
					{
                        //xSerialPutChar( cRxedChar);
						vSerialPutString((char *)&cRxedChar, 1 );

						if( ucInputIndex < cmdMAX_INPUT_SIZE )
						{
							cInputString[ ucInputIndex ] = cRxedChar;
							ucInputIndex++;
						}
					}
				}
			}

			/* Must ensure to give the mutex back. */
//			Semaphore_post( xTxMutex );
		}
	}
}


