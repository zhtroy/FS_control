#include "Message/Message.h"
#include <stdint.h>
#include <string.h>
#include "FreeRTOS_CLI.h"


BaseType_t prvMessageSend( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
BaseType_t prvMessageStatus( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

extern int32_t autoStrtol(const char *str);
uint8_t strSplitToData(const char *pcParameter, uint8_t *data);


const CLI_Command_Definition_t xMessageSend =
{
	"msgSend",
	"\r\n\r\n=========LTE Command===========\r\n \
msgSend <type> <data>:Send Message. \r\n \
<type>: 0-rfid,1-mmradar,2-uart,3-timer,\r\n \
        4-sonicradar,5-cell,6-Empty \r\n \
<data>: Message Data Array\r\n \
ie, msgSend 1 1,2,3,4\r\n",
	prvMessageSend, /* The function to run. */
	2 /* Three parameters are expected, which can take any value. */
};

const CLI_Command_Definition_t xMessageStatus =
{
	"msgStatus",
	"\r\n\r\n=========Message Command===========\r\n \
msgStatus:return message queue status. \r\n ",
	prvMessageStatus, /* The function to run. */
	0 /* Three parameters are expected, which can take any value. */
};

BaseType_t prvMessageStatus( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static UBaseType_t uxParameterNumber = 0;
    static uint16_t usValue = 0;

    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Command Process*/
    memset( pcWriteBuffer, 0x00, xWriteBufferLen );

	sprintf(pcWriteBuffer,"[Message Queue] free:%d\t posted:%d \tfreeQ zero times:%d\n",
									Message_getFreeNum(), Message_getPostedNum(),Message_getFreeQzerotimes());

	xReturn = pdFALSE;


    return xReturn;
}

BaseType_t prvMessageSend( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static UBaseType_t uxParameterNumber = 0;
    p_msg_t sendmsg;
    static uint8_t dataType;
    char strArray[64];
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( uxParameterNumber == 0 )
	{	
        /* Command Process*/
		uxParameterNumber = 1U;
        memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		xReturn = pdPASS;
	}
    else 
    {
        /* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							uxParameterNumber,		/* Return the next parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);
        
        if(uxParameterNumber == 1)     /*Type */
        {
            dataType = autoStrtol(pcParameter);
            uxParameterNumber = 2;
            // xReturn = pdFALSE;
            xReturn = pdPASS;
        }
        else if(uxParameterNumber == 2)
        {
            sendmsg = Message_getEmpty();
            sendmsg->type = (msg_type_t)dataType;
            strcpy(strArray,pcParameter);
            sendmsg->dataLen = strSplitToData(strArray,sendmsg->data);
            Message_post(sendmsg);
            uxParameterNumber = 0;
            xReturn = pdFALSE;
        }
    }
       
	return xReturn;
}

