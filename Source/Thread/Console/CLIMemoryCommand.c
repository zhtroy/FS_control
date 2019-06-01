/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

#define BYTE_TYPE 1
#define WORD_TYPE 2
#define DWORD_TYPE 4

extern int32_t ioRead(int32_t baseAddr,int32_t offset,int8_t bytes);
extern void ioWrite(int32_t baseAddr,int32_t offset,int32_t value,int8_t bytes);
extern int32_t autoStrtol(const char *str);

BaseType_t prvMemroyWrite( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
BaseType_t prvMemroyRead( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

const CLI_Command_Definition_t xMemeoryWrite =
{
	"mwr",
	"\r\n\r\n=========Memory Command========\r\n \
mwr <type> <addr> <value>:Memory Write\r\n \
<type>:-l(long),-s(short),-c(char)\r\n \
<addr>: Address\r\n \
<value>: Value\r\n \
ie,mwr -s 0x60000000 0x01\r\n",
	prvMemroyWrite, /* The function to run. */
	3 /* Three parameters are expected, which can take any value. */
};

const CLI_Command_Definition_t xMemeoryRead =
{
	"mrd",
	"\r\n \
mrd <type> <addr> <len>:Memory Read\r\n \
<type>:-l(long),-s(short),-c(char)\r\n \
<addr>: Address\r\n \
<len> : Length\r\n \
ie,mrd -s 0x60000000 100\r\n",
	prvMemroyRead, /* The function to run. */
	3 /* Three parameters are expected, which can take any value. */
};


BaseType_t prvMemroyWrite( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static UBaseType_t uxParameterNumber = 0;
    static uint8_t ucType = 0;
    static uint32_t ulAddr = 0;
    static uint32_t ulValue = 0;

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
            uxParameterNumber++;
            if(xParameterStringLength == 2)
            { 
                if(strncmp(pcParameter,"-l",( size_t ) xParameterStringLength) == 0)
                    ucType = DWORD_TYPE;     //32Bits(Dwords)
                else if(strncmp(pcParameter,"-s",( size_t ) xParameterStringLength) == 0)
                    ucType = WORD_TYPE;     //16Bits(Words)
                else if(strncmp(pcParameter,"-c",( size_t ) xParameterStringLength) == 0)
                    ucType = BYTE_TYPE;     //8Bits(Byte)
                else
                    xReturn = pdFALSE;
            }
            else
                xReturn = pdFALSE;
            
            if (xReturn == pdFALSE)
            {
                memset( pcWriteBuffer, 0x00, xWriteBufferLen );
                strncpy( pcWriteBuffer, "\r\nType Error:Please Input Type<-l,-s or -c>\r\n", xWriteBufferLen );
                uxParameterNumber = 0;
            }
            else;
        }
        else if(uxParameterNumber == 2) /*Address */
        {
            uxParameterNumber++;
            ulAddr = autoStrtol(pcParameter);
        }
        else if(uxParameterNumber == 3) /*Value */
        {  
            ulValue = autoStrtol(pcParameter);
            ioWrite(ulAddr,0,ulValue,ucType);

            memset( pcWriteBuffer, 0x00, xWriteBufferLen );
            sprintf( pcWriteBuffer, "0x%08x:%x", ulAddr,ulValue );
            uxParameterNumber = 0;
            xReturn = pdFALSE;
        }
    }
       
	return xReturn;
}

BaseType_t prvMemroyRead( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static UBaseType_t uxParameterNumber = 0;
    static uint8_t ucType = 0;
    static uint32_t ulAddr = 0;
    static uint16_t usLen = 0;
    uint32_t ulValue = 0;
    uint16_t i;
    char strAarry[16];
    char * pcStr = strAarry; 

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( uxParameterNumber == 0 )
	{	
        /* Command Process*/
        memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		uxParameterNumber = 1U;
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
            uxParameterNumber++;
            if(xParameterStringLength == 2)
            { 
                if(strncmp(pcParameter,"-l",( size_t ) xParameterStringLength) == 0)
                    ucType = DWORD_TYPE;    
                else if(strncmp(pcParameter,"-s",( size_t ) xParameterStringLength) == 0)
                    ucType = WORD_TYPE;     
                else if(strncmp(pcParameter,"-c",( size_t ) xParameterStringLength) == 0)
                    ucType = BYTE_TYPE;     
                else
                    xReturn = pdFALSE;
            }
            else
                xReturn = pdFALSE;
            
            if (xReturn == pdFALSE)
            {
                memset( pcWriteBuffer, 0x00, xWriteBufferLen );
                strncpy( pcWriteBuffer, "\r\nType Error:Please Input Type<-l,-s or -c>\r\n", xWriteBufferLen );
                uxParameterNumber = 0;
            }
            else;
            
        }
        else if(uxParameterNumber == 2) /*Address */
        {
            uxParameterNumber++;
            ulAddr = autoStrtol(pcParameter);
        }
        else if(uxParameterNumber == 3) /*Value */
        {  
            //usLen = strtol(pcParameter, NULL, 16);
            usLen = autoStrtol(pcParameter);
            
            memset( pcWriteBuffer, 0x00, xWriteBufferLen );
            
            for(i=0;i<usLen;i++)
            {
                memset( pcStr, 0x00, 16 );
                if((i%(16/ucType)) == 0)
                {
                    sprintf(pcStr,"\r\n0x%08x: ",ulAddr+i*ucType);
                    strncat(pcWriteBuffer,pcStr,strlen(pcStr));
                }
                else;
                
                ulValue = ioRead(ulAddr,i,ucType);
                
                if(ucType == BYTE_TYPE)
                    sprintf(pcStr,"0x%02x ",ulValue);
                else if(ucType == WORD_TYPE)
                    sprintf(pcStr,"0x%04x ",ulValue);
                else if(ucType == DWORD_TYPE)
                    sprintf(pcStr,"0x%08x ",ulValue);
                else;

                if(strlen(pcStr) + strlen(pcWriteBuffer) < configCOMMAND_INT_MAX_OUTPUT_SIZE)
                    strncat(pcWriteBuffer,pcStr,strlen(pcStr));
                else
                    break;
            }            
            
            uxParameterNumber = 0;
            xReturn = pdFALSE;
        }
    }
       
	return xReturn;
}


