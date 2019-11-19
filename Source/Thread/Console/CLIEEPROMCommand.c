/*
 * CLIS2CCommand.c
 *
 *  Created on: 2019-6-11
 *      Author: DELL
 */

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"
#include "mpu9250/mpu9250_drv.h"
#include <ti/sysbios/knl/Task.h>

BaseType_t prvSetCarID( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
BaseType_t prvGetCarID( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
BaseType_t prvEEPROMClear( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvEEPROMRead( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
BaseType_t prvEEPROMWrite( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
const CLI_Command_Definition_t xSetCarID =
{
    "setID",
    "\r\n Setting Car Nums of Station. \
    ex:setID 0x6001\r\n",
    prvSetCarID, /* The function to run. */
    1
};

const CLI_Command_Definition_t xGetCarID =
{
    "getID",
    "\r\n Setting Station Status. \
    ex:getID\r\n",
    prvGetCarID, /* The function to run. */
    0
};

const CLI_Command_Definition_t xEEPROMWrite =
{
    "ewr",
    "\r\n\r\n=========EEPROM Command===========\r\n \
ewr <addr> <data>:EEPROM Write \r\n \
<addr>: register address,\r\n \
<data>: Data Array\r\n \
ie, ewr 0 1,2,3,4\r\n",
    prvEEPROMWrite, /* The function to run. */
    2 /* Three parameters are expected, which can take any value. */
};



const CLI_Command_Definition_t xEEPROMClear =
{
    "eclr",
    "\r\n\r\n=========EEPROM Command===========\r\n \
eclr <addr> <len>:EEPROM Write \r\n \
<addr>: register address,\r\n \
<len>: Length\r\n \
ie, eclr 0 100\r\n",
    prvEEPROMClear, /* The function to run. */
    2 /* Three parameters are expected, which can take any value. */
};

const CLI_Command_Definition_t xEEPROMRead =
{
    "erd",
    "\r\n\r\n=========EEPROM Command===========\r\n \
erd <addr> <len>:EEPROM Write \r\n \
<addr>: register address,\r\n \
<len>: Length\r\n \
ie, erd 0 100\r\n",
    prvEEPROMRead, /* The function to run. */
    2 /* Three parameters are expected, which can take any value. */
};

BaseType_t prvSetCarID( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
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
                            pcCommandString,        /* The command string itself. */
                            uxParameterNumber,      /* Return the next parameter. */
                            &xParameterStringLength /* Store the parameter string length. */
                        );

        if(uxParameterNumber == 1)
        {
            usValue = autoStrtol(pcParameter);
            memset( pcWriteBuffer, 0x00, xWriteBufferLen );
            if(mpu9250WriteBytes(EEPROM_SLV_ADDR,0,2,&usValue) == -1)
            {

                strncpy( pcWriteBuffer, "\r\nCar ID Set Failed\r\n", xWriteBufferLen );
            }
            else
            {
                strncpy( pcWriteBuffer, "\r\nCar ID Set Success\r\n", xWriteBufferLen );
            }
            xReturn = pdFALSE;
        }
    }

    return xReturn;
}

BaseType_t prvGetCarID( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
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
    if(mpu9250ReadBytes(EEPROM_SLV_ADDR,128,2,&usValue) == -1)
    {
        strncpy( pcWriteBuffer, "\r\nCar ID Get Failed\r\n", xWriteBufferLen );
    }
    else
    {
        sprintf(pcWriteBuffer,"%x",usValue);
    }
    xReturn = pdFALSE;


    return xReturn;
}

BaseType_t prvEEPROMClear( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static UBaseType_t uxParameterNumber = 0;
    static uint8_t ucAddr = 0;
    static uint8_t ucLen = 0;
    uint8_t len;
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
                            pcCommandString,        /* The command string itself. */
                            uxParameterNumber,      /* Return the next parameter. */
                            &xParameterStringLength /* Store the parameter string length. */
                        );

        if(uxParameterNumber == 1) /*Address */
        {
            uxParameterNumber++;
            ucAddr = autoStrtol(pcParameter);
            xReturn = pdPASS;
        }
        else if(uxParameterNumber == 2) /*Length */
        {
            ucLen = autoStrtol(pcParameter);
            memset( pcWriteBuffer, 0x00, xWriteBufferLen );

            while(ucLen > 0)
            {
                len = (ucAddr/8 + 1)*8 - ucAddr;
                if(len <= ucLen)
                {
                    ucLen -= len;

                }
                else
                {
                    len = ucLen;
                    ucLen = 0;
                }
                mpu9250WriteBytes(EEPROM_SLV_ADDR,ucAddr,len,pcWriteBuffer);
                Task_sleep(10);
                ucAddr += len;
            }

            uxParameterNumber = 0;
            xReturn = pdFALSE;
        }
    }

    return xReturn;
}

BaseType_t prvEEPROMWrite( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static UBaseType_t uxParameterNumber = 0;
    static uint8_t ucAddr;
    static uint8_t ucLen;
    char strArray[256];
    static uint8_t ucData[128];
    uint8_t len;
    uint8_t *pData;
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
                            pcCommandString,        /* The command string itself. */
                            uxParameterNumber,      /* Return the next parameter. */
                            &xParameterStringLength /* Store the parameter string length. */
                        );

        if(uxParameterNumber == 1)     /*Type */
        {
            ucAddr = autoStrtol(pcParameter);
            uxParameterNumber = 2;
            xReturn = pdPASS;
        }
        else if(uxParameterNumber == 2)
        {
            strcpy(strArray,pcParameter);
            ucLen = strSplitToData(strArray,ucData);
            pData = ucData;
            while(ucLen > 0)
            {
                len = (ucAddr/8 + 1)*8 - ucAddr;
                if(len <= ucLen)
                {
                    ucLen -= len;
                }
                else
                {
                    len = ucLen;
                    ucLen = 0;
                }
                mpu9250WriteBytes(EEPROM_SLV_ADDR,ucAddr,len,pData);
                pData+=len;
                ucAddr += len;
                Task_sleep(10);
            }

            uxParameterNumber = 0;
            xReturn = pdFALSE;
        }
    }

    return xReturn;
}

BaseType_t prvEEPROMRead( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
    const char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static UBaseType_t uxParameterNumber = 0;
    static uint8_t ucAddr = 0;
    static uint8_t ucLen = 0;
    static uint8_t ucValue[128];
    uint8_t i;
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
                            pcCommandString,        /* The command string itself. */
                            uxParameterNumber,      /* Return the next parameter. */
                            &xParameterStringLength /* Store the parameter string length. */
                        );

        if(uxParameterNumber == 1) /*Address */
        {
            uxParameterNumber++;
            ucAddr = autoStrtol(pcParameter);
            xReturn = pdPASS;
        }
        else if(uxParameterNumber == 2) /*Value */
        {
            //usLen = strtol(pcParameter, NULL, 16);
            ucLen = autoStrtol(pcParameter);

            memset( pcWriteBuffer, 0x00, xWriteBufferLen );
            mpu9250ReadBytes(EEPROM_SLV_ADDR,ucAddr,ucLen,ucValue);

            for(i=0;i<ucLen;i++)
            {
                memset( pcStr, 0x00, 16 );
                if((i%(16)) == 0)
                {
                    sprintf(pcStr,"\r\n0x%02x: ",ucAddr+i);
                    strncat(pcWriteBuffer,pcStr,strlen(pcStr));
                }
                else;

                sprintf(pcStr,"0x%02x ",ucValue[i]);

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

uint16_t GetLocalCarID()
{
    uint16_t usValue;
    mpu9250ReadBytes(EEPROM_SLV_ADDR,0,2,&usValue);
    return usValue;
}
