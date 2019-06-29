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
#define EEPROM_SLV_ADDR (0x50)
BaseType_t prvSetCarID( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
BaseType_t prvGetCarID( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
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
    if(mpu9250ReadBytes(EEPROM_SLV_ADDR,0,2,&usValue) == -1)
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

uint16_t GetLocalCarID()
{
    uint16_t usValue;
    mpu9250ReadBytes(EEPROM_SLV_ADDR,0,2,&usValue);
    return usValue;
}
