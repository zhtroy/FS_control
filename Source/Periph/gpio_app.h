/**
 * \file     gpio.h
 *
 * \brief    This file contains the function prototypes for the device
 *           abstraction layer for GPIO and some related macros.
 */

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef      __GPIO_APP_H__
#define      __GPIO_APP_H__

#include "hw_gpio.h"
#ifdef __cplusplus
extern "C" {
#endif

/************* GPIO pin directions.************************************/

/* This is used to configure a GPIO pin as an input pin. */
#define GPIO_DIR_INPUT                1
/* This is used to configure a GPIO pin as an output pin.*/
#define GPIO_DIR_OUTPUT               0

/******************Interrupt Trigger Level Types.**********************/

/* Disable interrupt generation on both the edges of a signal on a pin.*/
#define GPIO_INT_TYPE_NOEDGE          0

/* Enable interrupt generation on falling edge of a signal on a pin.*/
#define GPIO_INT_TYPE_FALLEDGE        1

/* Enable interrupt generation on the rising edge of a signal on a pin.*/
#define GPIO_INT_TYPE_RISEDGE         2

/* Enable interrupt generation on both the edges of a signal on a pin.*/
#define GPIO_INT_TYPE_BOTHEDGE        3

/*****************Interrupt Pending status.*****************************/

/* This signifies interrupt status as cleared.*/
#define GPIO_INT_NOPEND               0

/* This signifies interrupt status as pending.*/
#define GPIO_INT_PEND                 1

/*****************Write values to a pin.********************************/

/* This is used to write a logic 0 to a pin.*/
#define GPIO_PIN_LOW                  0

/* This is used to write a logic 1 to a pin.*/
#define GPIO_PIN_HIGH                 1

#define PINMUX19_GPIO6_0_ENABLE (SYSCFG_PINMUX19_PINMUX19_27_24_GPIO6_0 << SYSCFG_PINMUX19_PINMUX19_27_24_SHIFT)
#define PINMUX13_GPIO6_10_ENABLE (SYSCFG_PINMUX13_PINMUX13_23_20_GPIO6_10 << SYSCFG_PINMUX13_PINMUX13_23_20_SHIFT)
#define PINMUX13_GPIO6_15_ENABLE (SYSCFG_PINMUX13_PINMUX13_3_0_GPIO6_15 << SYSCFG_PINMUX13_PINMUX13_3_0_SHIFT)
#define PINMUX13_GPIO6_14_ENABLE (SYSCFG_PINMUX13_PINMUX13_7_4_GPIO6_14 << SYSCFG_PINMUX13_PINMUX13_7_4_SHIFT)
#define PINMUX13_GPIO6_13_ENABLE (SYSCFG_PINMUX13_PINMUX13_11_8_GPIO6_13 << SYSCFG_PINMUX13_PINMUX13_11_8_SHIFT)
#define PINMUX1_GPIO0_0_ENABLE (SYSCFG_PINMUX1_PINMUX1_31_28_GPIO0_0 << SYSCFG_PINMUX1_PINMUX1_31_28_SHIFT)
#define PINMUX1_GPIO0_1_ENABLE (SYSCFG_PINMUX1_PINMUX1_27_24_GPIO0_1 << SYSCFG_PINMUX1_PINMUX1_27_24_SHIFT)
#define PINMUX1_GPIO0_5_ENABLE (SYSCFG_PINMUX1_PINMUX1_11_8_GPIO0_5 << SYSCFG_PINMUX1_PINMUX1_11_8_SHIFT)
#define PINMUX1_GPIO0_2_ENABLE (SYSCFG_PINMUX1_PINMUX1_23_20_GPIO0_2 << SYSCFG_PINMUX1_PINMUX1_23_20_SHIFT)
#define PINMUX19_GPIO2_15_ENABLE (SYSCFG_PINMUX5_PINMUX5_3_0_GPIO2_15 << SYSCFG_PINMUX5_PINMUX5_3_0_SHIFT)
#define PINMUX13_GPIO8_10_ENABLE (SYSCFG_PINMUX18_PINMUX18_31_28_GPIO8_10 << SYSCFG_PINMUX18_PINMUX18_31_28_SHIFT)

#define GPIO_BANK_NUMS 	(16)
#define GPIO0_0_INDEX	(0*GPIO_BANK_NUMS + 1 + 0)
#define GPIO0_1_INDEX	(0*GPIO_BANK_NUMS + 1 + 1)
#define GPIO0_5_INDEX   (0*GPIO_BANK_NUMS + 1 + 5)
#define GPIO0_2_INDEX   (0*GPIO_BANK_NUMS + 1 + 2)
#define GPIO2_15_INDEX	(2*GPIO_BANK_NUMS + 1 + 15)
#define GPIO6_0_INDEX 	(6*GPIO_BANK_NUMS + 1 + 0)
#define GPIO6_10_INDEX 	(6*GPIO_BANK_NUMS + 1 + 10)
#define GPIO6_13_INDEX  (6*GPIO_BANK_NUMS + 1 + 13)
#define GPIO6_14_INDEX 	(6*GPIO_BANK_NUMS + 1 + 14)
#define GPIO6_15_INDEX 	(6*GPIO_BANK_NUMS + 1 + 15)
#define GPIO8_10_INDEX	(8*GPIO_BANK_NUMS + 1 + 10)

#define GPIO_MPU9250_INT 	GPIO6_10_INDEX
#define GPIO_UART_INT		GPIO0_0_INDEX
#define GPIO_CAN_INT		GPIO0_1_INDEX
#define GPIO_SIGNAL_INT     GPIO0_5_INDEX
#define GPIO_UART485_INT    GPIO0_2_INDEX
#define GPIO_FPGA_RST		GPIO6_15_INDEX
#define GPIO_FPGA_DONE		GPIO8_10_INDEX
#define	GPIO_RUN_LED		GPIO6_0_INDEX
#define GPIO_LTE_RESET      GPIO2_15_INDEX
#define GPIO_EEPROM_WP      GPIO6_13_INDEX




/*****************************************************************************
**                   FUNCTION DECLARATIONS
*****************************************************************************/
void GPIOInit(void);
void GPIOFpgaReset(void);
void GPIOToggleLed(void);
void GPIOWaitFpgaDone(void);
//void GPIOTESTIsr(void);

#ifdef __cplusplus
}
#endif
#endif
















