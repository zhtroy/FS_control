/*
 * cslr_ectl1.h
 *
 * This file contains the macros for Register Chip Support Library (CSL) which 
 * can be used for operations on the respective underlying hardware/peripheral
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
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

#ifndef _CSLR_ECTL_H_
#define _CSLR_ECTL_H_

#include <cslr/cslr.h>
#include <cslr/tistdtypes.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 REVID;
    volatile Uint32 SOFTRESET;
    volatile Uint8 RSVD0[4];
    volatile Uint32 INTCONTROL;
    volatile Uint32 C0RXTHRESHEN;
    volatile Uint32 C0RXEN;
    volatile Uint32 C0TXEN;
    volatile Uint32 C0MISCEN;
    volatile Uint32 C1RXTHRESHEN;
    volatile Uint32 C1RXEN;
    volatile Uint32 C1TXEN;
    volatile Uint32 C1MISCEN;
    volatile Uint8 RSVD1[16];
    volatile Uint32 C0RXTHRESHSTAT;
    volatile Uint32 C0RXSTAT;
    volatile Uint32 C0TXSTAT;
    volatile Uint32 C0MISCSTAT;
    volatile Uint32 C1RXTHRESHSTAT;
    volatile Uint32 C1RXSTAT;
    volatile Uint32 C1TXSTAT;
    volatile Uint32 C1MISCSTAT;
    volatile Uint32 RSVD;
    volatile Uint8 RSVD2[12];
    volatile Uint32 C0RXIMAX;
    volatile Uint32 C0TXIMAX;
    volatile Uint32 C1RXIMAX;
    volatile Uint32 C1TXIMAX;
} CSL_EctlRegs;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* REVID */

#define CSL_ECTL_REVID_REV_MASK (0xFFFFFFFFu)
#define CSL_ECTL_REVID_REV_SHIFT (0x00000000u)
#define CSL_ECTL_REVID_REV_RESETVAL (0x4EC80100u)

#define CSL_ECTL_REVID_RESETVAL (0x4EC80100u)

/* SOFTRESET */


#define CSL_ECTL_SOFTRESET_RESET_MASK (0x00000001u)
#define CSL_ECTL_SOFTRESET_RESET_SHIFT (0x00000000u)
#define CSL_ECTL_SOFTRESET_RESET_RESETVAL (0x00000000u)

#define CSL_ECTL_SOFTRESET_RESETVAL (0x00000000u)

/* INTCONTROL */

#define CSL_ECTL_INTCONTROL_INTPACEEN_MASK (0x003F0000u)
#define CSL_ECTL_INTCONTROL_INTPACEEN_SHIFT (0x00000010u)
#define CSL_ECTL_INTCONTROL_INTPACEEN_RESETVAL (0x00000000u)

#define CSL_ECTL_INTCONTROL_C2TXPACEEN_MASK (0x00200000u)
#define CSL_ECTL_INTCONTROL_C2TXPACEEN_SHIFT (0x00000015u)
#define CSL_ECTL_INTCONTROL_C2TXPACEEN_RESETVAL (0x00000000u)
/*----C2TXPACEEN Tokens----*/
#define CSL_ECTL_INTCONTROL_C2TXPACEEN_DISABLE (0x00000000u)
#define CSL_ECTL_INTCONTROL_C2TXPACEEN_ENABLE (0x00000001u)

#define CSL_ECTL_INTCONTROL_C2RXPACEEN_MASK (0x00100000u)
#define CSL_ECTL_INTCONTROL_C2RXPACEEN_SHIFT (0x00000014u)
#define CSL_ECTL_INTCONTROL_C2RXPACEEN_RESETVAL (0x00000000u)
/*----C2RXPACEEN Tokens----*/
#define CSL_ECTL_INTCONTROL_C2RXPACEEN_DISABLE (0x00000000u)
#define CSL_ECTL_INTCONTROL_C2RXPACEEN_ENABLE (0x00000001u)

#define CSL_ECTL_INTCONTROL_C1TXPACEEN_MASK (0x00080000u)
#define CSL_ECTL_INTCONTROL_C1TXPACEEN_SHIFT (0x00000013u)
#define CSL_ECTL_INTCONTROL_C1TXPACEEN_RESETVAL (0x00000000u)
/*----C1TXPACEEN Tokens----*/
#define CSL_ECTL_INTCONTROL_C1TXPACEEN_DISABLE (0x00000000u)
#define CSL_ECTL_INTCONTROL_C1TXPACEEN_ENABLE (0x00000001u)

#define CSL_ECTL_INTCONTROL_C1RXPACEEN_MASK (0x00040000u)
#define CSL_ECTL_INTCONTROL_C1RXPACEEN_SHIFT (0x00000012u)
#define CSL_ECTL_INTCONTROL_C1RXPACEEN_RESETVAL (0x00000000u)
/*----C1RXPACEEN Tokens----*/
#define CSL_ECTL_INTCONTROL_C1RXPACEEN_DISABLE (0x00000000u)
#define CSL_ECTL_INTCONTROL_C1RXPACEEN_ENABLE (0x00000001u)

#define CSL_ECTL_INTCONTROL_C0TXPACEEN_MASK (0x00020000u)
#define CSL_ECTL_INTCONTROL_C0TXPACEEN_SHIFT (0x00000011u)
#define CSL_ECTL_INTCONTROL_C0TXPACEEN_RESETVAL (0x00000000u)
/*----C0TXPACEEN Tokens----*/
#define CSL_ECTL_INTCONTROL_C0TXPACEEN_DISABLE (0x00000000u)
#define CSL_ECTL_INTCONTROL_C0TXPACEEN_ENABLE (0x00000001u)

#define CSL_ECTL_INTCONTROL_C0RXPACEEN_MASK (0x00010000u)
#define CSL_ECTL_INTCONTROL_C0RXPACEEN_SHIFT (0x00000010u)
#define CSL_ECTL_INTCONTROL_C0RXPACEEN_RESETVAL (0x00000000u)
/*----C0RXPACEEN Tokens----*/
#define CSL_ECTL_INTCONTROL_C0RXPACEEN_DISABLE (0x00000000u)
#define CSL_ECTL_INTCONTROL_C0RXPACEEN_ENABLE (0x00000001u)


#define CSL_ECTL_INTCONTROL_INTPRESCALE_MASK (0x00000FFFu)
#define CSL_ECTL_INTCONTROL_INTPRESCALE_SHIFT (0x00000000u)
#define CSL_ECTL_INTCONTROL_INTPRESCALE_RESETVAL (0x00000000u)

#define CSL_ECTL_INTCONTROL_RESETVAL (0x00000000u)

/* C0RXTHRESHEN */


#define CSL_ECTL_C0RXTHRESHEN_RXCH7THRESHEN_MASK (0x00000080u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH7THRESHEN_SHIFT (0x00000007u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH7THRESHEN_RESETVAL (0x00000000u)
/*----RXCH7THRESHEN Tokens----*/
#define CSL_ECTL_C0RXTHRESHEN_RXCH7THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH7THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXTHRESHEN_RXCH6THRESHEN_MASK (0x00000040u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH6THRESHEN_SHIFT (0x00000006u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH6THRESHEN_RESETVAL (0x00000000u)
/*----RXCH6THRESHEN Tokens----*/
#define CSL_ECTL_C0RXTHRESHEN_RXCH6THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH6THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXTHRESHEN_RXCH5THRESHEN_MASK (0x00000020u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH5THRESHEN_SHIFT (0x00000005u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH5THRESHEN_RESETVAL (0x00000000u)
/*----RXCH5THRESHEN Tokens----*/
#define CSL_ECTL_C0RXTHRESHEN_RXCH5THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH5THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXTHRESHEN_RXCH4THRESHEN_MASK (0x00000010u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH4THRESHEN_SHIFT (0x00000004u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH4THRESHEN_RESETVAL (0x00000000u)
/*----RXCH4THRESHEN Tokens----*/
#define CSL_ECTL_C0RXTHRESHEN_RXCH4THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH4THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXTHRESHEN_RXCH3THRESHEN_MASK (0x00000008u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH3THRESHEN_SHIFT (0x00000003u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH3THRESHEN_RESETVAL (0x00000000u)
/*----RXCH3THRESHEN Tokens----*/
#define CSL_ECTL_C0RXTHRESHEN_RXCH3THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH3THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXTHRESHEN_RXCH2THRESHEN_MASK (0x00000004u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH2THRESHEN_SHIFT (0x00000002u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH2THRESHEN_RESETVAL (0x00000000u)
/*----RXCH2THRESHEN Tokens----*/
#define CSL_ECTL_C0RXTHRESHEN_RXCH2THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH2THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXTHRESHEN_RXCH1THRESHEN_MASK (0x00000002u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH1THRESHEN_SHIFT (0x00000001u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH1THRESHEN_RESETVAL (0x00000000u)
/*----RXCH1THRESHEN Tokens----*/
#define CSL_ECTL_C0RXTHRESHEN_RXCH1THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH1THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXTHRESHEN_RXCH0THRESHEN_MASK (0x00000001u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH0THRESHEN_SHIFT (0x00000000u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH0THRESHEN_RESETVAL (0x00000000u)
/*----RXCH0THRESHEN Tokens----*/
#define CSL_ECTL_C0RXTHRESHEN_RXCH0THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXTHRESHEN_RXCH0THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXTHRESHEN_RESETVAL (0x00000000u)

/* C0RXEN */


#define CSL_ECTL_C0RXEN_RXCH7EN_MASK (0x00000080u)
#define CSL_ECTL_C0RXEN_RXCH7EN_SHIFT (0x00000007u)
#define CSL_ECTL_C0RXEN_RXCH7EN_RESETVAL (0x00000000u)
/*----RXCH7EN Tokens----*/
#define CSL_ECTL_C0RXEN_RXCH7EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXEN_RXCH7EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXEN_RXCH6EN_MASK (0x00000040u)
#define CSL_ECTL_C0RXEN_RXCH6EN_SHIFT (0x00000006u)
#define CSL_ECTL_C0RXEN_RXCH6EN_RESETVAL (0x00000000u)
/*----RXCH6EN Tokens----*/
#define CSL_ECTL_C0RXEN_RXCH6EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXEN_RXCH6EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXEN_RXCH5EN_MASK (0x00000020u)
#define CSL_ECTL_C0RXEN_RXCH5EN_SHIFT (0x00000005u)
#define CSL_ECTL_C0RXEN_RXCH5EN_RESETVAL (0x00000000u)
/*----RXCH5EN Tokens----*/
#define CSL_ECTL_C0RXEN_RXCH5EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXEN_RXCH5EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXEN_RXCH4EN_MASK (0x00000010u)
#define CSL_ECTL_C0RXEN_RXCH4EN_SHIFT (0x00000004u)
#define CSL_ECTL_C0RXEN_RXCH4EN_RESETVAL (0x00000000u)
/*----RXCH4EN Tokens----*/
#define CSL_ECTL_C0RXEN_RXCH4EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXEN_RXCH4EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXEN_RXCH3EN_MASK (0x00000008u)
#define CSL_ECTL_C0RXEN_RXCH3EN_SHIFT (0x00000003u)
#define CSL_ECTL_C0RXEN_RXCH3EN_RESETVAL (0x00000000u)
/*----RXCH3EN Tokens----*/
#define CSL_ECTL_C0RXEN_RXCH3EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXEN_RXCH3EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXEN_RXCH2EN_MASK (0x00000004u)
#define CSL_ECTL_C0RXEN_RXCH2EN_SHIFT (0x00000002u)
#define CSL_ECTL_C0RXEN_RXCH2EN_RESETVAL (0x00000000u)
/*----RXCH2EN Tokens----*/
#define CSL_ECTL_C0RXEN_RXCH2EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXEN_RXCH2EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXEN_RXCH1EN_MASK (0x00000002u)
#define CSL_ECTL_C0RXEN_RXCH1EN_SHIFT (0x00000001u)
#define CSL_ECTL_C0RXEN_RXCH1EN_RESETVAL (0x00000000u)
/*----RXCH1EN Tokens----*/
#define CSL_ECTL_C0RXEN_RXCH1EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXEN_RXCH1EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXEN_RXCH0EN_MASK (0x00000001u)
#define CSL_ECTL_C0RXEN_RXCH0EN_SHIFT (0x00000000u)
#define CSL_ECTL_C0RXEN_RXCH0EN_RESETVAL (0x00000000u)
/*----RXCH0EN Tokens----*/
#define CSL_ECTL_C0RXEN_RXCH0EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0RXEN_RXCH0EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0RXEN_RESETVAL (0x00000000u)

/* C0TXEN */


#define CSL_ECTL_C0TXEN_TXCH7EN_MASK (0x00000080u)
#define CSL_ECTL_C0TXEN_TXCH7EN_SHIFT (0x00000007u)
#define CSL_ECTL_C0TXEN_TXCH7EN_RESETVAL (0x00000000u)
/*----TXCH7EN Tokens----*/
#define CSL_ECTL_C0TXEN_TXCH7EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0TXEN_TXCH7EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0TXEN_TXCH6EN_MASK (0x00000040u)
#define CSL_ECTL_C0TXEN_TXCH6EN_SHIFT (0x00000006u)
#define CSL_ECTL_C0TXEN_TXCH6EN_RESETVAL (0x00000000u)
/*----TXCH6EN Tokens----*/
#define CSL_ECTL_C0TXEN_TXCH6EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0TXEN_TXCH6EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0TXEN_TXCH5EN_MASK (0x00000020u)
#define CSL_ECTL_C0TXEN_TXCH5EN_SHIFT (0x00000005u)
#define CSL_ECTL_C0TXEN_TXCH5EN_RESETVAL (0x00000000u)
/*----TXCH5EN Tokens----*/
#define CSL_ECTL_C0TXEN_TXCH5EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0TXEN_TXCH5EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0TXEN_TXCH4EN_MASK (0x00000010u)
#define CSL_ECTL_C0TXEN_TXCH4EN_SHIFT (0x00000004u)
#define CSL_ECTL_C0TXEN_TXCH4EN_RESETVAL (0x00000000u)
/*----TXCH4EN Tokens----*/
#define CSL_ECTL_C0TXEN_TXCH4EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0TXEN_TXCH4EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0TXEN_TXCH3EN_MASK (0x00000008u)
#define CSL_ECTL_C0TXEN_TXCH3EN_SHIFT (0x00000003u)
#define CSL_ECTL_C0TXEN_TXCH3EN_RESETVAL (0x00000000u)
/*----TXCH3EN Tokens----*/
#define CSL_ECTL_C0TXEN_TXCH3EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0TXEN_TXCH3EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0TXEN_TXCH2EN_MASK (0x00000004u)
#define CSL_ECTL_C0TXEN_TXCH2EN_SHIFT (0x00000002u)
#define CSL_ECTL_C0TXEN_TXCH2EN_RESETVAL (0x00000000u)
/*----TXCH2EN Tokens----*/
#define CSL_ECTL_C0TXEN_TXCH2EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0TXEN_TXCH2EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0TXEN_TXCH1EN_MASK (0x00000002u)
#define CSL_ECTL_C0TXEN_TXCH1EN_SHIFT (0x00000001u)
#define CSL_ECTL_C0TXEN_TXCH1EN_RESETVAL (0x00000000u)
/*----TXCH1EN Tokens----*/
#define CSL_ECTL_C0TXEN_TXCH1EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0TXEN_TXCH1EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0TXEN_TXCH0EN_MASK (0x00000001u)
#define CSL_ECTL_C0TXEN_TXCH0EN_SHIFT (0x00000000u)
#define CSL_ECTL_C0TXEN_TXCH0EN_RESETVAL (0x00000000u)
/*----TXCH0EN Tokens----*/
#define CSL_ECTL_C0TXEN_TXCH0EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0TXEN_TXCH0EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0TXEN_RESETVAL (0x00000000u)

/* C0MISCEN */


#define CSL_ECTL_C0MISCEN_STATPENDEN_MASK (0x00000008u)
#define CSL_ECTL_C0MISCEN_STATPENDEN_SHIFT (0x00000003u)
#define CSL_ECTL_C0MISCEN_STATPENDEN_RESETVAL (0x00000000u)
/*----STATPENDEN Tokens----*/
#define CSL_ECTL_C0MISCEN_STATPENDEN_DISABLE (0x00000000u)
#define CSL_ECTL_C0MISCEN_STATPENDEN_ENABLE (0x00000001u)

#define CSL_ECTL_C0MISCEN_HOSTPENDEN_MASK (0x00000004u)
#define CSL_ECTL_C0MISCEN_HOSTPENDEN_SHIFT (0x00000002u)
#define CSL_ECTL_C0MISCEN_HOSTPENDEN_RESETVAL (0x00000000u)
/*----HOSTPENDEN Tokens----*/
#define CSL_ECTL_C0MISCEN_HOSTPENDEN_DISABLE (0x00000000u)
#define CSL_ECTL_C0MISCEN_HOSTPENDEN_ENABLE (0x00000001u)

#define CSL_ECTL_C0MISCEN_LINKINT0EN_MASK (0x00000002u)
#define CSL_ECTL_C0MISCEN_LINKINT0EN_SHIFT (0x00000001u)
#define CSL_ECTL_C0MISCEN_LINKINT0EN_RESETVAL (0x00000000u)
/*----LINKINT0EN Tokens----*/
#define CSL_ECTL_C0MISCEN_LINKINT0EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0MISCEN_LINKINT0EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0MISCEN_USERINT0EN_MASK (0x00000001u)
#define CSL_ECTL_C0MISCEN_USERINT0EN_SHIFT (0x00000000u)
#define CSL_ECTL_C0MISCEN_USERINT0EN_RESETVAL (0x00000000u)
/*----USERINT0EN Tokens----*/
#define CSL_ECTL_C0MISCEN_USERINT0EN_DISABLE (0x00000000u)
#define CSL_ECTL_C0MISCEN_USERINT0EN_ENABLE (0x00000001u)

#define CSL_ECTL_C0MISCEN_RESETVAL (0x00000000u)

/* C1RXTHRESHEN */


#define CSL_ECTL_C1RXTHRESHEN_RXCH7THRESHEN_MASK (0x00000080u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH7THRESHEN_SHIFT (0x00000007u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH7THRESHEN_RESETVAL (0x00000000u)
/*----RXCH7THRESHEN Tokens----*/
#define CSL_ECTL_C1RXTHRESHEN_RXCH7THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH7THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXTHRESHEN_RXCH6THRESHEN_MASK (0x00000040u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH6THRESHEN_SHIFT (0x00000006u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH6THRESHEN_RESETVAL (0x00000000u)
/*----RXCH6THRESHEN Tokens----*/
#define CSL_ECTL_C1RXTHRESHEN_RXCH6THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH6THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXTHRESHEN_RXCH5THRESHEN_MASK (0x00000020u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH5THRESHEN_SHIFT (0x00000005u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH5THRESHEN_RESETVAL (0x00000000u)
/*----RXCH5THRESHEN Tokens----*/
#define CSL_ECTL_C1RXTHRESHEN_RXCH5THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH5THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXTHRESHEN_RXCH4THRESHEN_MASK (0x00000010u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH4THRESHEN_SHIFT (0x00000004u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH4THRESHEN_RESETVAL (0x00000000u)
/*----RXCH4THRESHEN Tokens----*/
#define CSL_ECTL_C1RXTHRESHEN_RXCH4THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH4THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXTHRESHEN_RXCH3THRESHEN_MASK (0x00000008u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH3THRESHEN_SHIFT (0x00000003u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH3THRESHEN_RESETVAL (0x00000000u)
/*----RXCH3THRESHEN Tokens----*/
#define CSL_ECTL_C1RXTHRESHEN_RXCH3THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH3THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXTHRESHEN_RXCH2THRESHEN_MASK (0x00000004u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH2THRESHEN_SHIFT (0x00000002u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH2THRESHEN_RESETVAL (0x00000000u)
/*----RXCH2THRESHEN Tokens----*/
#define CSL_ECTL_C1RXTHRESHEN_RXCH2THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH2THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXTHRESHEN_RXCH1THRESHEN_MASK (0x00000002u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH1THRESHEN_SHIFT (0x00000001u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH1THRESHEN_RESETVAL (0x00000000u)
/*----RXCH1THRESHEN Tokens----*/
#define CSL_ECTL_C1RXTHRESHEN_RXCH1THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH1THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXTHRESHEN_RXCH0THRESHEN_MASK (0x00000001u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH0THRESHEN_SHIFT (0x00000000u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH0THRESHEN_RESETVAL (0x00000000u)
/*----RXCH0THRESHEN Tokens----*/
#define CSL_ECTL_C1RXTHRESHEN_RXCH0THRESHEN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXTHRESHEN_RXCH0THRESHEN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXTHRESHEN_RESETVAL (0x00000000u)

/* C1RXEN */


#define CSL_ECTL_C1RXEN_RXCH7EN_MASK (0x00000080u)
#define CSL_ECTL_C1RXEN_RXCH7EN_SHIFT (0x00000007u)
#define CSL_ECTL_C1RXEN_RXCH7EN_RESETVAL (0x00000000u)
/*----RXCH7EN Tokens----*/
#define CSL_ECTL_C1RXEN_RXCH7EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXEN_RXCH7EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXEN_RXCH6EN_MASK (0x00000040u)
#define CSL_ECTL_C1RXEN_RXCH6EN_SHIFT (0x00000006u)
#define CSL_ECTL_C1RXEN_RXCH6EN_RESETVAL (0x00000000u)
/*----RXCH6EN Tokens----*/
#define CSL_ECTL_C1RXEN_RXCH6EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXEN_RXCH6EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXEN_RXCH5EN_MASK (0x00000020u)
#define CSL_ECTL_C1RXEN_RXCH5EN_SHIFT (0x00000005u)
#define CSL_ECTL_C1RXEN_RXCH5EN_RESETVAL (0x00000000u)
/*----RXCH5EN Tokens----*/
#define CSL_ECTL_C1RXEN_RXCH5EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXEN_RXCH5EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXEN_RXCH4EN_MASK (0x00000010u)
#define CSL_ECTL_C1RXEN_RXCH4EN_SHIFT (0x00000004u)
#define CSL_ECTL_C1RXEN_RXCH4EN_RESETVAL (0x00000000u)
/*----RXCH4EN Tokens----*/
#define CSL_ECTL_C1RXEN_RXCH4EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXEN_RXCH4EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXEN_RXCH3EN_MASK (0x00000008u)
#define CSL_ECTL_C1RXEN_RXCH3EN_SHIFT (0x00000003u)
#define CSL_ECTL_C1RXEN_RXCH3EN_RESETVAL (0x00000000u)
/*----RXCH3EN Tokens----*/
#define CSL_ECTL_C1RXEN_RXCH3EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXEN_RXCH3EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXEN_RXCH2EN_MASK (0x00000004u)
#define CSL_ECTL_C1RXEN_RXCH2EN_SHIFT (0x00000002u)
#define CSL_ECTL_C1RXEN_RXCH2EN_RESETVAL (0x00000000u)
/*----RXCH2EN Tokens----*/
#define CSL_ECTL_C1RXEN_RXCH2EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXEN_RXCH2EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXEN_RXCH1EN_MASK (0x00000002u)
#define CSL_ECTL_C1RXEN_RXCH1EN_SHIFT (0x00000001u)
#define CSL_ECTL_C1RXEN_RXCH1EN_RESETVAL (0x00000000u)
/*----RXCH1EN Tokens----*/
#define CSL_ECTL_C1RXEN_RXCH1EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXEN_RXCH1EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXEN_RXCH0EN_MASK (0x00000001u)
#define CSL_ECTL_C1RXEN_RXCH0EN_SHIFT (0x00000000u)
#define CSL_ECTL_C1RXEN_RXCH0EN_RESETVAL (0x00000000u)
/*----RXCH0EN Tokens----*/
#define CSL_ECTL_C1RXEN_RXCH0EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1RXEN_RXCH0EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1RXEN_RESETVAL (0x00000000u)

/* C1TXEN */


#define CSL_ECTL_C1TXEN_TXCH7EN_MASK (0x00000080u)
#define CSL_ECTL_C1TXEN_TXCH7EN_SHIFT (0x00000007u)
#define CSL_ECTL_C1TXEN_TXCH7EN_RESETVAL (0x00000000u)
/*----TXCH7EN Tokens----*/
#define CSL_ECTL_C1TXEN_TXCH7EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1TXEN_TXCH7EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1TXEN_TXCH6EN_MASK (0x00000040u)
#define CSL_ECTL_C1TXEN_TXCH6EN_SHIFT (0x00000006u)
#define CSL_ECTL_C1TXEN_TXCH6EN_RESETVAL (0x00000000u)
/*----TXCH6EN Tokens----*/
#define CSL_ECTL_C1TXEN_TXCH6EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1TXEN_TXCH6EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1TXEN_TXCH5EN_MASK (0x00000020u)
#define CSL_ECTL_C1TXEN_TXCH5EN_SHIFT (0x00000005u)
#define CSL_ECTL_C1TXEN_TXCH5EN_RESETVAL (0x00000000u)
/*----TXCH5EN Tokens----*/
#define CSL_ECTL_C1TXEN_TXCH5EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1TXEN_TXCH5EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1TXEN_TXCH4EN_MASK (0x00000010u)
#define CSL_ECTL_C1TXEN_TXCH4EN_SHIFT (0x00000004u)
#define CSL_ECTL_C1TXEN_TXCH4EN_RESETVAL (0x00000000u)
/*----TXCH4EN Tokens----*/
#define CSL_ECTL_C1TXEN_TXCH4EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1TXEN_TXCH4EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1TXEN_TXCH3EN_MASK (0x00000008u)
#define CSL_ECTL_C1TXEN_TXCH3EN_SHIFT (0x00000003u)
#define CSL_ECTL_C1TXEN_TXCH3EN_RESETVAL (0x00000000u)
/*----TXCH3EN Tokens----*/
#define CSL_ECTL_C1TXEN_TXCH3EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1TXEN_TXCH3EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1TXEN_TXCH2EN_MASK (0x00000004u)
#define CSL_ECTL_C1TXEN_TXCH2EN_SHIFT (0x00000002u)
#define CSL_ECTL_C1TXEN_TXCH2EN_RESETVAL (0x00000000u)
/*----TXCH2EN Tokens----*/
#define CSL_ECTL_C1TXEN_TXCH2EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1TXEN_TXCH2EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1TXEN_TXCH1EN_MASK (0x00000002u)
#define CSL_ECTL_C1TXEN_TXCH1EN_SHIFT (0x00000001u)
#define CSL_ECTL_C1TXEN_TXCH1EN_RESETVAL (0x00000000u)
/*----TXCH1EN Tokens----*/
#define CSL_ECTL_C1TXEN_TXCH1EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1TXEN_TXCH1EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1TXEN_TXCH0EN_MASK (0x00000001u)
#define CSL_ECTL_C1TXEN_TXCH0EN_SHIFT (0x00000000u)
#define CSL_ECTL_C1TXEN_TXCH0EN_RESETVAL (0x00000000u)
/*----TXCH0EN Tokens----*/
#define CSL_ECTL_C1TXEN_TXCH0EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1TXEN_TXCH0EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1TXEN_RESETVAL (0x00000000u)

/* C1MISCEN */


#define CSL_ECTL_C1MISCEN_STATPENDEN_MASK (0x00000008u)
#define CSL_ECTL_C1MISCEN_STATPENDEN_SHIFT (0x00000003u)
#define CSL_ECTL_C1MISCEN_STATPENDEN_RESETVAL (0x00000000u)
/*----STATPENDEN Tokens----*/
#define CSL_ECTL_C1MISCEN_STATPENDEN_DISABLE (0x00000000u)
#define CSL_ECTL_C1MISCEN_STATPENDEN_ENABLE (0x00000001u)

#define CSL_ECTL_C1MISCEN_HOSTPENDEN_MASK (0x00000004u)
#define CSL_ECTL_C1MISCEN_HOSTPENDEN_SHIFT (0x00000002u)
#define CSL_ECTL_C1MISCEN_HOSTPENDEN_RESETVAL (0x00000000u)
/*----HOSTPENDEN Tokens----*/
#define CSL_ECTL_C1MISCEN_HOSTPENDEN_DISABLE (0x00000000u)
#define CSL_ECTL_C1MISCEN_HOSTPENDEN_ENABLE (0x00000001u)

#define CSL_ECTL_C1MISCEN_LINKINT0EN_MASK (0x00000002u)
#define CSL_ECTL_C1MISCEN_LINKINT0EN_SHIFT (0x00000001u)
#define CSL_ECTL_C1MISCEN_LINKINT0EN_RESETVAL (0x00000000u)
/*----LINKINT0EN Tokens----*/
#define CSL_ECTL_C1MISCEN_LINKINT0EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1MISCEN_LINKINT0EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1MISCEN_USERINT0EN_MASK (0x00000001u)
#define CSL_ECTL_C1MISCEN_USERINT0EN_SHIFT (0x00000000u)
#define CSL_ECTL_C1MISCEN_USERINT0EN_RESETVAL (0x00000000u)
/*----USERINT0EN Tokens----*/
#define CSL_ECTL_C1MISCEN_USERINT0EN_DISABLE (0x00000000u)
#define CSL_ECTL_C1MISCEN_USERINT0EN_ENABLE (0x00000001u)

#define CSL_ECTL_C1MISCEN_RESETVAL (0x00000000u)

/* C0RXTHRESHSTAT */


#define CSL_ECTL_C0RXTHRESHSTAT_RXCH7THRESHSTAT_MASK (0x00000080u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH7THRESHSTAT_SHIFT (0x00000007u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH7THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXTHRESHSTAT_RXCH6THRESHSTAT_MASK (0x00000040u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH6THRESHSTAT_SHIFT (0x00000006u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH6THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXTHRESHSTAT_RXCH5THRESHSTAT_MASK (0x00000020u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH5THRESHSTAT_SHIFT (0x00000005u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH5THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXTHRESHSTAT_RXCH4THRESHSTAT_MASK (0x00000010u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH4THRESHSTAT_SHIFT (0x00000004u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH4THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXTHRESHSTAT_RXCH3THRESHSTAT_MASK (0x00000008u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH3THRESHSTAT_SHIFT (0x00000003u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH3THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXTHRESHSTAT_RXCH2THRESHSTAT_MASK (0x00000004u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH2THRESHSTAT_SHIFT (0x00000002u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH2THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXTHRESHSTAT_RXCH1THRESHSTAT_MASK (0x00000002u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH1THRESHSTAT_SHIFT (0x00000001u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH1THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXTHRESHSTAT_RXCH0THRESHSTAT_MASK (0x00000001u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH0THRESHSTAT_SHIFT (0x00000000u)
#define CSL_ECTL_C0RXTHRESHSTAT_RXCH0THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXTHRESHSTAT_RESETVAL (0x00000000u)

/* C0RXSTAT */


#define CSL_ECTL_C0RXSTAT_RXCH7STAT_MASK (0x00000080u)
#define CSL_ECTL_C0RXSTAT_RXCH7STAT_SHIFT (0x00000007u)
#define CSL_ECTL_C0RXSTAT_RXCH7STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXSTAT_RXCH6STAT_MASK (0x00000040u)
#define CSL_ECTL_C0RXSTAT_RXCH6STAT_SHIFT (0x00000006u)
#define CSL_ECTL_C0RXSTAT_RXCH6STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXSTAT_RXCH5STAT_MASK (0x00000020u)
#define CSL_ECTL_C0RXSTAT_RXCH5STAT_SHIFT (0x00000005u)
#define CSL_ECTL_C0RXSTAT_RXCH5STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXSTAT_RXCH4STAT_MASK (0x00000010u)
#define CSL_ECTL_C0RXSTAT_RXCH4STAT_SHIFT (0x00000004u)
#define CSL_ECTL_C0RXSTAT_RXCH4STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXSTAT_RXCH3STAT_MASK (0x00000008u)
#define CSL_ECTL_C0RXSTAT_RXCH3STAT_SHIFT (0x00000003u)
#define CSL_ECTL_C0RXSTAT_RXCH3STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXSTAT_RXCH2STAT_MASK (0x00000004u)
#define CSL_ECTL_C0RXSTAT_RXCH2STAT_SHIFT (0x00000002u)
#define CSL_ECTL_C0RXSTAT_RXCH2STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXSTAT_RXCH1STAT_MASK (0x00000002u)
#define CSL_ECTL_C0RXSTAT_RXCH1STAT_SHIFT (0x00000001u)
#define CSL_ECTL_C0RXSTAT_RXCH1STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXSTAT_RXCH0STAT_MASK (0x00000001u)
#define CSL_ECTL_C0RXSTAT_RXCH0STAT_SHIFT (0x00000000u)
#define CSL_ECTL_C0RXSTAT_RXCH0STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXSTAT_RESETVAL (0x00000000u)

/* C0TXSTAT */


#define CSL_ECTL_C0TXSTAT_TXCH7STAT_MASK (0x00000080u)
#define CSL_ECTL_C0TXSTAT_TXCH7STAT_SHIFT (0x00000007u)
#define CSL_ECTL_C0TXSTAT_TXCH7STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0TXSTAT_TXCH6STAT_MASK (0x00000040u)
#define CSL_ECTL_C0TXSTAT_TXCH6STAT_SHIFT (0x00000006u)
#define CSL_ECTL_C0TXSTAT_TXCH6STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0TXSTAT_TXCH5STAT_MASK (0x00000020u)
#define CSL_ECTL_C0TXSTAT_TXCH5STAT_SHIFT (0x00000005u)
#define CSL_ECTL_C0TXSTAT_TXCH5STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0TXSTAT_TXCH4STAT_MASK (0x00000010u)
#define CSL_ECTL_C0TXSTAT_TXCH4STAT_SHIFT (0x00000004u)
#define CSL_ECTL_C0TXSTAT_TXCH4STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0TXSTAT_TXCH3STAT_MASK (0x00000008u)
#define CSL_ECTL_C0TXSTAT_TXCH3STAT_SHIFT (0x00000003u)
#define CSL_ECTL_C0TXSTAT_TXCH3STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0TXSTAT_TXCH2STAT_MASK (0x00000004u)
#define CSL_ECTL_C0TXSTAT_TXCH2STAT_SHIFT (0x00000002u)
#define CSL_ECTL_C0TXSTAT_TXCH2STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0TXSTAT_TXCH1STAT_MASK (0x00000002u)
#define CSL_ECTL_C0TXSTAT_TXCH1STAT_SHIFT (0x00000001u)
#define CSL_ECTL_C0TXSTAT_TXCH1STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0TXSTAT_TXCH0STAT_MASK (0x00000001u)
#define CSL_ECTL_C0TXSTAT_TXCH0STAT_SHIFT (0x00000000u)
#define CSL_ECTL_C0TXSTAT_TXCH0STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0TXSTAT_RESETVAL (0x00000000u)

/* C0MISCSTAT */


#define CSL_ECTL_C0MISCSTAT_STATPENDSTAT_MASK (0x00000008u)
#define CSL_ECTL_C0MISCSTAT_STATPENDSTAT_SHIFT (0x00000003u)
#define CSL_ECTL_C0MISCSTAT_STATPENDSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0MISCSTAT_HOSTPENDSTAT_MASK (0x00000004u)
#define CSL_ECTL_C0MISCSTAT_HOSTPENDSTAT_SHIFT (0x00000002u)
#define CSL_ECTL_C0MISCSTAT_HOSTPENDSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0MISCSTAT_LINKINT0STAT_MASK (0x00000002u)
#define CSL_ECTL_C0MISCSTAT_LINKINT0STAT_SHIFT (0x00000001u)
#define CSL_ECTL_C0MISCSTAT_LINKINT0STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0MISCSTAT_USERINT0STAT_MASK (0x00000001u)
#define CSL_ECTL_C0MISCSTAT_USERINT0STAT_SHIFT (0x00000000u)
#define CSL_ECTL_C0MISCSTAT_USERINT0STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C0MISCSTAT_RESETVAL (0x00000000u)

/* C1RXTHRESHSTAT */


#define CSL_ECTL_C1RXTHRESHSTAT_RXCH7THRESHSTAT_MASK (0x00000080u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH7THRESHSTAT_SHIFT (0x00000007u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH7THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXTHRESHSTAT_RXCH6THRESHSTAT_MASK (0x00000040u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH6THRESHSTAT_SHIFT (0x00000006u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH6THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXTHRESHSTAT_RXCH5THRESHSTAT_MASK (0x00000020u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH5THRESHSTAT_SHIFT (0x00000005u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH5THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXTHRESHSTAT_RXCH4THRESHSTAT_MASK (0x00000010u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH4THRESHSTAT_SHIFT (0x00000004u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH4THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXTHRESHSTAT_RXCH3THRESHSTAT_MASK (0x00000008u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH3THRESHSTAT_SHIFT (0x00000003u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH3THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXTHRESHSTAT_RXCH2THRESHSTAT_MASK (0x00000004u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH2THRESHSTAT_SHIFT (0x00000002u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH2THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXTHRESHSTAT_RXCH1THRESHSTAT_MASK (0x00000002u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH1THRESHSTAT_SHIFT (0x00000001u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH1THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXTHRESHSTAT_RXCH0THRESHSTAT_MASK (0x00000001u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH0THRESHSTAT_SHIFT (0x00000000u)
#define CSL_ECTL_C1RXTHRESHSTAT_RXCH0THRESHSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXTHRESHSTAT_RESETVAL (0x00000000u)

/* C1RXSTAT */


#define CSL_ECTL_C1RXSTAT_RXCH7STAT_MASK (0x00000080u)
#define CSL_ECTL_C1RXSTAT_RXCH7STAT_SHIFT (0x00000007u)
#define CSL_ECTL_C1RXSTAT_RXCH7STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXSTAT_RXCH6STAT_MASK (0x00000040u)
#define CSL_ECTL_C1RXSTAT_RXCH6STAT_SHIFT (0x00000006u)
#define CSL_ECTL_C1RXSTAT_RXCH6STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXSTAT_RXCH5STAT_MASK (0x00000020u)
#define CSL_ECTL_C1RXSTAT_RXCH5STAT_SHIFT (0x00000005u)
#define CSL_ECTL_C1RXSTAT_RXCH5STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXSTAT_RXCH4STAT_MASK (0x00000010u)
#define CSL_ECTL_C1RXSTAT_RXCH4STAT_SHIFT (0x00000004u)
#define CSL_ECTL_C1RXSTAT_RXCH4STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXSTAT_RXCH3STAT_MASK (0x00000008u)
#define CSL_ECTL_C1RXSTAT_RXCH3STAT_SHIFT (0x00000003u)
#define CSL_ECTL_C1RXSTAT_RXCH3STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXSTAT_RXCH2STAT_MASK (0x00000004u)
#define CSL_ECTL_C1RXSTAT_RXCH2STAT_SHIFT (0x00000002u)
#define CSL_ECTL_C1RXSTAT_RXCH2STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXSTAT_RXCH1STAT_MASK (0x00000002u)
#define CSL_ECTL_C1RXSTAT_RXCH1STAT_SHIFT (0x00000001u)
#define CSL_ECTL_C1RXSTAT_RXCH1STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXSTAT_RXCH0STAT_MASK (0x00000001u)
#define CSL_ECTL_C1RXSTAT_RXCH0STAT_SHIFT (0x00000000u)
#define CSL_ECTL_C1RXSTAT_RXCH0STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXSTAT_RESETVAL (0x00000000u)

/* C1TXSTAT */


#define CSL_ECTL_C1TXSTAT_TXCH7STAT_MASK (0x00000080u)
#define CSL_ECTL_C1TXSTAT_TXCH7STAT_SHIFT (0x00000007u)
#define CSL_ECTL_C1TXSTAT_TXCH7STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1TXSTAT_TXCH6STAT_MASK (0x00000040u)
#define CSL_ECTL_C1TXSTAT_TXCH6STAT_SHIFT (0x00000006u)
#define CSL_ECTL_C1TXSTAT_TXCH6STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1TXSTAT_TXCH5STAT_MASK (0x00000020u)
#define CSL_ECTL_C1TXSTAT_TXCH5STAT_SHIFT (0x00000005u)
#define CSL_ECTL_C1TXSTAT_TXCH5STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1TXSTAT_TXCH4STAT_MASK (0x00000010u)
#define CSL_ECTL_C1TXSTAT_TXCH4STAT_SHIFT (0x00000004u)
#define CSL_ECTL_C1TXSTAT_TXCH4STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1TXSTAT_TXCH3STAT_MASK (0x00000008u)
#define CSL_ECTL_C1TXSTAT_TXCH3STAT_SHIFT (0x00000003u)
#define CSL_ECTL_C1TXSTAT_TXCH3STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1TXSTAT_TXCH2STAT_MASK (0x00000004u)
#define CSL_ECTL_C1TXSTAT_TXCH2STAT_SHIFT (0x00000002u)
#define CSL_ECTL_C1TXSTAT_TXCH2STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1TXSTAT_TXCH1STAT_MASK (0x00000002u)
#define CSL_ECTL_C1TXSTAT_TXCH1STAT_SHIFT (0x00000001u)
#define CSL_ECTL_C1TXSTAT_TXCH1STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1TXSTAT_TXCH0STAT_MASK (0x00000001u)
#define CSL_ECTL_C1TXSTAT_TXCH0STAT_SHIFT (0x00000000u)
#define CSL_ECTL_C1TXSTAT_TXCH0STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1TXSTAT_RESETVAL (0x00000000u)

/* C1MISCSTAT */


#define CSL_ECTL_C1MISCSTAT_STATPENDSTAT_MASK (0x00000008u)
#define CSL_ECTL_C1MISCSTAT_STATPENDSTAT_SHIFT (0x00000003u)
#define CSL_ECTL_C1MISCSTAT_STATPENDSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1MISCSTAT_HOSTPENDSTAT_MASK (0x00000004u)
#define CSL_ECTL_C1MISCSTAT_HOSTPENDSTAT_SHIFT (0x00000002u)
#define CSL_ECTL_C1MISCSTAT_HOSTPENDSTAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1MISCSTAT_LINKINT0STAT_MASK (0x00000002u)
#define CSL_ECTL_C1MISCSTAT_LINKINT0STAT_SHIFT (0x00000001u)
#define CSL_ECTL_C1MISCSTAT_LINKINT0STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1MISCSTAT_USERINT0STAT_MASK (0x00000001u)
#define CSL_ECTL_C1MISCSTAT_USERINT0STAT_SHIFT (0x00000000u)
#define CSL_ECTL_C1MISCSTAT_USERINT0STAT_RESETVAL (0x00000000u)

#define CSL_ECTL_C1MISCSTAT_RESETVAL (0x00000000u)

/* RSVD */


#define CSL_ECTL_RSVD_RESETVAL (0x00000000u)

/* C0RXIMAX */


#define CSL_ECTL_C0RXIMAX_RXIMAX_MASK (0x0000003Fu)
#define CSL_ECTL_C0RXIMAX_RXIMAX_SHIFT (0x00000000u)
#define CSL_ECTL_C0RXIMAX_RXIMAX_RESETVAL (0x00000000u)

#define CSL_ECTL_C0RXIMAX_RESETVAL (0x00000000u)

/* C0TXIMAX */


#define CSL_ECTL_C0TXIMAX_TXIMAX_MASK (0x0000003Fu)
#define CSL_ECTL_C0TXIMAX_TXIMAX_SHIFT (0x00000000u)
#define CSL_ECTL_C0TXIMAX_TXIMAX_RESETVAL (0x00000000u)

#define CSL_ECTL_C0TXIMAX_RESETVAL (0x00000000u)

/* C1RXIMAX */


#define CSL_ECTL_C1RXIMAX_RXIMAX_MASK (0x0000003Fu)
#define CSL_ECTL_C1RXIMAX_RXIMAX_SHIFT (0x00000000u)
#define CSL_ECTL_C1RXIMAX_RXIMAX_RESETVAL (0x00000000u)

#define CSL_ECTL_C1RXIMAX_RESETVAL (0x00000000u)

/* C1TXIMAX */


#define CSL_ECTL_C1TXIMAX_TXIMAX_MASK (0x0000003Fu)
#define CSL_ECTL_C1TXIMAX_TXIMAX_SHIFT (0x00000000u)
#define CSL_ECTL_C1TXIMAX_TXIMAX_RESETVAL (0x00000000u)

#define CSL_ECTL_C1TXIMAX_RESETVAL (0x00000000u)

#endif  //_CSLR_ECTL1_H_
