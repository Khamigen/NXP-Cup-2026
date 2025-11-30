/*
 * Copyright 2003-20xx Haute �cole ARC Ing�ni�rie, Switzerland. 
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Haute �cole ARC Ing�ni�rie nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/*------------------------------------------------------------
File name : 	iDio.h	
Author and date :	Monnerat Serge 7 mars 201x

Goal : 

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $


-----------------------------------------------------------------------------
*/
#ifndef __iDio__
#define __iDio__

#include "Utils/def.h"

// IO port direction
typedef enum
{
	kIoOutput,
	kIoInput
} IoDirectionEnum;

// IO bit mask
typedef enum
{
	kMaskIo0 = 0x01,
	kMaskIo1 = 0x02,
	kMaskIo2 = 0x04,
	kMaskIo3 = 0x08,
	kMaskIo4 = 0x10,
	kMaskIo5 = 0x20,
	kMaskIo6 = 0x40,
	kMaskIo7 = 0x80,
	kMaskIo8 = 0x0100,
	kMaskIo9 = 0x0200,
	kMaskIo10 = 0x0400,
	kMaskIo11 = 0x0800,
	kMaskIo12 = 0x1000,
	kMaskIo13 = 0x2000,
	kMaskIo14 = 0x4000,
	kMaskIo15 = 0x8000,
	kMaskIo16 = 0x010000,
	kMaskIo17 = 0x020000,
	kMaskIo18 = 0x040000,
	kMaskIo19 = 0x080000,
	kMaskIo20 = 0x100000,
	kMaskIo21 = 0x200000,
	kMaskIo22 = 0x400000,
	kMaskIo23 = 0x800000,
	kMaskIo24 = 0x01000000,
	kMaskIo25 = 0x02000000,
	kMaskIo26 = 0x04000000,
	kMaskIo27 = 0x08000000,
	kMaskIo28 = 0x10000000,
	kMaskIo29 = 0x20000000,
	kMaskIo30 = 0x40000000,
	kMaskIo31 = 0x80000000,
	kMaskIoAll = 0xffffffff
} IoMaskEnum;

// Avalaible IO ports
typedef enum
{
	kPortA,
	kPortB,
	kPortC,
	kPortD,
	kPortE
} PortIOEnum;

// IO state
typedef enum
{
	kIoOn,
	kIoOff
} IoStateEnum;

// IO interrupt type enum
typedef enum
{
	kIOIrqRisingEdge = 0x9,
	kIOIrqFallingEdge = 0xa,
	kIOIrqEachEdge = 0xb,
	kIOIrqAt1 = 0xc,
	kIOIrqAt0 = 0x8
} IOIrqModeEnum;

//-----------------------------------------------------------------------------
// IO ports direction setup
// aPort: which IO port
// aMask: which port pin(s) to configure
// aDir:  input or output
//-----------------------------------------------------------------------------
void iDio_SetPortDirection(PortIOEnum aPort, IoMaskEnum aMask, IoDirectionEnum aDir);

//-----------------------------------------------------------------------------
// Output pin set state
// aPort: which IO port
// aMask: which port pin(s) to configure
// aState:  output pin state
//-----------------------------------------------------------------------------
void iDio_SetPort(PortIOEnum aPort, IoMaskEnum aMask, IoStateEnum aState);

//-----------------------------------------------------------------------------
// Read input pin state
// aPort: which IO port
// aMask: which port pin(s) to configure
// return:  input pin state
//-----------------------------------------------------------------------------
bool iDio_GetPort(PortIOEnum aPort, IoMaskEnum aMask);

#endif
