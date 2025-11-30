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
File name :	iDio.c
Author and date :	Monnerat Serge 7 mars 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "iDio.h"
#include "MK64F12.h"

//-----------------------------------------------------------------------------
// IO ports direction setup
// aPort: which IO port
// aMask: which port pin(s) to configure
// aDir:  input or output
//-----------------------------------------------------------------------------
void iDio_SetPortDirection(PortIOEnum aPort, IoMaskEnum aMask, IoDirectionEnum aDir)
{
	UInt32 *aPortPtr;

	// Port Data Direction Register (GPIOx_PDDR)
	switch (aPort)
	{
		// GPIOA_PDDR
	case kPortA:
		aPortPtr = (UInt32 *)&GPIOA->PDDR;
		break;

		// GPIOB_PDDR
	case kPortB:
		aPortPtr = (UInt32 *)&GPIOB->PDDR;
		break;

		// GPIOC_PDDR
	case kPortC:
		aPortPtr = (UInt32 *)&GPIOC->PDDR;
		break;

	// GPIOD_PDDR
	case kPortD:
		aPortPtr = (UInt32 *)&GPIOD->PDDR;
		break;
	// GPIOE_PDDR
	case kPortE:
		aPortPtr = (UInt32 *)&GPIOE->PDDR;
		break;
	}

	// IO direction setup
	if (kIoOutput == aDir)
	{
		*aPortPtr |= aMask;
	}
	else if (kIoInput == aDir)
	{
		*aPortPtr &= ~aMask;
	}
}

//-----------------------------------------------------------------------------
// Output pin set state
// aPort: which IO port
// aMask: which port pin(s) to configure
// aState:  output pin state
//-----------------------------------------------------------------------------
void iDio_SetPort(PortIOEnum aPort, IoMaskEnum aMask, IoStateEnum aState)
{
	UInt32 *aPortSetPtr;
	UInt32 *aPortClrPtr;

	// Port Data Output Register (GPIOx_PDOR)
	// Port Clear Output Register (GPIOx_PCOR)
	switch (aPort)
	{
	// GPIOA_PDOR & GPIOA_PCOR
	case kPortA:
		aPortSetPtr = (UInt32 *)&GPIOA->PDOR;
		aPortClrPtr = (UInt32 *)&GPIOA->PCOR;
		break;
	// GPIOB_PDOR & GPIOB_PCOR
	case kPortB:
		aPortSetPtr = (UInt32 *)&GPIOB->PDOR;
		aPortClrPtr = (UInt32 *)&GPIOB->PCOR;
		break;

	// GPIOC_PDOR & GPIOC_PCOR
	case kPortC:
		aPortSetPtr = (UInt32 *)&GPIOC->PDOR;
		aPortClrPtr = (UInt32 *)&GPIOC->PCOR;
		break;

	// GPIOD_PDOR & GPIOD_PCOR
	case kPortD:
		aPortSetPtr = (UInt32 *)&GPIOD->PDOR;
		aPortClrPtr = (UInt32 *)&GPIOD->PCOR;
		break;
	// GPIOE_PDOR & GPIOE_PCOR
	case kPortE:
		aPortSetPtr = (UInt32 *)&GPIOE->PDOR;
		aPortClrPtr = (UInt32 *)&GPIOE->PCOR;
		break;
	}

	// Set or reset the output
	if (kIoOn == aState)
	{
		*aPortSetPtr |= aMask;
	}
	else if (kIoOff == aState)
	{
		*aPortClrPtr |= aMask;
	}
}

//-----------------------------------------------------------------------------
// Read input pin state
// aPort: which IO port
// aMask: which port pin(s) to configure
// return:  input pin state
//-----------------------------------------------------------------------------
bool iDio_GetPort(PortIOEnum aPort, IoMaskEnum aMask)
{
	bool aRet = false;
	UInt32 *aPortPtr;

	// Port Data Input Register (GPIOx_PDIR)
	switch (aPort)
	{
	// GPIOA_PDIR
	case kPortA:
		aPortPtr = (UInt32 *)&GPIOA->PDIR;
		break;

	// GPIOB_PDIR
	case kPortB:
		aPortPtr = (UInt32 *)&GPIOB->PDIR;
		break;

	// GPIOC_PDIR
	case kPortC:
		aPortPtr = (UInt32 *)&GPIOC->PDIR;
		break;

	// GPIOD_PDIR
	case kPortD:
		aPortPtr = (UInt32 *)&GPIOD->PDIR;
		break;
	// GPIOE_PDIR
	case kPortE:
		aPortPtr = (UInt32 *)&GPIOE->PDIR;
		break;
	}

	// Input pin state
	aRet = (bool)((*aPortPtr & aMask) == aMask);

	return aRet;
}
