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
File name :	iDac.c
Author and date :	Monnerat Serge 28 f�vr. 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "iDac.h"
#include "MK64F12.h"

//------------------------------------------------------------
// Constante
//------------------------------------------------------------
#define kDACx_SR_RESET 2
#define kDACx_C0_RESET 0
#define kDACx_C1_RESET 0
#define kDACx_C2_RESET 15 //0x0f

//------------------------------------------------------------
// Prototype
//------------------------------------------------------------
// Set DAC register to reset value
static void iDac_SetResetVal(DacEnum aDac);

//------------------------------------------------------------
// DAC 0 & 1 setup
//------------------------------------------------------------
void iDac_Config(DacEnum aDac, DacModeEnum aMode)
{
	// Enable du clock du DAC
	SIM->SCGC2 |= SIM_SCGC2_DAC0_MASK;

	switch (aMode)
	{
	case kNonBuffVRefIn:
		if (kDac0 == aDac)
		{
			// Set DAC register to reset value
			iDac_SetResetVal(kDac0);

			// DAC Control Register (DACx_C0)
			// DACBBIEN =0 , buffer read pointer  bottom flag interrupt is disabled
			// DACBTIEN = 0 ,buffer read pointer top flag interrupt is disabled
			// DACBWIEN = 0, buffer water mark interrupt disabled
			// DACSWTRG=0, NO Software trigger yet
			// LPEN = 0, high power mode
			// DACTRGSEL =1, Databuffer outputs to DACO pin whenever a write to DACDAT0
			// DACRFS=0,The DAC select DACREF_1 (1.2V) as the reference voltage,select VREF Output as reference
			// DACEN=1, DAC enabled
			DAC0->C0 |= (DAC_C0_DACEN_MASK | DAC_C0_DACTRGSEL_MASK | DAC_C0_DACRFS_MASK);

			// DAC Control Register (DACx_C1)
			// DMAEN=0, DMA disabled
			// DACBFWM=0, DAC buffer watermark select 1 words
			// DACBFMD=0, DAC buffer work mode select -> Normal Mode
			// DACBFEN=0, Buffer read pointer disabled. The converted data is always the first word of the buffer.
			DAC0->C1 = 0;

			// DAC Control Register (DACx_C2)
			// DACBFRP=0,DAC buffer read pointer =0
			// DACBFUP=15,DAC buffer upper limit
			DAC0->C2 = (0 | DAC_C2_DACBFRP(0) | DAC_C2_DACBFUP(0x0f));
		}
		else if (kDac1 == aDac)
		{
			// Set DAC register to reset value
			iDac_SetResetVal(kDac1);

			// DAC Control Register (DACx_C0)
			// DACBBIEN =0 , buffer read pointer  bottom flag interrupt is disabled
			// DACBTIEN = 0 ,buffer read pointer top flag interrupt is disabled
			// DACBWIEN = 0, buffer water mark interrupt disabled
			// DACSWTRG=0, NO Software trigger yet
			// LPEN = 0, high power mode
			// DACTRGSEL =1, Databuffer outputs to DACO pin whenever a write to DACDAT0
			// DACRFS=0,The DAC selets DACREF_1 as the reference voltage,select VREF Output as reference
			// DACEN=1, DAC enabled
			DAC1->C0 |= (DAC_C0_DACEN_MASK | DAC_C0_DACTRGSEL_MASK);

			// DAC Control Register (DACx_C1)
			// DMAEN=0, DMA disabled
			// DACBFWM=0, DAC buffer watermark select 1 words
			// DACBFMD=0, DAC buffer work mode select -> Normal Mode
			// DACBFEN=0, Buffer read pointer disabled. The converted data is always the first word of the buffer.
			DAC1->C1 = 0;

			// DAC Control Register (DACx_C2)
			// DACBFRP=0,DAC buffer read pointer =0
			// DACBFUP=15,DAC buffer upper limit
			DAC1->C2 = (0 | DAC_C2_DACBFRP(0) | DAC_C2_DACBFUP(0x0f));
		}

		break;

	case kBuffVRefIn:
		if (kDac0 == aDac)
		{
			// Set DAC register to reset value
			iDac_SetResetVal(kDac0);

			//....
		}
		else if (kDac1 == aDac)
		{
			// Set DAC register to reset value
			iDac_SetResetVal(kDac1);

			// ....
		}

		break;
	}
}

//------------------------------------------------------------
// Dac enable
// aDac: which dac (0 or 1)
//------------------------------------------------------------
void iDac_Enable(DacEnum aDac)
{
	// DAC Control Register (DACx_C0)
	// DACEN=1, DAC enabled
	switch (aDac)
	{
	case kDac0:
		DAC0->C0 |= DAC_C0_DACEN_MASK;
		break;

	case kDac1:
		DAC1->C0 |= DAC_C0_DACEN_MASK;
		break;
	}
}

//------------------------------------------------------------
// Dac disable
// aDac: which dac (0 or 1)
//------------------------------------------------------------
void iDac_Disable(DacEnum aDac)
{
	// DAC Control Register (DACx_C0)
	// DACEN=1, DAC enabled
	switch (aDac)
	{
	case kDac0:
		DAC0->C0 &= (~DAC_C0_DACEN_MASK);
		break;

	case kDac1:
		DAC1->C0 &= (~DAC_C0_DACEN_MASK);
		break;
	}
}

//------------------------------------------------------------
// Set DAC register to reset value
// aDac: which dac (0 or 1)
//------------------------------------------------------------
static void iDac_SetResetVal(DacEnum aDac)
{
	UInt8 i;
	UInt16 *aPtr;

	switch (aDac)
	{
	case kDac0:
		aPtr = (UInt16 *)&DAC0->DAT[0].DATL;
		for (i = 0; i < 16; i++, aPtr++)
		{
			*aPtr = 0;
		}
		DAC0->SR = kDACx_SR_RESET;
		DAC0->C0 = kDACx_C0_RESET;
		DAC0->C1 = kDACx_C1_RESET;
		DAC0->C2 = kDACx_C2_RESET;
		break;

	case kDac1:
		aPtr = (UInt16 *)&DAC1->DAT[0].DATL;
		for (i = 0; i < 16; i++, aPtr++)
		{
			*aPtr = 0;
		}
		DAC1->SR = kDACx_SR_RESET;
		DAC1->C0 = kDACx_C0_RESET;
		DAC1->C1 = kDACx_C1_RESET;
		DAC1->C2 = kDACx_C2_RESET;
		break;
	}
}

//------------------------------------------------------------
// Set DAC buffer value
// aDac: which dac (0 or 1)
// aBuffIndex: which buffer position to configure
// aBuffVal: which value to set
//------------------------------------------------------------
void iDac_SETDACBUFFER(DacEnum aDac, UInt8 aBuffIndex, float aBuffVal)
{
	UInt16 *aPtr;
	UInt16 aVal;

	aVal = (UInt16)(1117.0 * aBuffVal + 372.0);

	if (aVal > 1489)
	{
		aVal = 1489;
	}
	if (aVal < 372)
	{
		aVal = 372;
	}

	switch (aDac)
	{
	case kDac0:
		aPtr = (UInt16 *)&DAC0->DAT[0].DATL;
		aPtr += aBuffIndex;
		*aPtr = aVal & 0x0fff;
		break;

	case kDac1:
		aPtr = (UInt16 *)&DAC1->DAT[0].DATL;
		aPtr += aBuffIndex;
		*aPtr = aVal & 0x0fff;
		break;
	}
}
