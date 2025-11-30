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
File name :	iPit.c
Author and date :	Monnerat Serge 29 ao�t 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "iPit.h"
#include "MK64F12.h"

//------------------------------------------------------------
// Constantes
//------------------------------------------------------------
// Maximum delay number to use together
#define kCounterNumber 10

//------------------------------------------------------------
// Types
//------------------------------------------------------------
// Delay struct
typedef struct
{
	UInt16 Counter;
	bool isFree;
	bool DelayDone;
} CounterStruct;

typedef struct
{
	CounterStruct CounterTab[kCounterNumber];
} DlyStruct;

static DlyStruct sDly[3];

//------------------------------------------------------------
// PIT setup
// aPit					: which PIT (0 to 3)
// aPeriodeMs  	: PIT period in ms
//------------------------------------------------------------
void iPit_Config(PitEnum aPit, UInt16 aPeriodeMs)
{
	static UInt32 aTmp = 0;
	UInt8 i = 0;

	// Enable du clock du p�riph�rique PIT
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

	// PIT Module Control Register (PIT_MCR)
	// --> Timers are stopped in debug mode
	PIT->MCR |= PIT_MCR_FRZ_MASK;

	// PIT interrupts setup
	if (aPit == kPit0)
	{
		// Counting value to do the delay
		// Counting Value=Delay Time/(1/Core clock)
		aTmp = aPeriodeMs / (1 / (kClockPeriphkHz));
		// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1017
		// Timer Load Value Register (PIT_LDVALn)
		PIT->CHANNEL[0].LDVAL = aTmp;
		// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1018
		// Timer Control Register (PIT_TCTRLn)
		// Enable PIT interrupt
		PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;
		// Clear prending interrupt
		NVIC_ClearPendingIRQ(PIT1_IRQn);
		// Interrupt Set-enable Registers
		// Interrupt enable
		NVIC_EnableIRQ(PIT1_IRQn);
		// Interrupt Priority Registers
		// Set interrupt priority
		NVIC_SetPriority(PIT1_IRQn, kPIT_IntPriority);
	}
	else if (aPit == kPit1)
	{
		// Counting value to do the delay
		// Counting Value=Delay Time/(1/Core clock)
		aTmp = aPeriodeMs / (1 / (kClockPeriphkHz));
		// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1017
		// Timer Load Value Register (PIT_LDVALn)
		PIT->CHANNEL[1].LDVAL = aTmp;
		// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1018
		// Timer Control Register (PIT_TCTRLn)
		// Enable PIT interrupt
		PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TIE_MASK;
		// Clear prending interrupt
		NVIC_ClearPendingIRQ(PIT1_IRQn);
		// Interrupt Set-enable Registers
		// Interrupt enable
		NVIC_EnableIRQ(PIT1_IRQn);
		// Interrupt Priority Registers
		// Set interrupt priority
		NVIC_SetPriority(PIT1_IRQn, kPIT_IntPriority);
	}
	else if (aPit == kPit2)
	{
		// Counting value to do the delay
		// Counting Value=Delay Time/(1/Core clock)
		aTmp = aPeriodeMs / (1 / (kClockPeriphkHz));
		// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1017
		// Timer Load Value Register (PIT_LDVALn)
		PIT->CHANNEL[2].LDVAL = aTmp;
		// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1018
		// Timer Control Register (PIT_TCTRLn)
		// Enable PIT interrupt
		PIT->CHANNEL[2].TCTRL |= PIT_TCTRL_TIE_MASK;
		// Clear prending interrupt
		NVIC_ClearPendingIRQ(PIT2_IRQn);
		// Interrupt Set-enable Registers
		// Interrupt enable
		NVIC_EnableIRQ(PIT2_IRQn);
		// Interrupt Priority Registers
		// Set interrupt priority
		NVIC_SetPriority(PIT2_IRQn, kPIT_IntPriority);
	}
	else if (aPit == kPit3)
	{
		// Counting value to do the delay
		// Counting Value=Delay Time/(1/Core clock)
		aTmp = aPeriodeMs / (1 / (kClockPeriphkHz));
		// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1017
		// Timer Load Value Register (PIT_LDVALn)
		PIT->CHANNEL[3].LDVAL = aTmp;
		// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1018
		// Timer Control Register (PIT_TCTRLn)
		// Enable PIT interrupt
		PIT->CHANNEL[3].TCTRL |= PIT_TCTRL_TIE_MASK;
		// Clear prending interrupt
		NVIC_ClearPendingIRQ(PIT3_IRQn);
		// Interrupt Set-enable Registers
		// Interrupt enable
		NVIC_EnableIRQ(PIT3_IRQn);
		// Interrupt Priority Registers
		// Set interrupt priority
		NVIC_SetPriority(PIT3_IRQn, kPIT_IntPriority);
	}

	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1016
	// PIT Module Control Register (PIT_MCR)
	// Clock for PIT Timers is enabled
	PIT->MCR &= (~PIT_MCR_MDIS_MASK);
}

//------------------------------------------------------------
// Start Pit
// aPit		: which PIT
//------------------------------------------------------------
void iPit_StartPit(PitEnum aPit)
{
	// Start PIT
	if (aPit == kPit0)
	{
		// Timer Control Register (PIT_TCTRLn)
		// Timer is active
		PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
	}
	else if (aPit == kPit1)
	{
		// Timer Control Register (PIT_TCTRLn)
		// Timer is active
		PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TEN_MASK;
	}
	else if (aPit == kPit2)
	{
		// Timer Control Register (PIT_TCTRLn)
		// Timer is active
		PIT->CHANNEL[2].TCTRL |= PIT_TCTRL_TEN_MASK;
	}
	else if (aPit == kPit3)
	{
		// Timer Control Register (PIT_TCTRLn)
		// Timer is active
		PIT->CHANNEL[3].TCTRL |= PIT_TCTRL_TEN_MASK;
	}
}

//------------------------------------------------------------
// Setup the delay structure
// aPit		: which PIT
//------------------------------------------------------------
void iPit_InitDelay(PitEnum aPit)
{
	UInt16 i = 0;

	for (i = 0; i < kCounterNumber; i++)
	{
		sDly[aPit].CounterTab[i].DelayDone = false;
		sDly[aPit].CounterTab[i].Counter = 0;
		sDly[aPit].CounterTab[i].isFree = true;
	}
}
//------------------------------------------------------------
// Get and setup a delay
// aPit		: which PIT
// aDelay	: the delay (ms)
// Retour	: the delay number
//------------------------------------------------------------
Int16 iPit_GetDelay(PitEnum aPit, UInt16 aDelay)
{
	Int16 aDelayNb = 0;
	UInt16 i = 0;

	// By default
	aDelayNb = 0;

	// Find and confegure a free delay
	for (i = 0; (i < kCounterNumber) && (false == sDly[aPit].CounterTab[i].isFree); i++, aDelayNb = i)
		;

	// Check if a delay was found
	if ((i == kCounterNumber) && (false == sDly[aPit].CounterTab[i - 1].isFree))
		aDelayNb = -1;
	else
	{
		// Delay setup
		sDly[aPit].CounterTab[aDelayNb].isFree = false;
		sDly[aPit].CounterTab[aDelayNb].Counter = aDelay;
		sDly[aPit].CounterTab[aDelayNb].DelayDone = false;
	}

	return aDelayNb;
}

//------------------------------------------------------------
// Check if the delay is done
// aPit			: which PIT
// aDelayNb	: the delay number
// Retour		: state of the delay
//------------------------------------------------------------
bool iPit_IsDelayDone(PitEnum aPit, UInt16 aDelayNb)
{
	return sDly[aPit].CounterTab[aDelayNb].DelayDone;
}

//------------------------------------------------------------
// Release a delay
// aPit			: which PIT
// aDelayNb	: the delay number
//------------------------------------------------------------
void iPit_DelayRelease(PitEnum aPit, UInt16 aDelayNb)
{
	sDly[aPit].CounterTab[aDelayNb].DelayDone = false;
	sDly[aPit].CounterTab[aDelayNb].Counter = 0;
	sDly[aPit].CounterTab[aDelayNb].isFree = true;
}

//------------------------------------------------------------
// Restart un delay
// aPit			: which PIT
// aDelayNb	: the delay number
// aDelay		: the delay time
//------------------------------------------------------------
void iPit_ReStart(PitEnum aPit, UInt16 aDelayNb, UInt16 aDelay)
{
	sDly[aPit].CounterTab[aDelayNb].DelayDone = false;
	sDly[aPit].CounterTab[aDelayNb].Counter = aDelay;
	sDly[aPit].CounterTab[aDelayNb].isFree = false;
}

//------------------------------------------------------------
// PIT 0 interrupt routine
//------------------------------------------------------------
// Cam�ra digitale
#if (kDigCamera == 1)
void PIT0_IRQHandler(void)
{
	UInt16 i = 0;
	UInt32 aTmp;

	// PIT TIF flag clear
	PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
	aTmp = PIT->CHANNEL[0].CVAL;

	// Counter update
	for (i = 0; i < kCounterNumber; i++)
	{
		if ((false == sDly[kPit0].CounterTab[i].isFree) && (sDly[kPit0].CounterTab[i].Counter > 0))
			sDly[kPit0].CounterTab[i].Counter--;

		if ((false == sDly[kPit0].CounterTab[i].isFree) && (sDly[kPit0].CounterTab[i].Counter == 0))
			sDly[kPit0].CounterTab[i].DelayDone = true;
	}
}
#endif

//------------------------------------------------------------
// PIT 1 interrupt routine
//------------------------------------------------------------
void PIT1_IRQHandler(void)
{
	UInt16 i = 0;
	UInt32 aTmp;

	// PIT TIF flag clear
	PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;
	aTmp = PIT->CHANNEL[1].CVAL;

	// Counter update
	for (i = 0; i < kCounterNumber; i++)
	{
		if ((false == sDly[kPit1].CounterTab[i].isFree) && (sDly[kPit1].CounterTab[i].Counter > 0))
			sDly[kPit1].CounterTab[i].Counter--;

		if ((false == sDly[kPit1].CounterTab[i].isFree) && (sDly[kPit1].CounterTab[i].Counter == 0))
			sDly[kPit1].CounterTab[i].DelayDone = true;
	}
}

//------------------------------------------------------------
// PIT 2 interrupt routine
//------------------------------------------------------------
void PIT2_IRQHandler(void)
{
	UInt16 i = 0;
	UInt32 aTmp;

	// PIT TIF flag clear
	PIT->CHANNEL[2].TFLG |= PIT_TFLG_TIF_MASK;
	aTmp = PIT->CHANNEL[2].CVAL;

	// Counter update
	for (i = 0; i < kCounterNumber; i++)
	{
		if ((false == sDly[kPit2].CounterTab[i].isFree) && (sDly[kPit2].CounterTab[i].Counter > 0))
			sDly[kPit2].CounterTab[i].Counter--;

		if ((false == sDly[kPit2].CounterTab[i].isFree) && (sDly[kPit2].CounterTab[i].Counter == 0))
			sDly[kPit2].CounterTab[i].DelayDone = true;
	}
}

//------------------------------------------------------------
// PIT 3 interrupt routine
//------------------------------------------------------------
void PIT3_IRQHandler(void)
{
	UInt16 i = 0;
	UInt32 aTmp;

	// PIT TIF flag clear
	PIT->CHANNEL[3].TFLG |= PIT_TFLG_TIF_MASK;
	aTmp = PIT->CHANNEL[3].CVAL;

	// Counter update
	for (i = 0; i < kCounterNumber; i++)
	{
		if ((false == sDly[kPit3].CounterTab[i].isFree) && (sDly[kPit3].CounterTab[i].Counter > 0))
			sDly[kPit3].CounterTab[i].Counter--;

		if ((false == sDly[kPit3].CounterTab[i].isFree) && (sDly[kPit3].CounterTab[i].Counter == 0))
			sDly[kPit3].CounterTab[i].DelayDone = true;
	}
}
