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
File name :	mDelay.c
Author and date :	Monnerat Serge 29 ao�t 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "Modules/mDelay.h"
#include "Interfaces/iPit.h"

//-----------------------------------------------------------------------------
// Delay module setup
//-----------------------------------------------------------------------------
void mDelay_Setup(void)
{
	// --> PIT0 used for analog camera
	// --> PIT1 used by non-blocking delay
	// --> PIT2 Pixy camera
	// --> PIT3 unused

	// PIT0 delays structure setup
	iPit_InitDelay(kPit1);
	iPit_InitDelay(kPit2);
	// PIT0 setup
	iPit_Config(kPit1, kPit1Period);
	iPit_Config(kPit2, kPit2Period);
}

//-----------------------------------------------------------------------------
// Delay module open
//-----------------------------------------------------------------------------
void mDelay_Open(void)
{
	iPit_StartPit(kPit1);
	iPit_StartPit(kPit2);
}

//-----------------------------------------------------------------------------
// Delay module close
//-----------------------------------------------------------------------------
void mDelay_Close(void)
{
}

//------------------------------------------------------------
// Get one delay
// aPit		: Wich PIT
// aDelay	: delay time in ms
// Retour	: delay number or -1 if no more free delay
//
//------------------------------------------------------------
Int16 mDelay_GetDelay(PitEnum aPit, UInt16 aDelay)
{
	return iPit_GetDelay(aPit, aDelay);
}

//------------------------------------------------------------
// Delay release
// aPit		: which PIT
// aDelayNb	: delay number
//------------------------------------------------------------
void mDelay_DelayRelease(PitEnum aPit, UInt16 aDelayNb)
{
	iPit_DelayRelease(aPit, aDelayNb);
}

//------------------------------------------------------------
// Delay restart
// aPit			: which PIT
// aDelay		: delay time
// aDelayNb	: delay number
//------------------------------------------------------------
void mDelay_ReStart(PitEnum aPit, UInt16 aDelayNb, UInt16 aDelay)
{
	iPit_ReStart(aPit, aDelayNb, aDelay);
}

//------------------------------------------------------------
// Check if the delay is done
// aPit			: which PIT
// aDelayNb	: delay number
// Retour		: flag state
//------------------------------------------------------------
bool mDelay_IsDelayDone(PitEnum aPit, UInt16 aDelayNb)
{
	return iPit_IsDelayDone(aPit, aDelayNb);
}

//------------------------------------------------------------
// Sleep function
// msecounds			: time in us
//------------------------------------------------------------
void usleep(int useconds)
{
	int i = 0;
	int j = 0;
	for (i = 0; i < useconds; i++)
	{
		j++;
	}
}
