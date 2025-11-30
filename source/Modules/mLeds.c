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
File name :	mLeds.c
Author and date :	Monnerat Serge 28 f�vr. 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "mLeds.h"
#include "Interfaces/iDio.h" //KM

//-----------------------------------------------------------------------------
// Configuration des ports IO utilis�s par les LEDS
//-----------------------------------------------------------------------------
void mLeds_Setup(void)
{
	// Configuration des Ports en entr�e ou en sortie
	iDio_SetPortDirection(kPortB, kMaskIo20 + kMaskIo21 + kMaskIo22 + kMaskIo23, kIoOutput);
}

//-----------------------------------------------------------------------------
// Start du module
//-----------------------------------------------------------------------------
void mLeds_Open(void)
{
}

//-----------------------------------------------------------------------------
// On Off des LED
// aMask:   choix des LED sur lesquelles on veut agir (si bit � 1)
// aState:  choix de l'�tat des LED d�sir�s
//-----------------------------------------------------------------------------
void mLeds_Write(LedMaskEnum aMask, LedStateEnum aState)
{
	UInt32 aTmp;

	if (kLedOn == aState)
	{
		aTmp = (0 | ((aMask & 0x1) << 20) | ((aMask & 0x2) << (20)) | ((aMask & 0x04) << (20)) | ((aMask & 0x08) << (20)));

		iDio_SetPort(kPortB, (IoMaskEnum)(aTmp), kIoOn);
	}
	else if (kLedOff == aState)
	{
		aTmp = (0 | ((aMask & 0x1) << 20) | ((aMask & 0x2) << (20)) | ((aMask & 0x04) << (20)) | ((aMask & 0x08) << (20)));
		iDio_SetPort(kPortB, (IoMaskEnum)(aTmp), kIoOff);
	}
}

//-----------------------------------------------------------------------------
// On Off des LED
// aMask:   choix des LED sur lesquelles on veut agir (si bit � 1)
//-----------------------------------------------------------------------------
void mLeds_Toggle(int aMask)
{
	UInt32 aTmp;
	static LedStateEnum pState = kLedOff;

	if (kLedOn == pState)
	{
		pState = kLedOff;

		aTmp = (0 | ((aMask & 0x1) << 20) | ((aMask & 0x2) << (20)) | ((aMask & 0x04) << (20)) | ((aMask & 0x08) << (20)));

		iDio_SetPort(kPortB, (IoMaskEnum)(aTmp), kIoOn);
	}
	else if (kLedOff == pState)
	{
		pState = kLedOn;
		aTmp = (0 | ((aMask & 0x1) << 20) | ((aMask & 0x2) << (20)) | ((aMask & 0x04) << (20)) | ((aMask & 0x08) << (20)));
		iDio_SetPort(kPortB, (IoMaskEnum)(aTmp), kIoOff);
	}
}
