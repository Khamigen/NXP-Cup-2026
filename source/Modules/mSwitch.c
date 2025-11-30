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
File name :	mSwitch.c
Author and date :	Monnerat Serge 29 ao�t 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "Modules/mSwitch.h" //KM
#include "Interfaces/iDio.h" //KM

//-----------------------------------------------------------------------------
// Configuration des ports IO utilis�s par les interrupteurs et les poussoirs
//-----------------------------------------------------------------------------
void mSwitch_Setup(void)
{
	// Configuration des Ports en entr�e ou en sortie
	iDio_SetPortDirection(kPortC, kMaskIo9 + kMaskIo10 + kMaskIo11 + kMaskIo12 + kMaskIo16 + kMaskIo17, kIoInput);
}

//-----------------------------------------------------------------------------
// Start du module
//-----------------------------------------------------------------------------
void mSwitch_Open(void)
{
}

//-------------------------------------------------------------------------
// Lecture des interrupteurs
// aSwitchNb: quel interrupteur veut-on lire
// Retour: �tat du switch
//-------------------------------------------------------------------------
bool mSwitch_ReadSwitch(SwitchEnum aSwitch)
{
	bool aState = false;

	switch (aSwitch)
	{
	case kSw1:
		aState = iDio_GetPort(kPortC, kMaskIo9);
		break;
	case kSw2:
		aState = iDio_GetPort(kPortC, kMaskIo10);
		break;
	case kSw3:
		aState = iDio_GetPort(kPortC, kMaskIo11);
		break;
	case kSw4:
		aState = iDio_GetPort(kPortC, kMaskIo12);
		break;
	}

	return aState;
}

//-------------------------------------------------------------------------
// Lecture des boutons poussoir
// aPushButNb: quel boutons poussoir veut-on lire
// Retour: �tat du bouton poussoir
//-------------------------------------------------------------------------
bool mSwitch_ReadPushBut(PushButEnum aPushButNb)
{
	bool aState = false;

	switch (aPushButNb)
	{
	case kPushButSW1:
		aState = iDio_GetPort(kPortC, kMaskIo16);
		break;
	case kPushButSW2:
		aState = iDio_GetPort(kPortC, kMaskIo17);
		break;
	}

	return aState;
}
