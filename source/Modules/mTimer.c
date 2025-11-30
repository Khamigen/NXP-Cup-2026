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
File name :	mTimer.c
Author and date :	Monnerat Serge 29 ao�t 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/
#include "Modules/mTimer.h"		   //KM
#include "Interfaces/iFlextimer.h" //KM
#include "Interfaces/iDio.h"	   //KM

//-----------------------------------------------------------------------------
// Speed Mes module setup
//-----------------------------------------------------------------------------
void mTimer_Setup(void)
{

	// Flextimer setup
	iFlextimer_Config();

	// Pin direction et enable du pont en H en sortie
	iDio_SetPortDirection(kPortA, kMaskIo15 + kMaskIo16 + kMaskIo17, kIoOutput);
	iDio_SetPortDirection(kPortB, kMaskIo9, kIoOutput);
	iDio_SetPortDirection(kPortB, kMaskIo16 + kMaskIo17, kIoInput);
	iDio_SetPortDirection(kPortA, kMaskIo5 + kMaskIo14, kIoInput);
}

//-----------------------------------------------------------------------------
// Speed Mes module start
//-----------------------------------------------------------------------------
void mTimer_Open(void)
{
	mTimer_SetServoDuty(0, 0);
	mTimer_SetServoDuty(1, 0);
	mTimer_SetMotorDuty(0, 0);
	// Enable des ponts en H
	iDio_SetPort(kPortA, kMaskIo17, kIoOn);
	iDio_SetPort(kPortA, kMaskIo15, kIoOff);
	iDio_SetPort(kPortA, kMaskIo16, kIoOff);
}

//-----------------------------------------------------------------------------
// Speed Mes module stop
//-----------------------------------------------------------------------------
void mTimer_Close(void)
{
}

//-----------------------------------------------------------------------------
// Lecture de la vitesse de rotation des moteurs
// Moteur A = moteur gauche --> valeur n�gative = en arri�re, valeur pos=en avant
// Moteur B = moteur droite
//-----------------------------------------------------------------------------
void mTimer_GetSpeed(float *aSpeedMotLeft, float *aSpeedMotRight)
{
	iFlextimer_GetSpeed(aSpeedMotLeft, aSpeedMotRight);
}

//---------------------------------------------------------------------------
// Modifie la position du servo
// Position entre -1.0 to 1.0
//---------------------------------------------------------------------------
void mTimer_SetServoDuty(UInt8 aServoNumber, float aPosition)
{
	iFlextimer_SetServoDuty(aServoNumber, aPosition);
}

//---------------------------------------------------------------------------
// Modifie le duty du PWM
// Moteur A = moteur gauche (-1 � +1)
// Moteur B = moteur droite (-1 � +1)
//---------------------------------------------------------------------------
void mTimer_SetMotorDuty(float aMotorLeft, float aMotorRight)
{
	iFlextimer_SetMotorDuty(aMotorLeft, aMotorRight);
}

//---------------------------------------------------------------------------
// Enable des ponts en H des moteurs DC
//---------------------------------------------------------------------------
void mTimer_EnableHBridge(void)
{
	// Enable des ponts en H
	iDio_SetPort(kPortA, kMaskIo17, kIoOn);
	iDio_SetPort(kPortA, kMaskIo15, kIoOff);
	iDio_SetPort(kPortA, kMaskIo16, kIoOff);
}

//---------------------------------------------------------------------------
// Disable des ponts en H des moteurs DC
//---------------------------------------------------------------------------
void mTimer_DisableHBridge(void)
{
	// Disable des ponts en H
	iDio_SetPort(kPortA, kMaskIo15, kIoOn);
	iDio_SetPort(kPortA, kMaskIo16, kIoOn);
}

//---------------------------------------------------------------------------
// Lecture du statut du pont en H gauche,
//---------------------------------------------------------------------------
bool mTimer_GetFaultMoteurLeft(void)
{
	// Lecture du statut
	return !iDio_GetPort(kPortB, kMaskIo16);
}

//---------------------------------------------------------------------------
// Lecture du statut du pont en H droit
//---------------------------------------------------------------------------
bool mTimer_GetFaultMoteurRight(void)
{
	// Lecture du statut
	return !iDio_GetPort(kPortB, kMaskIo17);
}
