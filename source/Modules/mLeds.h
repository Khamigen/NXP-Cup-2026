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
File name : 	mLeds.h
Author and date :	Monnerat Serge 29 ao�t 201x

Goal : ON/OFF des LEDs

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $


-----------------------------------------------------------------------------
*/

#ifndef MLEDS_H_
#define MLEDS_H_

#include "Utils/def.h" //KM

// Masque des LED
typedef enum
{
  kMaskLed1 = 1,
  kMaskLed2 = 2,
  kMaskLed3 = 4,
  kMaskLed4 = 8
} LedMaskEnum;

// Etat des LED
typedef enum
{
  kLedOff,
  kLedOn
} LedStateEnum;

//-----------------------------------------------------------------------------
// Configuration des ports IO utilis�s par les LEDS
//-----------------------------------------------------------------------------
void mLeds_Setup(void);

//-----------------------------------------------------------------------------
// Start du module
//-----------------------------------------------------------------------------
void mLeds_Open(void);

//-----------------------------------------------------------------------------
// On Off des LED
// aMask:   choix des LED sur lesquelles on veut agir (si bit � 1)
// aState:  choix de l'�tat des LED d�sir�s
//-----------------------------------------------------------------------------
void mLeds_Write(LedMaskEnum aMask, LedStateEnum aState);

//-----------------------------------------------------------------------------
// On Off des LED
// aMask:   choix des LED sur lesquelles on veut agir (si bit � 1)
//-----------------------------------------------------------------------------
void mLeds_Toggle(int aMask);

#endif
