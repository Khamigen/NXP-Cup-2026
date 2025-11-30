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
Nom du fichier : 	iAd.h	
Auteur et Date :	Monnerat Serge 8.5.201x

But : Interface du p�riph�rique ADC 

Modifications
Date		Faite	Ctrl		Description
------------------------------------------------------------
*/

#ifndef __IAD__
#define __IAD__

#include "Utils/def.h" //KM

// ADC channel enum
typedef enum
{
	kADC_SE0 = 0,
	kADC_SE1 = 1,
	kADC_SE2 = 2,
	kADC_SE3 = 3,
	kADC_SE4a = 4,
	kADC_SE5a = 5,
	kADC_SE6a = 6,
	kADC_SE7a = 7,
	kADC_SE4b = 4,
	kADC_SE5b = 5,
	kADC_SE6b = 6,
	kADC_SE7b = 7,
	kADC_SE8 = 8,
	kADC_SE9 = 9,
	kADC_SE11 = 11,
	kADC_SE12 = 12,
	kADC_SE13 = 13,
	kADC_SE14 = 14,
	kADC_SE15 = 15
} ADCChannelEnum;

//------------------------------------------------------------
// Configuration du convertisseur AD
//------------------------------------------------------------
void iAd_Config(void);

//------------------------------------------------------------
// Lecture du r�sultat
// Retour: le r�sultat
//------------------------------------------------------------
UInt16 iAd_GetResult(void);

//------------------------------------------------------------
// S�lection du canal � convertir
// aCh: quel canal
//------------------------------------------------------------
void iAd_SelectChannel(ADCChannelEnum aCh);

//------------------------------------------------------------
// Lecture du drapeau de fin de conversion
// Retour: �tat du flag
//------------------------------------------------------------
bool iAd_GetCompleteConvStatus(void);

//------------------------------------------------------------
// Calibration du convertisseur AD
// Retour: true = OK, false = KO
//------------------------------------------------------------
bool iAd_Cal(void);

#endif
