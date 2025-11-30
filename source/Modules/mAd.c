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
File name :	mAd.c
Author and date :	Monnerat Serge 28 f�vr. 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "mAd.h"
#include "Interfaces/iAd.h" //KM

//-----------------------------------------------------------------------
// Configuration du module ADC
//-----------------------------------------------------------------------
void mAd_Setup(void)
{
	iAd_Config();
	iAd_Cal();
	iAd_Config();
}

//-----------------------------------------------------------------------
// ADC module open
//-----------------------------------------------------------------------
void mAd_Open(void)
{
}

//-----------------------------------------------------------------------
// ADC module close
//-----------------------------------------------------------------------
void mAd_Close(void)
{
}

//-----------------------------------------------------------------------
// Start et lecture du r�sultat
// On retourne entre -1 et +1
//-----------------------------------------------------------------------
float mAd_Read(ADCInputEnum aAdcInp)
{
	UInt16 aRet;
	float aVal;

	switch (aAdcInp)
	{
	case kPot1:
		iAd_SelectChannel(kADC_SE3);
		while (iAd_GetCompleteConvStatus() == false)
			;
		aRet = iAd_GetResult();
		// On retourne entre -1 et +1
		aVal = (aRet / -32768.0) + 1.0;
		break;
	case kPot2:
		iAd_SelectChannel(kADC_SE1);
		while (iAd_GetCompleteConvStatus() == false)
			;
		aRet = iAd_GetResult();
		// On retourne entre -1 et +1
		aVal = (aRet / -32768.0) + 1.0;
		break;
	case kUBatt:
		iAd_SelectChannel(kADC_SE4b);
		while (iAd_GetCompleteConvStatus() == false)
			;
		aRet = iAd_GetResult();
		aVal = (aRet * (3.3 / 65535.0)) / 0.1754386;
		break;
	case kIHBridgeLeft:
		iAd_SelectChannel(kADC_SE8);
		while (iAd_GetCompleteConvStatus() == false)
			;
		aRet = iAd_GetResult();
		aVal = aRet * (3.3 / 65535.0);
		aVal /= 100.0;
		aVal *= 375.0;
		break;
	case kIHBridgeRight:
		iAd_SelectChannel(kADC_SE9);
		while (iAd_GetCompleteConvStatus() == false)
			;
		aRet = iAd_GetResult();
		aVal = ((aRet * (3.3 / 65535.0)) / 100.0) * 375.0;
		break;
	}

	return aVal;
}

//-----------------------------------------------------------------------
// Start et lecture du r�sultat
// On retourne entre 0 et 65535
//-----------------------------------------------------------------------
float mAd_ReadCamera(ADCInputEnum aAdcInp)
{
	UInt16 aRet;

	switch (aAdcInp)
	{
	case kPot1:
		iAd_SelectChannel(kADC_SE3);
		while (iAd_GetCompleteConvStatus() == false)
			;
		aRet = iAd_GetResult();
		break;
	case kPot2:
		iAd_SelectChannel(kADC_SE1);
		while (iAd_GetCompleteConvStatus() == false)
			;
		aRet = iAd_GetResult();
		break;
	}

	return aRet;
}
