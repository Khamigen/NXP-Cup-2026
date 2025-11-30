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
/*----------------------------------------------------------
File name :	iAd.c
Author and date :	Monnerat Serge 28 f�vr. 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "iAd.h"
#include "MK64F12.h"

//------------------------------------------------------------
// Configuration du convertisseur AD
//------------------------------------------------------------
void iAd_Config(void)
{
	// Lets calibrate the ADC. 1st setup how the channel will be used.
	NVIC_DisableIRQ(ADC1_IRQn);

	// ADC configuration register 1 (ADCx_CFG1)
	// ADLPC=0, Normal power configuration
	// ADIV=0,The divide ratio is 1 and the clock rate is (input clock)/1
	// ADLSMP=0,short sample time
	// MODE=11,It is single-ended 16-bit conversion
	// ADICLK=0,Bus clock divided by 1 --> 50MHz
	ADC1->CFG1 = 0;
	ADC1->CFG1 |= (ADC_CFG1_ADIV(0) | ADC_CFG1_MODE(3) | ADC_CFG1_ADICLK(0));

	// Configuration register 2 (ADCx_CFG2)
	// MUXSEL=1,ADxxb channels are selected
	// ADACKEN=0,Asynchronous clock output disabled
	// ADHSC=1,high speed conversion sequence selected
	// ADLSTS=3,2 extra ADCK cycles; 6 ADCK cycles total sample time
	ADC1->CFG2 = 0 | ADC_CFG2_ADHSC_MASK | ADC_CFG2_ADLSTS(3) | ADC_CFG2_MUXSEL(1);

	// --> une conversion 5us --> 128 pixels: 640us

	// Status and control register 2 (ADCx_SC2)
	// ADTRG=0,Software trigger selected
	// ACFE=0,Compare function disabled
	// ACFGT=0,Configures less than threshold, outside range not inclusive and inside range not inclusive functionality
	// based on the values placed in the CV1 and CV2 registers
	// ACREN=0,Range function disabled. Only the compare value 1 register (CV1) is compared
	// DMAEN=0,DMA is disabled
	// REFSEL=0,Default voltage reference pin pair (external pins VREFH and VREFL)
	ADC1->SC2 = 0;

	// Status and control register 2 (ADCx_SC3)
	// ADCO=0, One conversion or one set of conversions if the hardware average function is enabled (AVGE=1) after initiating a conversion.
	// AVGE=0, Hardware average function disabled
	// AVGS=0, 4 samples averaged
	ADC1->SC3 = 0 | ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0);

	// ADC status and control registers 1 (ADCx_SC1n)
	// AIEN=0,Conversion complete interrupt disabled
	// DIFF=0,Differential mode enable -> Single-ended conversions and input channels are selected
	// ADCH=0x03,Input channel select -> POT1
	ADC1->SC1[0] = 0;
	ADC1->SC1[0] = 0 | (ADC_SC1_ADCH(3));
}

//------------------------------------------------------------
// Lecture du r�sultat
// Retour: le r�sultat
//------------------------------------------------------------
UInt16 iAd_GetResult(void)
{
	UInt16 aVal;

	// Lecture du r�sultat
	aVal = ADC1->R[0];

	return aVal;
}

//------------------------------------------------------------
// S�lection du canal � convertir
// aCh: quel canal
//------------------------------------------------------------
void iAd_SelectChannel(ADCChannelEnum aCh)
{
	UInt16 aVal;

	// Clear ADCH field
	ADC1->SC1[0] &= (~ADC_SC1_ADCH_MASK);

	// Configuration du canal
	ADC1->SC1[0] |= (ADC_SC1_ADCH(aCh));
}

//------------------------------------------------------------
// Lecture du drapeau de fin de conversion
// Retour: �tat du flag
//------------------------------------------------------------
bool iAd_GetCompleteConvStatus(void)
{
	bool aRet;

	// Lecture du flag
	aRet = (ADC1->SC1[0] & ADC_SC1_COCO_MASK) == ADC_SC1_COCO_MASK;

	return aRet;
}

//------------------------------------------------------------
// Calibration du convertisseur AD
// Retour: true = OK, false = KO
//------------------------------------------------------------
bool iAd_Cal(void)
{
	UInt16 aCalVar = 0;
	bool aRet;

	// Max ADC clock -> 4MHz
	ADC1->CFG1 &= ((~ADC_CFG1_ADIV_MASK) & (~ADC_CFG1_ADICLK_MASK));
	ADC1->CFG1 |= (ADC_CFG1_ADIV(3) | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(1) | ADC_CFG1_ADLSMP_MASK);

	// Enable Software Conversion Trigger for Calibration Process
	ADC1->SC2 &= (~ADC_SC2_ADTRG_MASK);
	// Set single conversion, clear avgs bitfield for next writing
	ADC1->SC3 &= ((~ADC_SC3_ADCO_MASK) & (~ADC_SC3_AVGS_MASK));
	// Turn averaging ON and set at max value ( 32 )
	ADC1->SC3 |= (ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3));
	// Start CAL
	ADC1->SC3 |= ADC_SC3_CAL_MASK;
	// Wait calibration end
	while ((ADC1->SC1[0] & ADC_SC1_COCO_MASK) == false)
		;

	// Check for Calibration fail error and return
	if ((ADC1->SC3 & ADC_SC3_CALF_MASK) == ADC_SC3_CALF_MASK)
	{
		aRet = false;
		// Clear CAL bit
		ADC1->SC3 &= (~ADC_SC3_CAL_MASK);
	}
	else
	{
		// Calculate plus-side calibration
		aCalVar = 0x00;

		aCalVar = ADC1->CLP0;
		aCalVar += ADC1->CLP1;
		aCalVar += ADC1->CLP2;
		aCalVar += ADC1->CLP3;
		aCalVar += ADC1->CLP4;
		aCalVar += ADC1->CLPS;
		aCalVar = aCalVar / 2;
		aCalVar |= 0x8000; // Set MSB
		ADC1->PG = ADC_PG_PG(aCalVar);

		// Calculate minus-side calibration
		aCalVar = 0x00;

		aCalVar = ADC1->CLM0;
		aCalVar += ADC1->CLM1;
		aCalVar += ADC1->CLM2;
		aCalVar += ADC1->CLM3;
		aCalVar += ADC1->CLM4;
		aCalVar += ADC1->CLMS;
		aCalVar = aCalVar / 2;
		aCalVar |= 0x8000; // Set MSB
		ADC1->MG = ADC_MG_MG(aCalVar);

		// Clear CAL bit
		ADC1->SC3 &= (~ADC_SC3_CAL_MASK);

		// Return OK
		aRet = true;
	}

	return aRet;
}
