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
File name : 	iDac.h	
Author and date :	Monnerat Serge 28 f�vr. 201x

Goal : 

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $


-----------------------------------------------------------------------------
*/
#ifndef __iDac__
#define __iDac__

#include "Utils/def.h" //KM

// Dac enum
typedef enum
{
	kDac0,
	kDac1
} DacEnum;

// Dac mode enum
typedef enum
{
	kNonBuffVRefIn,
	kBuffVRefIn
} DacModeEnum;

//------------------------------------------------------------
// DAC 0 & 1 setup
//------------------------------------------------------------
void iDac_Config(DacEnum aDac, DacModeEnum aMode);

//------------------------------------------------------------
// Dac enable
// aDac: which dac (0 or 1)
//------------------------------------------------------------
void iDac_Enable(DacEnum aDac);

//------------------------------------------------------------
// Dac disable
// aDac: which dac (0 or 1)
//------------------------------------------------------------
void iDac_Disable(DacEnum aDac);

//------------------------------------------------------------
// Set DAC buffer value
// aDac: which dac (0 or 1)
// aBuffIndex: which buffer position to configure
// aBuffVal: which value to set
//------------------------------------------------------------
void iDac_SETDACBUFFER(DacEnum aDac, UInt8 aBuffIndex, float aBuffVal);

#endif
