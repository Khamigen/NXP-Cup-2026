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
/*-----------------------------------------------------------------------------
File name 			: mVL6180x.h
Author				: Baertschi Jonas & Cestele Yves
Date				: 13 janv. 2017
Description			:

Modifications		: -

---------------------------------------------------------------------------*/

//--------------------------------------------------------------
// 				INCLUDES & DEFINES
//--------------------------------------------------------------

#ifndef SOURCES_MODULES_MVL6180X_H_
#define SOURCES_MODULES_MVL6180X_H_

#include "Utils/def.h" //KM

#define kVL6180x_w_addr 0x52
#define kVL6180X_REG_IDENTIFICATION_MODEL_ID 0x00
#define kVL6180X_REG_SYSTEM_INTERRUPT_CONFIG 0x14
#define kVL6180X_REG_SYSTEM_INTERRUPT_CLEAR 0x15
#define kVL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET 0x16
#define kVL6180X_REG_SYSRANGE_START 0x18
#define kVL6180X_REG_SYSALS_START 0x38
#define kVL6180X_REG_SYSALS_ANALOGUE_GAIN 0x3F
#define kVL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI 0x40
#define kVL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO 0x41
#define kVL6180X_REG_RESULT_ALS_VAL 0x50
#define kVL6180X_REG_RESULT_RANGE_VAL 0x62
#define kVL6180X_REG_RESULT_RANGE_STATUS 0x4d
#define kVL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO 0x4f

//--------------------------------------------------------------
// 				FUNCTIONS'S LIST
//--------------------------------------------------------------

//--------------------------------------------------------------
// Name			: mVL6180x_Setup
// Author		: Baertschi Jonas & Cestele Yves
// Date			: 13 janv. 2017
// Descritpion	: I2C bus setup
//
// Return		: void
//
// Modification	:
//--------------------------------------------------------------
void mVL6180x_Setup(void);

//--------------------------------------------------------------
// Name			: mVL6180x_Open
// Author		: Baertschi Jonas & Cestele Yves
// Date			: 13 janv. 2017
// Descritpion	: Open the I2C bus
//
// Return		: void
//
// Modification	:
//--------------------------------------------------------------
void mVL6180x_Open(void);

//--------------------------------------------------------------
// Name			: mVL6180x_Close
// Author		: Baertschi Jonas & Cestele Yves
// Date			: 13 janv. 2017
// Descritpion	: Close the bus
//
// Return		: void
//
// Modification	:
//--------------------------------------------------------------
void mVL6180x_Close(void);

//--------------------------------------------------------------
// Name			: mVL6180x_StartRange
// Author		: Baertschi Jonas & Cestele Yves
// Date			: 13 janv. 2017
// Descritpion	: Setup the VL61 module
//
// Return		: bool
//
// Modification	:
//--------------------------------------------------------------
bool mVL6180x_StartRange(void);

//--------------------------------------------------------------
// Name			: mVL6180x_GetRange
// Author		: Baertschi Jonas & Cestele Yves
// Date			: 13 janv. 2017
// Descritpion	: Return the distance measured by the captor
//
// Parameters	: UInt8*	distance in mm
//
// Return		: bool
//
// Modification	:
//--------------------------------------------------------------
bool mVL6180x_GetRange(UInt8 *aData);

#endif /* SOURCES_MODULES_MVL6180X_H_ */
