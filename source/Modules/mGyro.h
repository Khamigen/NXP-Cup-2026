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
File name :				mGyro.h
Author and date :	serge.monnerat, 28.2.201x

Goal :  FXAS21002C gyroscope

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

History:


-----------------------------------------------------------------------------
*/

#ifndef __FXAS21002C__
#define __FXAS21002C__

#include "Utils/def.h" //KM

//-----------------------------------------------------------------------------
// Constantes
//-----------------------------------------------------------------------------

#define kFXAS21002CAddr 0x20 // Read:0x41, Write:0x40

// FXAS21002C internal register addresses
#define DR_F_STATUS 0x00  // Alias for DR_STATUS or F_STATUS
#define OUT_X_MSB 0x01	// [7:0] are 8 MSBs of 16 bit X-axis data sample
#define OUT_X_LSB 0x02	// [7:0] are 8 MSBs of 16 bit X-axis data sample
#define OUT_Y_MSB 0x03	// [7:0] are 8 MSBs of 16 bit Y-axis data sample
#define OUT_Y_LSB 0x04	// [7:0] are 8 MSBs of 16 bit Y-axis data sample
#define OUT_Z_MSB 0x05	// [7:0] are 8 MSBs of 16 bit Z-axis data sample
#define OUT_Z_LSB 0x06	// [7:0] are 8 MSBs of 16 bit Z-axis data sample; \
						  // Auto-increment address depends on the setting made in CTRL_REG3[WRAPTOONE],(defaults to 0x00)
#define DR_STATUS 0x07	// Data-ready status information
#define F_STATUS 0x08	 // FIFO Status
#define F_SETUP 0x09	  // FIFO setup
#define F_EVENT 0x0A	  // FIFO event
#define INT_SRC_FLAG 0x0B // Interrupt event source status flags
#define WHO_AM_I 0x0C	 // Device ID
#define CTRL_GREG0 0x0D   // Control register 0: Full-scale range selection,high-pass filter setting, SPI mode selection
#define RT_CFG 0x0E		  // Rate threshold function configuration
#define RT_SRC 0x0F		  // Rate threshold event flags status register
#define RT_THS 0x10		  // Rate threshold function threshold register
#define RT_COUNT 0x11	 // Rate threshold function debounce counter
#define TEMP 0x12		  // Device temperature in �C
#define CTRL_GREG1 0x13   // Control register 1: Operating mode, ODR selection, self-test and soft reset
#define CTRL_GREG2 0x14   // Control register 2: Interrupt configuration settings
#define CTRL_GREG3 0x15   // Control Register 3: Auto-increment address configuration, external power control, FSR expansion

//-----------------------------------------------------------------------------
// Types
//-----------------------------------------------------------------------------
// Structure de lecture des acc�l�rations angulaires brutes
typedef struct
{
	float x;
	float y;
	float z;
} GyroRAWDATAEnum;

//-----------------------------------------------------------------------------
// FXAS21002C Setup
//-----------------------------------------------------------------------------
void mGyro_Setup(void);

//-----------------------------------------------------------------------------
// Open FXAS21002C
//-----------------------------------------------------------------------------
void mGyro_Open(void);

//-----------------------------------------------------------------------------
// Close FXAS21002C
//-----------------------------------------------------------------------------
void mGyro_Close(void);

//-----------------------------------------------------------------------------
// Reset FXAS21002C
//-----------------------------------------------------------------------------
void mGyro_Reset(void);

//-----------------------------------------------------------------------------
// Set a FXAS21002C register
//-----------------------------------------------------------------------------
bool mGyro_SetData(UInt8 aReg, UInt8 aVal[], UInt8 aSize, UInt8 aAddr);

//-----------------------------------------------------------------------------
// Read a FXAS21002C register
//-----------------------------------------------------------------------------
bool mGyro_GetData(UInt8 Reg, UInt8 aData[], UInt8 aSize, UInt8 aAddr);

//-----------------------------------------------------------------------------
// Init du FXAS21002C
//-----------------------------------------------------------------------------
bool mGyro_Init(void);

//-----------------------------------------------------------------------------
// Calibration de l'offset de l'acc�l�rom�tre
//-----------------------------------------------------------------------------
bool mGyro_Calibration(void);

//-----------------------------------------------------------------------------
// Lecture du gyroscope
//-----------------------------------------------------------------------------
bool mGyro_ReadData(float *aX, float *aY, float *aZ);

//-----------------------------------------------------------------------------
// Lecture du gyroscope avec correction en degr�s/s
//-----------------------------------------------------------------------------
bool mGyro_ReadData_mDPS(float *aX, float *aY, float *aZ);

#endif
