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
Nom du fichier : 	def.h	
Auteur et Date :	Monnerat Serge 29 ao�t 2013

But : Some general settings

-----------------------------------------------------------------------------
Historique:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#ifndef DEF_H_
#define DEF_H_

#include <stdbool.h>

//------------------------------------------------------------
// Constants
//------------------------------------------------------------
// Core clock
#define kClockCpu 100.0			// 100MHz
#define kClockCpukHz 100000.0   // 100MHz
#define kClockPeriphkHz 50000.0 // 50MHz
#define CORE_CLOCK (100000000)
#define PERIPHERAL_BUS_CLOCK (CORE_CLOCK / 2)

// Cam�ra digitale ou analogique
#define kDigCamera 1
// PIT1 period --> 1 ms
#define kPit1Period 1
#define kPit2Period 1
// Avec ou sans acc�l�rom�tre, magn�to
#define kWithAccel 0

// Avec ou sans Lidar
#define kWithLidar 0

//------------------------------------------------------------
// Interrupt priority (O highest, 15 lowest)
//------------------------------------------------------------
#define kPIT_IntPriority 15
#define kUART3_IntPriority 0
#define kUART4_IntPriority 0
#define kFTM1_IntPriority 1
#define kPORB_IntPriority 2
#define kPORD_IntPriority 2

//------------------------------------------------------------
// bool and standard type
//------------------------------------------------------------
//typedef int bool;
//typedef int bool;
typedef bool BOOL;
#define TRUE 1
#define true 1
#define FALSE 0
#define false 0

typedef unsigned char UInt8;	   /*  8 bits */
typedef unsigned short int UInt16; /* 16 bits */
typedef unsigned long int UInt32;  /* 32 bits */
typedef unsigned long long UInt64; /* 32 bits */

typedef char Int8;		 /*  8 bits */
typedef short int Int16; /* 16 bits */
typedef int Int32;		 /* 32 bits */

typedef volatile Int8 VInt8;   /*  8 bits */
typedef volatile Int16 VInt16; /* 16 bits */
typedef volatile Int32 VInt32; /* 32 bits */
typedef long long Int64;	   /* 32 bits */

typedef volatile UInt8 VUInt8;   /*  8 bits */
typedef volatile UInt16 VUInt16; /* 16 bits */
typedef volatile UInt32 VUInt32; /* 32 bits */

typedef volatile UInt8 VUInt8;   /*  8 bits */
typedef volatile UInt16 VUInt16; /* 16 bits */
typedef volatile UInt32 VUInt32; /* 32 bits */
typedef volatile UInt32 UINT32;
typedef volatile UInt8 UINT8;
typedef volatile Int8 INT8;
typedef volatile UInt16 UINT16;
typedef volatile UInt64 UINT64;
typedef volatile Int64 INT64;
typedef volatile Int16 INT16;
typedef volatile Int32 INT32;
typedef volatile Int8 CHAR;

// Macros min & max
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) < (b) ? (b) : (a))

//------------------------------------------------------------
// Type
//------------------------------------------------------------
// PIT enum
typedef enum
{
	kPit0 = 0,
	kPit1 = 1,
	kPit2 = 2,
	kPit3 = 3
} PitEnum;

// Structure contenant une frame re�ue par wifi
typedef struct
{
	UInt8 Type;
	UInt16 Size;
	UInt8 Payload[300];
	UInt16 Index;
} ReadFramestruct;

#endif
