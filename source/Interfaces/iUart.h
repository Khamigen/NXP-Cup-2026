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
/*------------------------------------
File name : 	iUart.h	
Author and date :	Monnerat Serge 5 mars 201x

Goal : interface s�rie pour le module wifi

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $


-----------------------------------------------------------------------------
*/
#ifndef __iUart__
#define __iUart__

#include "Utils/def.h"

//------------------------------------------------------------
// UART status flag
//------------------------------------------------------------
typedef enum
{
	kSciTransmitDataRegEmpty = 0x80,
	kSciTransmitComplete = 0x40,
	kSciReceiveDataRegFull = 0x20,
	kSciIdleLineDetect = 0x10,
	kSciOverrunErr = 0x08,
	kSciNoiseErr = 0x04,
	kSciFramingErr = 0x02,
	kSciParityErr = 0x01
} UartStatusEnum;

//------------------------------------------------------------
// UART fifo status flag
//------------------------------------------------------------
typedef enum
{
	kTxFifoEmpty = 0x80,
	kRxFifoEmpty = 0x40,
	kTxFifoOverflow = 0x02,
	kRxFifoUnderflow = 0x01
} UartFifoStatusEnum;

typedef void (*iUart_OnCharReceived)(UINT8 aChar);

//-----------------------------------------------------------------------
// UART 3 module configuration
//-----------------------------------------------------------------------
void iUart_Config(void);

//-----------------------------------------------------------------------
// Reconfigure Uart baudrate for faster transfert debit
//-----------------------------------------------------------------------
void iUart_WifiChangeBaudrate(int aBaudrate);

//-----------------------------------------------------------------------
// Set rx callback listener
//-----------------------------------------------------------------------
void iUart_WifiSetRxListener(iUart_OnCharReceived aCallback);

//------------------------------------------------------------
// Get Uart3 flags state
// aStatus: which flag to read
// retour	: flag state
//------------------------------------------------------------
bool iUart_GetStatus(UartStatusEnum aStatus);

// Uart3 Data register write
// aData: datas to transmit
//------------------------------------------------------------
void iUart_SetData(UInt8 aData);

//------------------------------------------------------------
// Uart4 Data register write
// aData: datas to transmit
//------------------------------------------------------------
void iUart_Uart4SetData(UInt8 aData);

//------------------------------------------------------------
// Get Uart4 flags state
// aStatus: which flag to read
// retour	: flag state
//------------------------------------------------------------
bool iUart_Uart4GetStatus(UartStatusEnum aStatus);

//------------------------------------------------------------
// Uart3 Data register read
// aData: datas to transmit
//------------------------------------------------------------
UInt8 iUart_ReadData(void);
//------------------------------------------------------------
// Uart4 Data register read
// aData: datas to transmit
//------------------------------------------------------------
UInt8 iUart_Uart4ReadData(void);

//------------------------------------------------------------
// Configuration du DMA pour l'AD
//------------------------------------------------------------
void iUart_ConfigDMA(void);

#endif
