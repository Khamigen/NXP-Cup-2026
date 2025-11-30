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
Nom du fichier : 	iSpi.h	
Auteur et Date :	Monnerat Serge 29 ao�t 201x

But : Interface permettant l'utilisation du SPI

-----------------------------------------------------------------------------
Historique:
-----------------------------------------------------------------------------

$History: $


-----------------------------------------------------------------------------
*/

#ifndef __ISPI__
#define __ISPI__

#include "Utils/def.h"

//-----------------------------------------------------------------------
// Types
//-----------------------------------------------------------------------

// Each K10 SPI
typedef enum
{
	kSpi0,
	kSpi1,
	kSpi2
} SPIEnum;

// Avalaible SPI chip select
typedef enum
{
	kCS0 = 0x00010000,
	kCS1 = 0x00020000,
	kCS2 = 0x00040000,
	kCS3 = 0x00080000,
	kCS4 = 0x00100000,
	kCS5 = 0x00200000
} CsEnum;

// SPI status flag
typedef enum
{
	kTCF = 0x80000000,   // Transfer Complete Flag
	kTXRXS = 0x40000000, // TX and RX Status --> 1 Transmit and receive operations are enabled (DSPI is in running state)
	kEOQF = 0x10000000,  // End of Queue Flag --> TX FIFO
	kTFUF = 0x08000000,  // Transmit FIFO Underflow Flag --> 0 No Tx FIFO underflow
	kTFFF = 0x02000000,  // Transmit FIFO Fill Flag --> 0 Tx FIFO is full
	kRFOF = 0x00080000,  // Receive FIFO Overflow Flag
	kRFDF = 0x00020000   // Receive FIFO Drain Flag --> 0 Rx FIFO is empty
} SpiStatusEnum;

// Spi config register avalaible
typedef enum
{
	kCTAR0 = 0x00000000,
	kCTAR1 = 0x10000000,
} SpiCfgRegisterEnum;

// CS continuous mode
typedef enum
{
	kCONTEn = 0x80000000,
	kCONTDis = 0x00000000
} CSContinuousModeEnum;

// Clock phase enum
typedef enum
{
	kCaptureLeadingEdge,
	kDataChangeLeadingEdge
} ClockPhaseEnum;

//-----------------------------------------------------------------------
// Setup du clock et de la fonction des pins
//-----------------------------------------------------------------------
void iSpi_ConfigClockAndPins(void);

//-----------------------------------------------------------------------
// SPI module configuration
// aSpi:which SPI (0,1 or 2)
//-----------------------------------------------------------------------
void iSpi_ConfigSPIModule(SPIEnum aSpi);

//-----------------------------------------------------------------------
// SPI clock and transfert configuration
// aSpi:which SPI (0,1 or 2)
// aCfg:which config register (CTAR0 or CTAR1)
//-----------------------------------------------------------------------
void iSpi_ConfigClockAndTransfert(SPIEnum aSpi, SpiCfgRegisterEnum aCfg);

//-----------------------------------------------------------------------
// SPI config interrupt & DMA
// aSpi:which SPI (0,1 or 2)
//-----------------------------------------------------------------------
void iSpi_ConfigInterrupAndDMA(SPIEnum aSpi);

//-----------------------------------------------------------------------
// SPI clock phase configuration
// aSpi:which SPI (0,1 or 2)
// aCfg:which config register (CTAR0 or CTAR1)
//-----------------------------------------------------------------------
void iSpi_ConfigClockPhase(SPIEnum aSpi, SpiCfgRegisterEnum aCfg, ClockPhaseEnum aPhase);

//-----------------------------------------------------------------------
// SPI CS inactive state
// aSpi:which SPI (0,1 or 2)
// aCs: which Cs
// aInactState: true -> inactive state is 1
//-----------------------------------------------------------------------
void iSpi_ChangeInactiveCSState(SPIEnum aSpi, CsEnum aCs, bool aInactState);

//-----------------------------------------------------------------------
// Clear TX FIFO
// aSpi:		which SPI
//-----------------------------------------------------------------------
void iSpi_ClearTxFifo(SPIEnum aSpi);

//-----------------------------------------------------------------------
// Flush RX FIFO
// aSpi:		which SPI
//-----------------------------------------------------------------------
void iSpi_FlushRXFifo(SPIEnum aSpi);

//-----------------------------------------------------------------------
// Start TX
// aSpi:which SPI
//-----------------------------------------------------------------------
void iSpi_StartTX(SPIEnum aSpi);

//-----------------------------------------------------------------------
// Stop TX
// aSpi:which SPI
//-----------------------------------------------------------------------
void iSpi_StopTX(SPIEnum aSpi);

//-----------------------------------------------------------------------------
// Get SPI status
// aSpi:which SPI
// aStatus: which status flag
// return	: flag state
//-----------------------------------------------------------------------------
bool iSpi_GetStatus(SPIEnum aSpi, SpiStatusEnum aStatus);

//-----------------------------------------------------------------------------
// Get TX FIFO counter
// aSpi:which SPI
// return	: Tx fifo counter
//-----------------------------------------------------------------------------
UInt8 iSpi_GetTxFifoCounter(SPIEnum aSpi);

//-----------------------------------------------------------------------------
// Get TX FIFO next pointer
// aSpi:which SPI
// return	: Tx fifo next pointer
//-----------------------------------------------------------------------------
UInt8 iSpi_GetTxFifoNextPointer(SPIEnum aSpi);

//-----------------------------------------------------------------------------
// Get RX FIFO Counter
// aSpi:which SPI
// return	: Rx fifo counter
//-----------------------------------------------------------------------------
UInt8 iSpi_GetRxFifoCounter(SPIEnum aSpi);

//-----------------------------------------------------------------------------
// Get RX FIFO pop next pointer
// aSpi:which SPI
// return	: Rx fifo pop next pointer
//-----------------------------------------------------------------------------
UInt8 iSpi_GetRxFifoNextPointer(SPIEnum aSpi);

//-----------------------------------------------------------------------------
// Spi status flag reset
// aSpi:which SPI
// aStatus: which flag to reset
//-----------------------------------------------------------------------------
void iSpi_ResetStatus(SPIEnum aSpi, SpiStatusEnum aStatus);

//-----------------------------------------------------------------------------
// Push Data & Config into the TX FIFO
// aSpi:which SPI
// aCfg:which config register (CTAR0 or CTAR1)
// aCont: CS remain asserted?
// aLastQueueData: is this frame the last in the queue?
// aCs: which CS to use
// aData: data to transmit
//-----------------------------------------------------------------------------
void iSpi_PushDataIntoFifo(SPIEnum aSpi, SpiCfgRegisterEnum aCfg,
						   CSContinuousModeEnum aCont, bool aLastQueueData,
						   CsEnum aCs, UInt16 aData);

//-----------------------------------------------------------------------------
// Read a Data from the RX queue
// aSpi:which SPI
//-----------------------------------------------------------------------------
UInt32 iSpi_ReadData(SPIEnum aSpi);

#endif
