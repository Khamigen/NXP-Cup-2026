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
File name :	iUart.c
Author and date :	Monnerat Serge 10.09.201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
 */

#include "iUart.h"
#include "MK64F12.h"

//-----------------------------------------------------------------------
// Constantes
//-----------------------------------------------------------------------
#define kUart3BaudRate 115200
#define kUart4BaudRate 115200

// Singleton driver instance
typedef struct sUART
{
	iUart_OnCharReceived rxCallback;
} sUART;

static sUART gUart3Instance;
static sUART *gUart3 = &gUart3Instance;

static sUART gUart4Instance;
static sUART *gUart4 = &gUart4Instance;

//-----------------------------------------------------------------------
// UART 3 module configuration --> wifi
//-----------------------------------------------------------------------
void iUart_Config(void)
{
	UInt16 aSbr;
	UInt8 aTmp;
	UInt16 aBrfa;

	// UART Control Register 2 (UARTx_C2)
	// Disable TX & RX before setup
	UART3->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

	// IRQ Config
	// Interrupt Clear-pending Registers
	// Clear prending interrupt
	NVIC_ClearPendingIRQ(UART3_RX_TX_IRQn);
	// Interrupt Set-enable Registers
	// Interrupt enable
	NVIC_EnableIRQ(UART3_RX_TX_IRQn);
	// Interrupt Priority Registers
	// Set interrupt priority
	NVIC_SetPriority(UART3_RX_TX_IRQn, kUART3_IntPriority);

	// UART Control Register 1 (UARTx_C1)
	// Loop Mode Select=0,Normal operation
	// UARTSWAI=0, UART clock continues to run in wait mode
	// RSRC=0,Selects internal loop back mode and receiver input is internally connected to transmitter output
	// M=0, Normal - start + 8 data bits (MSB/LSB first as determined by MSBF) + stop
	// WAKE=0,Idle-line wakeup
	// ILT=0,Idle character bit count starts after start bit
	// PE=0,Parity function disabled
	// PT=0,Even parity
	UART3->C1 = 0; /* We need all default settings, so entire register is cleared */

	// UART Baud Rate Registers:High (UARTx_BDH)
	// UART Baud Rate Registers: Low (UARTx_BDL)
	// Calculate baud rate settings
	aSbr = (UInt16)((kClockPeriphkHz * 1000) / (kUart3BaudRate * 16));
	// Save off the current value of the UARTx_BDH except for the SBR field
	aTmp = UART3->BDH & ~(UART_BDH_SBR(0x1F));
	UART3->BDH = aTmp | UART_BDH_SBNS_MASK | UART_BDH_SBR(((aSbr & 0x1F00) >> 8));
	UART3->BDL = (UInt8)(aSbr & UART_BDL_SBR_MASK);

	// UART Control Register 4 (UARTx_C4)
	// BRFA, Baud Rate Fine Adjust
	// Determine if a fractional divider is needed to get closer to the baud rate
	aBrfa = (((kClockPeriphkHz * 32000) / (kUart4BaudRate * 16)) - (aSbr * 32));
	// Save off the current value of the UARTx_C4 register except for the BRFA field */
	aTmp = UART3->C4 & ~(UART_C4_BRFA(0x1F));
	UART3->C4 = aTmp | UART_C4_BRFA(aBrfa);

	// UART FIFO Parameters (UARTx_PFIFO)
	// TXFE=1,Transmit FIFO is enabled. Buffer is depth indicted by TXFIFOSIZE
	// TXFIFOSIZE=3,Transmit FIFO/Buffer Depth = 16 Datawords
	UART3->PFIFO = 0;
	UART3->PFIFO |= UART_PFIFO_TXFE_MASK;
	UART3->PFIFO |= UART_PFIFO_TXFIFOSIZE(3);

	// Set level to trigger TDRE flag whenever there is space in the TXFIFO
	// TXFIFOSIZE=3,Transmit FIFO/Buffer Depth = 16 Datawords
	UART3->PFIFO |= UART_PFIFO_RXFE_MASK;
	uint8_t txfifo_size =
		(2 << ((UART3->PFIFO & UART_PFIFO_TXFIFOSIZE_MASK) >>
			   UART_PFIFO_TXFIFOSIZE_SHIFT));
	UART3->TWFIFO = UART_TWFIFO_TXWATER(txfifo_size - 1);

	/* Trigger RX interrupt when there is 1 byte or more in the RXFIFO */
	UART3->RWFIFO = 1;

	// UART FIFO Control Register (UARTx_CFIFO)
	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1283
	// Flush RX & TX fifo
	UART3->CFIFO |= (UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK);

	// UART Control Register 2 (UARTx_C2)
	// TIE=0, TDRE interrupt and DMA transfer requests disabled
	// TCIE=0, TC interrupt requests disabled
	// RIE=1,RDRF interrupt or DMA transfer requests enabled
	// ILIE=0,IDLE interrupt requests disabled
	// TE & RE =1, Enable receiver and transmitter
	UART3->C2 = 0;
	UART3->C2 |= UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK;

	// Interrupt Set-enable Registers
	// Interrupt enable
	NVIC_EnableIRQ(UART3_RX_TX_IRQn);

	// UART Control Register 2 (UARTx_C2)
	// Disable TX & RX before setup
	UART4->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

	// IRQ Config
	// Interrupt Clear-pending Registers
	// Clear prending interrupt
	NVIC_ClearPendingIRQ(UART4_RX_TX_IRQn);

	// Interrupt Priority Registers
	// Set interrupt priority
	NVIC_SetPriority(UART4_RX_TX_IRQn, kUART4_IntPriority);

	// UART Control Register 1 (UARTx_C1)
	// Loop Mode Select=0,Normal operation
	// UARTSWAI=0, UART clock continues to run in wait mode
	// RSRC=0,Selects internal loop back mode and receiver input is internally connected to transmitter output
	// M=0, Normal - start + 8 data bits (MSB/LSB first as determined by MSBF) + stop
	// WAKE=0,Idle-line wakeup
	// ILT=0,Idle character bit count starts after start bit
	// PE=0,Parity function disabled
	// PT=0,Even parity
	UART4->C1 = 0; /* We need all default settings, so entire register is cleared */

	// UART Baud Rate Registers:High (UARTx_BDH)
	// UART Baud Rate Registers: Low (UARTx_BDL)
	// Calculate baud rate settings
	aSbr = (UInt16)((kClockPeriphkHz * 1000) / (kUart4BaudRate * 16));
	// Save off the current value of the UARTx_BDH except for the SBR field
	aTmp = UART4->BDH & ~(UART_BDH_SBR(0x1F));
	UART4->BDH = aTmp | UART_BDH_SBNS_MASK | UART_BDH_SBR(((aSbr & 0x1F00) >> 8));
	UART4->BDL = (UInt8)(aSbr & UART_BDL_SBR_MASK);

	// UART Control Register 4 (UARTx_C4)
	// BRFA, Baud Rate Fine Adjust
	// Determine if a fractional divider is needed to get closer to the baud rate
	aBrfa = (((kClockPeriphkHz * 32000) / (kUart4BaudRate * 16)) - (aSbr * 32));
	// Save off the current value of the UARTx_C4 register except for the BRFA field */
	aTmp = UART4->C4 & ~(UART_C4_BRFA(0x1F));
	UART4->C4 = aTmp | UART_C4_BRFA(aBrfa);

	// UART FIFO Parameters (UARTx_PFIFO)
	// TXFE=1,Transmit FIFO is enabled. Buffer is depth indicted by TXFIFOSIZE
	// TXFIFOSIZE=3,Transmit FIFO/Buffer Depth = 16 Datawords
	UART4->PFIFO = 0;
	UART4->PFIFO |= (UART_PFIFO_TXFE_MASK | UART_PFIFO_TXFIFOSIZE(3));

	// UART FIFO Control Register (UARTx_CFIFO)
	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1283
	// Flush RX & TX fifo
	UART4->CFIFO |= (UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK);

	// UART Control Register 2 (UARTx_C2)
	// TIE=0, TDRE interrupt and DMA transfer requests disabled
	// TCIE=0, TC interrupt requests disabled
	// RIE=1,RDRF interrupt or DMA transfer requests enabled
	// ILIE=0,IDLE interrupt requests disabled
	// TE & RE =1, Enable receiver and transmitter
	UART4->C2 = 0;
	UART4->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);

	// Interrupt Set-enable Registers
	// Interrupt enable
	NVIC_EnableIRQ(UART4_RX_TX_IRQn);
	// Enable de l'interruption
	//enable_irq(INT_UART4_RX_TX-16);
	//set_irq_priority (INT_UART4_RX_TX-16, 0);
}

void iUart_WifiChangeBaudrate(int aBaudrate)
{
	UInt16 aSbr;
	UInt8 aTmp;
	UInt16 aBrfa;

	// UART Control Register 2 (UARTx_C2)
	// Disable TX & RX before setup
	UART3->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

	// UART Baud Rate Registers:High (UARTx_BDH)
	// UART Baud Rate Registers: Low (UARTx_BDL)
	// Calculate baud rate settings
	aSbr = (UInt16)((kClockPeriphkHz * 1000) / (aBaudrate * 16));
	// Save off the current value of the UARTx_BDH except for the SBR field
	aTmp = UART3->BDH & ~(UART_BDH_SBR(0x1F));
	UART3->BDH = aTmp | UART_BDH_SBNS_MASK | UART_BDH_SBR(((aSbr & 0x1F00) >> 8));
	UART3->BDL = (UInt8)(aSbr & UART_BDL_SBR_MASK);

	// UART Control Register 4 (UARTx_C4)
	// BRFA, Baud Rate Fine Adjust
	// Determine if a fractional divider is needed to get closer to the baud rate
	aBrfa = (((kClockPeriphkHz * 32000) / (aBaudrate * 16)) - (aSbr * 32));
	// Save off the current value of the UARTx_C4 register except for the BRFA field */
	aTmp = UART3->C4 & ~(UART_C4_BRFA(0x1F));
	UART3->C4 = aTmp | UART_C4_BRFA(aBrfa);

	/* Trigger RX interrupt when there is 1 byte or more in the RXFIFO */
	UART3->RWFIFO = 1;
	// UART FIFO Control Register (UARTx_CFIFO)
	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1283
	// Flush RX & TX fifo
	UART3->CFIFO |= (UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK);

	UART3->C2 |= UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK;
}

void iUart_WifiSetRxListener(iUart_OnCharReceived aCallback)
{
	gUart3->rxCallback = aCallback;
}

//------------------------------------------------------------
// Get Uart3 flags state
// aStatus: which flag to read
// retour	: flag state
//------------------------------------------------------------
bool iUart_GetStatus(UartStatusEnum aStatus)
{
	// UART Status Register 1 (UARTx_S1)
	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1267
	return ((UART3->S1 & aStatus) == aStatus);
}

//------------------------------------------------------------
// Uart3 Data register write
// aData: data to transmit
//------------------------------------------------------------
void iUart_SetData(UInt8 aData)
{
	// UART Data Register (UARTx_D)
	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1274
	UART3->D = aData;
}

//---------------------------------------------------------------------------
// RX interrupt wifi
//---------------------------------------------------------------------------
void UART3_RX_TX_IRQHandler(void)
{
	Int8 aVal;
	UInt8 sStatus;
	UInt8 dummy;

	sStatus = UART3->S1;

	// Clear overrun issues
	if (sStatus & UART_S1_OR_MASK)
	{
		sStatus = UART3->S1;
		dummy = UART3->D;
		dummy++;
	}

	if (sStatus & UART_S1_RDRF_MASK)
	{
		// Read the RX byte
		aVal = UART3->D;

		// If the callback listener is registered call it
		if (gUart3->rxCallback)
		{
			gUart3->rxCallback(aVal);
		}
	}
}

//------------------------------------------------------------
// Uart4 Data register write
// aData: datas to transmit
//------------------------------------------------------------
void iUart_Uart4SetData(UInt8 aData)
{
	// UART Data Register (UARTx_D)
	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1274
	UART4->D = aData;
}

//------------------------------------------------------------
// Get Uart4 flags state
// aStatus: which flag to read
// retour	: flag state
//------------------------------------------------------------
bool iUart_Uart4GetStatus(UartStatusEnum aStatus)
{
	// UART Status Register 1 (UARTx_S1)
	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1267
	return ((UART4->S1 & aStatus) == aStatus);
}

//---------------------------------------------------------------------------
// RX interrupt wifi
//---------------------------------------------------------------------------
void UART4_RX_TX_IRQHandler(void)
{
	Int8 aVal;
	UInt8 sStatus;
	UInt8 dummy;

	sStatus = UART4->S1;

	// Clear overrun issues
	if (sStatus & UART_S1_OR_MASK)
	{
		sStatus = UART4->S1;
		dummy = UART4->D;
		dummy++;
	}

	if (sStatus & UART_S1_RDRF_MASK)
	{
		// Read the RX byte
		aVal = UART4->D;

		// If the callback listener is registered call it
		if (gUart4->rxCallback)
		{
			gUart4->rxCallback(aVal);
		}
	}
}
//------------------------------------------------------------
// Uart4 Data register read
// aData: datas to transmit
//------------------------------------------------------------
UInt8 iUart_Uart4ReadData(void)
{
	return UART4->D;
}
