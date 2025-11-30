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
File name :	iI2C.c
Author and date :	Monnerat Serge 7 mars 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/
#include "iI2C.h"
#include "MK64F12.h"

//------------------------------------------------------------
// Constante
//------------------------------------------------------------
#define BUSSPEED 50000000
#define I2CBAUDRATE 400000

//------------------------------------------------------------
// I2C module config
//------------------------------------------------------------
void iI2C_Config(void)
{

	// I2C clock enable
	// System Clock Gating Control Register 4 (SIM_SCGC4)
	SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;

	// PTE24 --> SCL I2C0 --> Acc�l�rom�tre et magn�tom�tre FXOS8700CQ
	PORTE->PCR[24] = 0 | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK | (PORT_PCR_MUX(5));
	// PTE25 --> SDA I2C0 --> Acc�l�rom�tre et magn�tom�tre FXOS8700CQ
	PORTE->PCR[25] = 0 | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ODE_MASK | PORT_PCR_DSE_MASK | (PORT_PCR_MUX(5));

	// PTB2 --> IO input --> interrupt INT1 FXOS8700CQ
	PORTB->PCR[2] = 0 | PORT_PCR_PS_MASK | PORT_PCR_ISF_MASK | PORT_PCR_IRQC(0xA) | (PORT_PCR_MUX(1));
	// PTB3 --> IO input --> interrupt INT2 FXOS8700CQ
	PORTB->PCR[3] = 0 | PORT_PCR_PS_MASK | (PORT_PCR_MUX(1));

	// PTD0 --> IO input --> interrupt INT1 FXAS21002C
	PORTD->PCR[0] = 0 | PORT_PCR_PS_MASK | PORT_PCR_ISF_MASK | PORT_PCR_IRQC(0x9) | (PORT_PCR_MUX(1));
	// PTD1 --> IO input --> interrupt INT2 FXAS21002C
	PORTD->PCR[1] = 0 | PORT_PCR_PS_MASK | (PORT_PCR_MUX(1));

	// Baud rate speed and I2C timing
	// I2C Frequency Divider register (I2Cx_F)
	// I2C clock rate=390,625 kHz (max 400kHz)
	// SDA Hold = 0.42us (max 0.9us)
	// SCL start Hold = 1.16 us (min 0.6us)
	// SCL stop Hold = 1.3 us (min 0.6us
	I2C0->F = 0;
	I2C0->F |= I2C_F_ICR(0x17) | I2C_F_MULT(0);

	//	// Clear prending interrupt
	//	NVIC_ClearPendingIRQ(PORTB_IRQn);
	//	// Interrupt Set-enable Registers
	//	// Interrupt enable
	//	NVIC_EnableIRQ(PORTB_IRQn);
	//	// Interrupt Priority Registers
	//	// Set interrupt priority
	//	NVIC_SetPriority(PORTB_IRQn,kPORB_IntPriority);
}

//------------------------------------------------------------
// Gyro interrupt enable
//------------------------------------------------------------
void iI2C_GyroIntEnable(void)
{
	// Clear prending interrupt
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	// Interrupt Set-enable Registers
	// Interrupt enable
	NVIC_EnableIRQ(PORTD_IRQn);
	// Interrupt Priority Registers
	// Set interrupt priority
	NVIC_SetPriority(PORTD_IRQn, kPORD_IntPriority);
}

//------------------------------------------------------------
// I2C interface enable
//------------------------------------------------------------
void iI2C_Enable(void)
{

	// I2C0 enable
	// I2C Control Register 1 (I2Cx_C1)
	I2C0->C1 |= I2C_C1_IICEN_MASK;
}

//------------------------------------------------------------
// I2C interface disable
//------------------------------------------------------------
void iI2C_stop(void)
{
	// I2C0 disable
	// I2C Control Register 1 (I2Cx_C1)
	I2C0->C1 &= (~I2C_C1_IICEN_MASK);
}

//------------------------------------------------------------
// Set START state
//------------------------------------------------------------
void iI2C_SetStartState(void)
{
	// I2C Control Register 1 (I2Cx_C1)
	I2C0->C1 |= I2C_C1_MST_MASK;
}

//------------------------------------------------------------
// Set repeated START state
//------------------------------------------------------------
void iI2C_SetRepeatedStartSate(void)
{
	// I2C Control Register 1 (I2Cx_C1)
	I2C0->C1 |= I2C_C1_RSTA_MASK;
}

//------------------------------------------------------------
// Set STOP state
//------------------------------------------------------------
void iI2C_SetStopState(void)
{
	// I2C Control Register 1 (I2Cx_C1)
	I2C0->C1 &= (~I2C_C1_MST_MASK);
}

//------------------------------------------------------------
// Generate automatic ACK or not
//------------------------------------------------------------
void iI2C_SetAckMode(I2CAckEnum aAck)
{

	if (aAck == kAckAuto)
	{
		// I2C Control Register 1 (I2Cx_C1)
		I2C0->C1 &= (~I2C_C1_TXAK_MASK);
	}
	else if (aAck == kNoAck)
	{
		// I2C Control Register 1 (I2Cx_C1)
		I2C0->C1 |= I2C_C1_TXAK_MASK;
	}
}

//------------------------------------------------------------
// Select if we transmit or receive
//------------------------------------------------------------
void iI2C_TxRxSelect(I2CTransmiteModeEnum aMode)
{
	if (kTxMode == aMode)
	{
		// TX
		I2C0->C1 |= I2C_C1_TX_MASK;
	}
	else if (kRxMode == aMode)
	{
		// RX
		I2C0->C1 &= ~I2C_C1_TX_MASK;
	}
}

//------------------------------------------------------------
// Send a data
//------------------------------------------------------------
void iI2C_SendData(UInt8 aData)
{
	// I2C Data I/O register (I2Cx_D)
	I2C0->D = aData;
}

//------------------------------------------------------------
// Wait End of transmit or receive
//------------------------------------------------------------
void iI2C_WaitEndOfRxOrTx(void)
{
	// Wait for IICIF flag
	// I2C Status register (I2Cx_S)
	while ((I2C0->S & I2C_S_IICIF_MASK) == 0)
		;
	// Clear the IICIF flag
	I2C0->S |= I2C_S_IICIF_MASK;
}

//------------------------------------------------------------
// Read received data, ! generate I2C clock if not in STOP mode
//------------------------------------------------------------
UInt8 iI2C_ReadData(void)
{
	UInt8 aReturnData = 0;

	// I2C Data I/O register (I2Cx_D)
	aReturnData = I2C0->D;

	return aReturnData;
}

//------------------------------------------------------------
// Get I2C status flags
//------------------------------------------------------------
bool iI2C_ReadStatus(I2CStatusEnum aStatus)
{
	return (bool)(((I2C0->S & aStatus) == aStatus));
}
