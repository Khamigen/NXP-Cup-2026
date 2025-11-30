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
File name :	iSpi.c
Author and date :	Monnerat Serge 29 ao�t 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "iSpi.h"
#include "MK64F12.h"

//-----------------------------------------------------------------------
// Setup du clock et de la fonction des pins
//-----------------------------------------------------------------------
void iSpi_ConfigClockAndPins(void)
{
	// Setup des clock SPI0 et 1
	SIM->SCGC6 |= SIM_SCGC6_SPI1_MASK;
	SIM->SCGC6 |= SIM_SCGC6_SPI0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

	// Setup Pins as SPI signals sauf CS4 --> cam�ra 2
	// CS4 as dig out --> transmission sur 24 bits
	PORTC->PCR[0] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	PORTC->PCR[5] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;
	PORTC->PCR[6] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;
	PORTC->PCR[7] = PORT_PCR_MUX(2) | PORT_PCR_DSE_MASK;
	// Setup Pins as SPI signals sauf CS4 --> cam�ra 1
	// CS4 as dig out --> transmission sur 24 bits
	PORTD->PCR[4] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	PORTD->PCR[5] = PORT_PCR_MUX(7) | PORT_PCR_DSE_MASK;
	PORTD->PCR[6] = PORT_PCR_MUX(7) | PORT_PCR_DSE_MASK;
	PORTD->PCR[7] = PORT_PCR_MUX(7) | PORT_PCR_DSE_MASK;
	// IO input Frame ready --> camera dig 2
	PORTD->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	// IO input Frame ready --> camera dig 1
	PORTD->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;

	// Reset  et interrupt module wifi
	PORTC->PCR[13] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	PORTC->PCR[18] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;

	// PPTC0 --> CS4 --> camera 2 = sortie
	GPIOC->PDDR |= (1 << 0);

	// PPTC13 --> Reset wifi --> sortie
	GPIOC->PDDR |= (1 << 13);

	// PPTC18 --> Interrupt wifi --> entr�e
	GPIOC->PDDR &= (~(1 << 18));

	// PPTD2 --> camera --> Frame ready = entr�e
	GPIOD->PDDR &= (~(1 << 2));

	// PPTD4 --> CS0 --> camera 1 = sortie
	GPIOD->PDDR |= (1 << 4);

	// CS4 camera 2 � 1
	GPIOC->PSOR |= (1 << 0);

	// Reset wifi � 1 --> pas de reset
	GPIOC->PSOR |= (1 << 13);

	// CS0 camera 1 � 1
	GPIOD->PSOR |= (1 << 4);
}

//-----------------------------------------------------------------------
// SPI module configuration
// aSpi:which SPI (0,1 or 2)
//-----------------------------------------------------------------------
void iSpi_ConfigSPIModule(SPIEnum aSpi)
{
	switch (aSpi)
	{
	case kSpi0:
		// DSPI Module Configuration Register (SPIx_MCR)
		// Master/Slave Mode Select --> master
		// Continuous SCK Enable --> disabled
		// DSPI Configuration --> SPI
		// Freeze --> Do not halt serial transfers in debug mode
		// Modified Timing Format Enable --> Modified SPI transfer format disabled
		// Peripheral Chip Select Strobe Enable --> PCS[5]/PCSS is used as the Peripheral Chip Select[5] signal
		// Receive FIFO Overflow Overwrite Enable --> Incoming data is ignored
		// Peripheral Chip Select x Inactive State --> The inactive state of PCSx is high
		// Doze Enable --> Doze mode has no effect on DSPI
		// Module Disable --> Enable DSPI clocks
		// Disable Transmit FIFO --> Tx FIFO is enabled
		// Disable Receive FIFO --> Rx FIFO is enabled
		// Clear TX FIFO --> Do not clear the Tx FIFO counter
		// Flushes the RX FIFO --> Do not clear the Rx FIFO counter
		// Sample Point --> 0 system clocks between SCK edge and SIN sample
		// Halt --> Stop transfers.
		// Reset fields
		SPI0->MCR &= 0x0000000;
		SPI0->MCR |= ((SPI_MCR_MSTR_MASK) | (SPI_MCR_PCSIS(7)) | (SPI_MCR_HALT_MASK));

		break;
	case kSpi1:
		// DSPI Module Configuration Register (SPIx_MCR)
		// Master/Slave Mode Select --> master
		// Continuous SCK Enable --> disabled
		// DSPI Configuration --> SPI
		// Freeze --> Do not halt serial transfers in debug mode
		// Modified Timing Format Enable --> Modified SPI transfer format disabled
		// Peripheral Chip Select Strobe Enable --> PCS[5]/PCSS is used as the Peripheral Chip Select[5] signal
		// Receive FIFO Overflow Overwrite Enable --> Incoming data is ignored
		// Peripheral Chip Select x Inactive State --> The inactive state of PCSx is high
		// Doze Enable --> Doze mode has no effect on DSPI
		// Module Disable --> Enable DSPI clocks
		// Disable Transmit FIFO --> Tx FIFO is enabled
		// Disable Receive FIFO --> Rx FIFO is enabled
		// Clear TX FIFO --> Do not clear the Tx FIFO counter
		// Flushes the RX FIFO --> Do not clear the Rx FIFO counter
		// Sample Point --> 0 system clocks between SCK edge and SIN sample
		// Halt --> Stop transfers.
		// Reset fields
		SPI1->MCR &= 0x0000000;
		SPI1->MCR |= ((SPI_MCR_MSTR_MASK) | (SPI_MCR_PCSIS(7)) | (SPI_MCR_HALT_MASK));
		break;
	case kSpi2:
		// DSPI Module Configuration Register (SPIx_MCR)
		// Master/Slave Mode Select --> master
		// Continuous SCK Enable --> disabled
		// DSPI Configuration --> SPI
		// Freeze --> Do not halt serial transfers in debug mode
		// Modified Timing Format Enable --> Modified SPI transfer format disabled
		// Peripheral Chip Select Strobe Enable --> PCS[5]/PCSS is used as the Peripheral Chip Select[5] signal
		// Receive FIFO Overflow Overwrite Enable --> Incoming data is ignored
		// Peripheral Chip Select x Inactive State --> The inactive state of PCSx is high
		// Doze Enable --> Doze mode has no effect on DSPI
		// Module Disable --> Enable DSPI clocks
		// Disable Transmit FIFO --> Tx FIFO is enabled
		// Disable Receive FIFO --> Rx FIFO is enabled
		// Clear TX FIFO --> Do not clear the Tx FIFO counter
		// Flushes the RX FIFO --> Do not clear the Rx FIFO counter
		// Sample Point --> 0 system clocks between SCK edge and SIN sample
		// Halt --> Stop transfers.
		// Reset fields
		SPI2->MCR &= 0x0000000;
		SPI2->MCR |= ((SPI_MCR_MSTR_MASK) | (SPI_MCR_PCSIS(1)) | (SPI_MCR_HALT_MASK));
		break;
	}
}

//-----------------------------------------------------------------------
// SPI CS inactive state
// aSpi:which SPI (0,1 or 2)
// aCs: which Cs
// aInactState: true -> inactive state is 1
//-----------------------------------------------------------------------
void iSpi_ChangeInactiveCSState(SPIEnum aSpi, CsEnum aCs, bool aInactState)
{
	switch (aSpi)
	{
	case kSpi0:
		// DSPI Module Configuration Register (SPIx_MCR)
		// Peripheral Chip Select x Inactive State
		if (aInactState == true)
		{
			SPI0->MCR |= aCs;
		}
		else
		{
			SPI0->MCR &= (~aCs);
		}
		break;
	case kSpi1:
		// DSPI Module Configuration Register (SPIx_MCR)
		// Peripheral Chip Select x Inactive State
		if (aInactState == true)
		{
			SPI1->MCR |= aCs;
		}
		else
		{
			SPI1->MCR &= (~aCs);
		}
		break;
	case kSpi2:
		// DSPI Module Configuration Register (SPIx_MCR)
		// Peripheral Chip Select x Inactive State
		if (aInactState == true)
		{
			SPI0->MCR |= aCs;
		}
		else
		{
			SPI0->MCR &= (~aCs);
		}
		break;
	}
}

//-----------------------------------------------------------------------
// SPI clock and transfer configuration
// aSpi:which SPI (0,1 or 2)
// aCfg:which config register (CTAR0 or CTAR1)
//-----------------------------------------------------------------------
void iSpi_ConfigClockAndTransfert(SPIEnum aSpi, SpiCfgRegisterEnum aCfg)
{
	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1174
	// DSPI Clock and Transfer Attributes Register
	switch (aSpi)
	{
	case kSpi0:
		if (kCTAR0 == aCfg)
		{
			// Fields reset
			SPI0->CTAR[0] &= 0x00000000;
			// Double Baud Rate --> The baud rate is computed normally with a 50/50 duty cycle
			// Frame Size --> 8 bits
			// Clock Polarity --> The inactive state value of SCK is low
			// Clock Phase --> Data is captured on the leading edge of SCK and changed on the following edge
			// LBS First --> Data is transferred LSB first
			// PCSSCK: PCS to SCK Delay Prescaler --> PCS to SCK Prescaler value is 1
			// PASC:After SCK Delay Prescaler --> Delay after Transfer Prescaler value is 1
			// PDT:Delay after Transfer Prescaler --> Delay after Transfer Prescaler value is 1
			// CSSCK:PCS to SCK Delay Scaler --> tCSC = (1/fSYS) x PCSSCK x CSSCK --> (1/50MHz)*1*2
			// ASC: After SCK Delay Scaler --> tASC = (1/fSYS) x PASC x ASC --> (1/50MHz)*1*1
			// DT:Delay After Transfer Scaler --> tDT = (1/fSYS) x PDT x DT --> (1/50MHz)*1*1
			// Baud Rate config
			// SCK baud rate = (fSYS/PBR) x [(1+DBR)/BR]
			// with fsys=fbus=50MHz
			// PBR = 0 div by 2
			// DBR = 0 (50% duty cycle)
			// BR = 0 --> div by 2
			// SPI Clock = (50MHz/2) * (1/2)=12.5MHz
			SPI0->CTAR[0] |= ((SPI_CTAR_FMSZ(7)) | SPI_CTAR_CPOL_MASK | SPI_CTAR_CPHA_MASK | SPI_CTAR_LSBFE_MASK | (SPI_CTAR_ASC(1)) | (SPI_CTAR_DT(1)) | (SPI_CTAR_PBR(0)) | (SPI_CTAR_BR(0)));
		}
		else if (kCTAR1 == aCfg)
		{
			// Fields reset
			SPI0->CTAR[1] &= 0x00000000;
			// Double Baud Rate --> The baud rate is computed normally with a 50/50 duty cycle
			// Frame Size --> 8 bits
			// Clock Polarity --> The inactive state value of SCK is high
			// Clock Phase --> Data is captured on the leading edge of SCK and changed on the following edge
			// LBS First --> Data is transferred MSB first
			// PCSSCK: PCS to SCK Delay Prescaler --> PCS to SCK Prescaler value is 1
			// PASC:After SCK Delay Prescaler --> Delay after Transfer Prescaler value is 1
			// PDT:Delay after Transfer Prescaler --> Delay after Transfer Prescaler value is 1
			// CSSCK:PCS to SCK Delay Scaler --> tCSC = (1/fSYS) x PCSSCK x CSSCK --> (1/50MHz)*1*2
			// ASC: After SCK Delay Scaler --> tASC = (1/fSYS) x PASC x ASC --> (1/50MHz)*1*1
			// DT:Delay After Transfer Scaler --> tDT = (1/fSYS) x PDT x DT --> (1/50MHz)*1*1
			// Baud Rate config
			// SCK baud rate = (fSYS/PBR) x [(1+DBR)/BR]
			// with fsys=fbus=50MHz
			// PBR = 1 div by 3,
			// DBR = 0 (50% duty cycle)
			// BR = 0 --> div by 2
			// SPI Clock = (50MHz/3) * (1/2)=8.3MHz
			SPI0->CTAR[1] |= ((SPI_CTAR_FMSZ(7)) | SPI_CTAR_CPOL_MASK | SPI_CTAR_CPHA_MASK | (SPI_CTAR_ASC(1)) | (SPI_CTAR_DT(1)) | (SPI_CTAR_PBR(1)) | (SPI_CTAR_BR(0)));
		}
		break;
	case kSpi1:
		if (kCTAR0 == aCfg)
		{
			// Fields reset
			SPI1->CTAR[0] &= 0x00000000;
			// Double Baud Rate --> The baud rate is computed normally with a 50/50 duty cycle
			// Frame Size --> 8 bits
			// Clock Polarity --> The inactive state value of SCK is low
			// Clock Phase --> Data is captured on the leading edge of SCK and changed on the following edge
			// LBS First --> Data is transferred LSB first
			// PCSSCK: PCS to SCK Delay Prescaler --> PCS to SCK Prescaler value is 1
			// PASC:After SCK Delay Prescaler --> Delay after Transfer Prescaler value is 1
			// PDT:Delay after Transfer Prescaler --> Delay after Transfer Prescaler value is 1
			// CSSCK:PCS to SCK Delay Scaler --> tCSC = (1/fSYS) x PCSSCK x CSSCK --> (1/50MHz)*1*2
			// ASC: After SCK Delay Scaler --> tASC = (1/fSYS) x PASC x ASC --> (1/50MHz)*1*1
			// DT:Delay After Transfer Scaler --> tDT = (1/fSYS) x PDT x DT --> (1/50MHz)*1*1
			// Baud Rate config
			// SCK baud rate = (fSYS/PBR) x [(1+DBR)/BR]
			// with fsys=fbus=50MHz
			// PBR = 0 div by 2
			// DBR = 0 (50% duty cycle)
			// BR = 0 --> div by 2
			// SPI Clock = (50MHz/2) * (1/2)=12.5MHz
			SPI1->CTAR[0] |= ((SPI_CTAR_FMSZ(7)) | SPI_CTAR_CPOL_MASK | SPI_CTAR_CPHA_MASK | SPI_CTAR_LSBFE_MASK | (SPI_CTAR_ASC(1)) | (SPI_CTAR_DT(1)) | (SPI_CTAR_PBR(0)) | (SPI_CTAR_BR(0)));
		}
		else if (kCTAR1 == aCfg)
		{ // this mode is needed for Pixy 2
			// for Melexis MLX75306 camera change SPI_CTAR_CPOL(0) to SPI_CTAR_CPOL_MASK

			// Fields reset
			SPI1->CTAR[1] &= 0x00000000;
			// Double Baud Rate --> The baud rate is computed normally with a 50/50 duty cycle
			// Frame Size --> 8 bits
			// Clock Polarity --> The inactive state value of SCK is low (!!!)
			// Clock Phase --> Data is captured on the leading edge of SCK and changed on the following edge
			// LBS First --> Data is transferred MSB first
			// PCSSCK: PCS to SCK Delay Prescaler --> PCS to SCK Prescaler value is 1
			// PASC:After SCK Delay Prescaler --> Delay after Transfer Prescaler value is 1
			// PDT:Delay after Transfer Prescaler --> Delay after Transfer Prescaler value is 1
			// CSSCK:PCS to SCK Delay Scaler --> tCSC = (1/fSYS) x PCSSCK x CSSCK --> (1/50MHz)*1*2
			// ASC: After SCK Delay Scaler --> tASC = (1/fSYS) x PASC x ASC --> (1/50MHz)*1*1
			// DT:Delay After Transfer Scaler --> tDT = (1/fSYS) x PDT x DT --> (1/50MHz)*1*1
			// Baud Rate config
			// SCK baud rate = (fSYS/PBR) x [(1+DBR)/BR]
			// with fsys=fbus=50MHz
			// PBR = 1 div by 3,
			// DBR = 0 (50% duty cycle)
			// BR = 0 --> div by 2
			// SPI Clock = (50MHz/3) * (1/2)=8.3MHz
			SPI1->CTAR[1] |= ((SPI_CTAR_FMSZ(7)) | SPI_CTAR_CPOL(0) | SPI_CTAR_CPHA_MASK | (SPI_CTAR_ASC(1)) | (SPI_CTAR_DT(1)) | (SPI_CTAR_PBR(1)) | (SPI_CTAR_BR(0)));
		}
		break;

	case kSpi2:
		if (kCTAR0 == aCfg)
		{
			// Fields reset
			SPI2->CTAR[0] &= 0x00000000;
			// Double Baud Rate --> The baud rate is computed normally with a 50/50 duty cycle
			// Frame Size --> 8 bits
			// Clock Polarity --> The inactive state value of SCK is low
			// Clock Phase --> Data is changed on the leading edge of SCK and captured on the following edge
			// LBS First --> Data is transferred MSB first
			// PCSSCK: PCS to SCK Delay Prescaler --> PCS to SCK Prescaler value is 1
			// PASC:After SCK Delay Prescaler --> Delay after Transfer Prescaler value is 1
			// PDT:Delay after Transfer Prescaler --> Delay after Transfer Prescaler value is 1
			// CSSCK:PCS to SCK Delay Scaler --> tCSC = (1/fSYS) x PCSSCK x CSSCK --> (1/50MHz)*1*2
			// ASC: After SCK Delay Scaler --> tASC = (1/fSYS) x PASC x ASC --> (1/50MHz)*1*1
			// DT:Delay After Transfer Scaler --> tDT = (1/fSYS) x PDT x DT --> (1/50MHz)*1*1
			// Baud Rate config
			// SCK baud rate = (fSYS/PBR) x [(1+DBR)/BR]
			// with fsys=fbus=50MHz
			// PBR = 0 div by 2
			// DBR = 0 (50% duty cycle)
			// BR = 0 --> div by 2
			// SPI Clock = (50MHz/2) * (1/2)=12.5MHz
			SPI2->CTAR[0] |= ((SPI_CTAR_FMSZ(7)) | SPI_CTAR_CPHA_MASK | (SPI_CTAR_ASC(0)) | (SPI_CTAR_DT(0)) | (SPI_CTAR_PBR(4)) | (SPI_CTAR_BR(4)));
		}
		else if (kCTAR1 == aCfg)
		{
			// Fields reset
			SPI2->CTAR[1] &= 0x00000000;
			// Double Baud Rate --> The baud rate is computed normally with a 50/50 duty cycle
			// Frame Size --> 8 bits
			// Clock Polarity --> The inactive state value of SCK is low
			// Clock Phase --> Data is captured on the leading edge of SCK and changed on the following edge
			// LBS First --> Data is transferred MSB first
			// PCSSCK: PCS to SCK Delay Prescaler --> PCS to SCK Prescaler value is 1
			// PASC:After SCK Delay Prescaler --> Delay after Transfer Prescaler value is 1
			// PDT:Delay after Transfer Prescaler --> Delay after Transfer Prescaler value is 1
			// CSSCK:PCS to SCK Delay Scaler --> tCSC = (1/fSYS) x PCSSCK x CSSCK --> (1/50MHz)*1*2
			// ASC: After SCK Delay Scaler --> tASC = (1/fSYS) x PASC x ASC --> (1/50MHz)*1*1
			// DT:Delay After Transfer Scaler --> tDT = (1/fSYS) x PDT x DT --> (1/50MHz)*1*1
			// Baud Rate config
			// SCK baud rate = (fSYS/PBR) x [(1+DBR)/BR]
			// with fsys=fbus=50MHz
			// PBR = 0 div by 2
			// DBR = 0 (50% duty cycle)
			// BR = 0 --> div by 2
			// SPI Clock = (50MHz/2) * (1/2)=12.5MHz
			SPI2->CTAR[1] |= ((SPI_CTAR_FMSZ(7)) | (SPI_CTAR_ASC(1)) | (SPI_CTAR_DT(1)) | (SPI_CTAR_PBR(2)) | (SPI_CTAR_BR(0)));
		}
		break;
	}
}

//-----------------------------------------------------------------------
// SPI config interrupt & DMA
// aSpi:which SPI (0,1 or 2)
//-----------------------------------------------------------------------
void iSpi_ConfigInterrupAndDMA(SPIEnum aSpi)
{
	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 1183
	// DSPI DMA/Interrupt Request Select and Enable Register(SPIx_RSER)
	switch (aSpi)
	{
	case kSpi0:
		// Transmission Complete Request Enable --> TCF interrupt requests are disabled
		// DSPI Finished Request Enable --> EOQF interrupt requests are disabled
		// Transmit FIFO Underflow Request Enable --> TFUF interrupt requests are disabled
		// Transmit FIFO Fill Request Enable --> TFFF interrupts or DMA requests are disabled
		// Transmit FIFO Fill DMA or Interrupt Request Select --> TFFF flag generates interrupt requests
		// Receive FIFO Overflow Request Enable --> RFOF interrupt requests are disabled
		// Receive FIFO Drain Request Enable --> RFDF interrupt or DMA requests are disabled
		// Receive FIFO Drain DMA or Interrupt Request Select --> Interrupt request
		// Field reset
		SPI0->RSER &= 0x00000000;
		//SPI0->RSER|=();
		break;

	case kSpi1:
		// Transmission Complete Request Enable --> TCF interrupt requests are disabled
		// DSPI Finished Request Enable --> EOQF interrupt requests are disabled
		// Transmit FIFO Underflow Request Enable --> TFUF interrupt requests are disabled
		// Transmit FIFO Fill Request Enable --> TFFF interrupts or DMA requests are disabled
		// Transmit FIFO Fill DMA or Interrupt Request Select --> TFFF flag generates interrupt requests
		// Receive FIFO Overflow Request Enable --> RFOF interrupt requests are disabled
		// Receive FIFO Drain Request Enable --> RFDF interrupt or DMA requests are disabled
		// Receive FIFO Drain DMA or Interrupt Request Select --> Interrupt request
		// Field reset
		SPI0->RSER &= 0x00000000;
		//SPI0->RSER|=();
		break;

	case kSpi2:
		// Transmission Complete Request Enable --> TCF interrupt requests are disabled
		// DSPI Finished Request Enable --> EOQF interrupt requests are disabled
		// Transmit FIFO Underflow Request Enable --> TFUF interrupt requests are disabled
		// Transmit FIFO Fill Request Enable --> TFFF interrupts or DMA requests are disabled
		// Transmit FIFO Fill DMA or Interrupt Request Select --> TFFF flag generates interrupt requests
		// Receive FIFO Overflow Request Enable --> RFOF interrupt requests are disabled
		// Receive FIFO Drain Request Enable --> RFDF interrupt or DMA requests are disabled
		// Receive FIFO Drain DMA or Interrupt Request Select --> Interrupt request
		// Field reset
		SPI0->RSER &= 0x00000000;
		//SPI0->RSER|=();
		break;
	}
}

//-----------------------------------------------------------------------
// SPI clock phase configuration
// aSpi:which SPI (0,1 or 2)
// aCfg:which config register (CTAR0 or CTAR1)
//-----------------------------------------------------------------------
void iSpi_ConfigClockPhase(SPIEnum aSpi, SpiCfgRegisterEnum aCfg, ClockPhaseEnum aPhase)
{
	// DSPI Clock and Transfer Attributes Register
	switch (aSpi)
	{
	case kSpi0:
		if (kCTAR0 == aCfg)
		{
			switch (aPhase)
			{
			case kCaptureLeadingEdge:
				SPI0->CTAR[0] &= (~SPI_CTAR_CPHA_MASK);
				break;
			case kDataChangeLeadingEdge:
				SPI0->CTAR[0] |= SPI_CTAR_CPHA_MASK;
				break;
			}
		}
		else if (kCTAR1 == aCfg)
		{
			switch (aPhase)
			{
			case kCaptureLeadingEdge:
				SPI0->CTAR[1] &= (~SPI_CTAR_CPHA_MASK);
				break;
			case kDataChangeLeadingEdge:
				SPI0->CTAR[1] |= SPI_CTAR_CPHA_MASK;
				break;
			}
		}
		break;
	case kSpi1:
		if (kCTAR0 == aCfg)
		{
			switch (aPhase)
			{
			case kCaptureLeadingEdge:
				SPI1->CTAR[0] &= (~SPI_CTAR_CPHA_MASK);
				break;
			case kDataChangeLeadingEdge:
				SPI1->CTAR[0] |= SPI_CTAR_CPHA_MASK;
				break;
			}
		}
		else if (kCTAR1 == aCfg)
		{
			switch (aPhase)
			{
			case kCaptureLeadingEdge:
				SPI1->CTAR[1] &= (~SPI_CTAR_CPHA_MASK);
				break;
			case kDataChangeLeadingEdge:
				SPI1->CTAR[1] |= SPI_CTAR_CPHA_MASK;
				break;
			}
		}
		break;

	case kSpi2:
		if (kCTAR0 == aCfg)
		{
			switch (aPhase)
			{
			case kCaptureLeadingEdge:
				SPI2->CTAR[0] &= (~SPI_CTAR_CPHA_MASK);
				break;
			case kDataChangeLeadingEdge:
				SPI2->CTAR[0] |= SPI_CTAR_CPHA_MASK;
				break;
			}
		}
		else if (kCTAR1 == aCfg)
		{
			switch (aPhase)
			{
			case kCaptureLeadingEdge:
				SPI2->CTAR[1] &= (~SPI_CTAR_CPHA_MASK);
				break;
			case kDataChangeLeadingEdge:
				SPI2->CTAR[1] |= SPI_CTAR_CPHA_MASK;
				break;
			}
		}
		break;
	}
}

//-----------------------------------------------------------------------
// Clear TX FIFO
// aSpi:		which SPI
//-----------------------------------------------------------------------
void iSpi_ClearTxFifo(SPIEnum aSpi)
{
	// DSPI Module Configuration Register (SPIx_MCR)
	switch (aSpi)
	{
	case kSpi0:
		SPI0->MCR |= SPI_MCR_CLR_TXF_MASK;
		break;
	case kSpi1:
		SPI1->MCR |= SPI_MCR_CLR_TXF_MASK;
		break;
	case kSpi2:
		SPI2->MCR |= SPI_MCR_CLR_TXF_MASK;
		break;
	}
}

//-----------------------------------------------------------------------
// Flush RX FIFO
// aSpi:		which SPI
//-----------------------------------------------------------------------
void iSpi_FlushRXFifo(SPIEnum aSpi)
{
	// DSPI Module Configuration Register (SPIx_MCR)
	switch (aSpi)
	{
	case kSpi0:
		SPI0->MCR |= SPI_MCR_CLR_RXF_MASK;
		break;
	case kSpi1:
		SPI1->MCR |= SPI_MCR_CLR_RXF_MASK;
		break;
	case kSpi2:
		SPI2->MCR |= SPI_MCR_CLR_RXF_MASK;
		break;
	}
}

//-----------------------------------------------------------------------
// Start TX
// aSpi:which SPI
//-----------------------------------------------------------------------
void iSpi_StartTX(SPIEnum aSpi)
{
	// DSPI Module Configuration Register (SPIx_MCR)
	switch (aSpi)
	{
	case kSpi0:
		SPI0->MCR &= (~SPI_MCR_HALT_MASK);
		break;
	case kSpi1:
		SPI1->MCR &= (~SPI_MCR_HALT_MASK);
		break;
	case kSpi2:
		SPI2->MCR &= (~SPI_MCR_HALT_MASK);
		break;
	}
}

//-----------------------------------------------------------------------
// Stop TX
// aSpi:which SPI
//-----------------------------------------------------------------------
void iSpi_StopTX(SPIEnum aSpi)
{
	// DSPI Module Configuration Register (SPIx_MCR)
	switch (aSpi)
	{
	case kSpi0:
		SPI0->MCR |= (SPI_MCR_HALT_MASK);
		break;
	case kSpi1:
		SPI1->MCR |= (SPI_MCR_HALT_MASK);
		break;
	case kSpi2:
		SPI2->MCR |= (SPI_MCR_HALT_MASK);
		break;
	}
}

//-----------------------------------------------------------------------------
// Get SPI status
// aSpi:which SPI
// aStatus: which status flag
// return	: flag state
//-----------------------------------------------------------------------------
bool iSpi_GetStatus(SPIEnum aSpi, SpiStatusEnum aStatus)
{
	bool aRet;

	// DSPI Status Register (SPIx_SR)
	switch (aSpi)
	{
	case kSpi0:
		aRet = ((SPI0->SR & aStatus) == aStatus);
		break;
	case kSpi1:
		aRet = ((SPI1->SR & aStatus) == aStatus);
		break;
	case kSpi2:
		aRet = ((SPI2->SR & aStatus) == aStatus);
		break;
	}

	return aRet;
}

//-----------------------------------------------------------------------------
// Get TX FIFO counter
// aSpi:which SPI
// return	: Tx fifo counter
//-----------------------------------------------------------------------------
UInt8 iSpi_GetTxFifoCounter(SPIEnum aSpi)
{
	UInt8 aRet;

	// DSPI Status Register (SPIx_SR)
	switch (aSpi)
	{
	case kSpi0:
		aRet = (UInt8)((SPI0->SR & SPI_SR_TXCTR_MASK) >> SPI_SR_TXCTR_SHIFT);
		break;
	case kSpi1:
		aRet = (UInt8)((SPI1->SR & SPI_SR_TXCTR_MASK) >> SPI_SR_TXCTR_SHIFT);
		break;
	case kSpi2:
		aRet = (UInt8)((SPI2->SR & SPI_SR_TXCTR_MASK) >> SPI_SR_TXCTR_SHIFT);
		break;
	}

	return aRet;
}

//-----------------------------------------------------------------------------
// Get TX FIFO next pointer
// aSpi:which SPI
// return	: Tx fifo next pointer
//-----------------------------------------------------------------------------
UInt8 iSpi_GetTxFifoNextPointer(SPIEnum aSpi)
{
	UInt8 aRet;

	// DSPI Status Register (SPIx_SR)
	switch (aSpi)
	{
	case kSpi0:
		aRet = (UInt8)((SPI0->SR & SPI_SR_TXNXTPTR_MASK) >> SPI_SR_TXNXTPTR_SHIFT);
		break;
	case kSpi1:
		aRet = (UInt8)((SPI1->SR & SPI_SR_TXNXTPTR_MASK) >> SPI_SR_TXNXTPTR_SHIFT);
		break;
	case kSpi2:
		aRet = (UInt8)((SPI2->SR & SPI_SR_TXNXTPTR_MASK) >> SPI_SR_TXNXTPTR_SHIFT);
		break;
	}

	return aRet;
}

//-----------------------------------------------------------------------------
// Get RX FIFO Counter
// aSpi:which SPI
// return	: Rx fifo counter
//-----------------------------------------------------------------------------
UInt8 iSpi_GetRxFifoCounter(SPIEnum aSpi)
{
	UInt8 aRet;

	// DSPI Status Register (SPIx_SR)
	switch (aSpi)
	{
	case kSpi0:
		aRet = (UInt8)((SPI0->SR & SPI_SR_RXCTR_MASK) >> SPI_SR_RXCTR_SHIFT);
		break;
	case kSpi1:
		aRet = (UInt8)((SPI1->SR & SPI_SR_RXCTR_MASK) >> SPI_SR_RXCTR_SHIFT);
		break;
	case kSpi2:
		aRet = (UInt8)((SPI2->SR & SPI_SR_RXCTR_MASK) >> SPI_SR_RXCTR_SHIFT);
		break;
	}

	return aRet;
}

//-----------------------------------------------------------------------------
// Get RX FIFO pop next pointer
// aSpi:which SPI
// return	: Rx fifo pop next pointer
//-----------------------------------------------------------------------------
UInt8 iSpi_GetRxFifoNextPointer(SPIEnum aSpi)
{
	UInt8 aRet;

	// DSPI Status Register (SPIx_SR)
	switch (aSpi)
	{
	case kSpi0:
		aRet = (UInt8)((SPI0->SR & SPI_SR_POPNXTPTR_MASK) >> SPI_SR_POPNXTPTR_SHIFT);
		break;
	case kSpi1:
		aRet = (UInt8)((SPI1->SR & SPI_SR_POPNXTPTR_MASK) >> SPI_SR_POPNXTPTR_SHIFT);
		break;
	case kSpi2:
		aRet = (UInt8)((SPI2->SR & SPI_SR_POPNXTPTR_MASK) >> SPI_SR_POPNXTPTR_SHIFT);
		break;
	}

	return aRet;
}

//-----------------------------------------------------------------------------
// Spi status flag reset
// aSpi:which SPI
// aStatus: which flag to reset
//-----------------------------------------------------------------------------
void iSpi_ResetStatus(SPIEnum aSpi, SpiStatusEnum aStatus)
{
	// DSPI Status Register (SPIx_SR)
	switch (aSpi)
	{
	case kSpi0:
		SPI0->SR |= aStatus;
		break;
	case kSpi1:
		SPI1->SR |= aStatus;
		break;
	case kSpi2:
		SPI2->SR |= aStatus;
		break;
	}
}

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
						   CsEnum aCs, UInt16 aData)
{
	UInt32 aTmp = 0;

	// DSPI PUSH TX FIFO Register In Master Mode (SPIx_PUSHR)
	aTmp = aCont | aCfg | aCont | (((UInt32)aLastQueueData) << 27) | aCs | ((UInt32)aData);

	switch (aSpi)
	{
	case kSpi0:
		SPI0->PUSHR = aTmp;
		break;
	case kSpi1:
		SPI1->PUSHR = aTmp;
		break;
	case kSpi2:
		while (iSpi_GetStatus(kSpi2, kTFFF) == false)
			;
		SPI2->PUSHR = aTmp;
		break;
	}
}

//-----------------------------------------------------------------------------
// Read a Data from the RX queue
// aSpi:which SPI
//-----------------------------------------------------------------------------
UInt32 iSpi_ReadData(SPIEnum aSpi)
{
	UInt32 aRet;

	// DSPI POP RX FIFO Register (SPIx_POPR)
	switch (aSpi)
	{
	case kSpi0:
		aRet = SPI0->POPR;
		break;
	case kSpi1:
		aRet = SPI1->POPR;
		break;
	case kSpi2:
		aRet = SPI2->POPR;
		break;
	}

	return aRet;
}
