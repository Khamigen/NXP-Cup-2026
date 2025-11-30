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
File name :	iCpu.c
Author and date :	Monnerat Serge 7 mars 201x

Description in the header file iCpu.h

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "iCpu.h"
#include "MK64F12.h"

// Peripherals clock init
static void iCpu_PeriphClkInit(void);

// Core and peripheral clock init
static void iCpu_PllInit(void);

// K10 pin functions configuration
static void iCpu_PinFctCfg(void);

// Internal voltage reference init (VREF0)
static void iCpu_VREFInit(void);

//------------------------------------------------------------
// Core and peripheral clock init
// External clock = 8MHz 	(20MHz)
// Core clock = 100MHz		(100MHz)
// Peripheral bus = 50MHz	(50MHz)
// Flexbus = 50MHz
// Flash = 25MHz
//------------------------------------------------------------
static void iCpu_PllInit(void)
{
	// If the internal load capacitors are being used, they should be selected
	// before enabling the oscillator. Application specific. 16pF and 8pF selected
	// in this example
	//OSC_CR = OSC_CR_SC16P_MASK | OSC_CR_SC8P_MASK;

	// Enable external oscillator, RANGE=2, HGO=1, EREFS=1, LP=0, IRCS=0
	MCG->C2 = MCG_C2_RANGE0(2) | MCG_C2_HGO0_MASK | MCG_C2_EREFS0_MASK;

	// after initialization of oscillator release latched state of oscillator and GPIO
	//SIM_SCGC4 |= SIM_SCGC4_LLWU_MASK;
	//LLWU_CS |= LLWU_CS_ACKISO_MASK;

	// Select external oscilator and Reference Divider and clear IREFS to start ext osc
	// CLKS=2, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
	MCG->C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);

	// Wait for oscillator to initialize
	while (!(MCG->S & MCG_S_OSCINIT0_MASK))
		;

	// Wait for Reference clock Status bit to clear
	while (MCG->S & MCG_S_IREFST_MASK)
		;

	// Wait for clock status bits to show clock source is ext ref clk
	while (((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2)
		;

	// Now in FBE

	// Configure PLL Ref Divider, PLLCLKEN=0, PLLSTEN=0, PRDIV=5
	// The crystal frequency is used to select the PRDIV value. Only even frequency crystals are supported
	// that will produce a 2MHz reference clock to the PLL.
	// 3 = divide factor -> 4 --> 8MHz/4 = 2MHz
	MCG->C5 = MCG_C5_PRDIV0(3);

	// Ensure MCG_C6 is at the reset default of 0. LOLIE disabled, PLL disabled, clk monitor disabled, PLL VCO divider is clear
	//MCG_C6 = 0x0;

	// Set the VCO divider and enable the PLL for 100MHz, LOLIE=0, PLLS=1, CME=0, VDIV=26 (2MHz * 50)
	MCG->C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV0(26);

	// Wait for PLL status bit to set
	while (!(MCG->S & MCG_S_PLLST_MASK))
		;

	// Wait for LOCK bit to set
	while (!(MCG->S & MCG_S_LOCK0_MASK))
		;

	// Set up the SIM clock dividers BEFORE switching to the PLL to ensure the
	// system clock speeds are in spec.
	// core = PLL (100MHz), bus = PLL/2 (50MHz), flexbus = PLL/2 (50MHz), flash = PLL/4 (25MHz)
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV3(1) | SIM_CLKDIV1_OUTDIV4(3);

	// Now running PBE Mode

	// Transition into PEE by setting CLKS to 0
	// CLKS=0, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
	MCG->C1 &= ~MCG_C1_CLKS_MASK;

	// Wait for clock status bits to update
	while (((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3)
		;

	// Now running PEE Mode

	// Set the trace clock to the core clock frequency
	SIM->SOPT2 |= SIM_SOPT2_TRACECLKSEL_MASK;

	// Enable the clock to the FlexBus module
	SIM->SCGC7 |= SIM_SCGC7_FLEXBUS_MASK;
}

//------------------------------------------------------------
// Pin functions configuration
//------------------------------------------------------------
static void iCpu_PinFctCfg(void)
{
	//enable Clocks to all ports
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

	// Setup Pins as GPIO pour le HBridge
	PORTA->PCR[15] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; // H-bridge left out enable
	PORTA->PCR[16] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; // H-bridge right out enable
	PORTA->PCR[17] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; // H-bridge left & right normal mode (no low power)
	PORTB->PCR[9] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;  // -> pas utilis�

	// Port pour les poussoirs
	PORTC->PCR[16] = PORT_PCR_MUX(1);
	PORTC->PCR[17] = PORT_PCR_MUX(1);

	// Ports pour les interrupteurs
	PORTC->PCR[9] = PORT_PCR_MUX(1);
	PORTC->PCR[10] = PORT_PCR_MUX(1);
	PORTC->PCR[11] = PORT_PCR_MUX(1);
	PORTC->PCR[12] = PORT_PCR_MUX(1);

	// Setup des pins pour l'UART 3
	PORTB->PCR[10] = 0 | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK | (PORT_PCR_MUX(0x3));
	PORTB->PCR[11] = 0 | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK | (PORT_PCR_MUX(0x3));

	// Port pour les poussoirs
	PORTC->PCR[14] = 0 | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK | (PORT_PCR_MUX(0x3));
	PORTC->PCR[15] = 0 | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_DSE_MASK | (PORT_PCR_MUX(0x3));

	// Fonction PWM du FTM2
	PORTB->PCR[18] = PORT_PCR_MUX(3);
	PORTB->PCR[19] = PORT_PCR_MUX(3);

	// Ports pour les LEDs
	PORTB->PCR[20] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	PORTB->PCR[21] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	PORTB->PCR[22] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	PORTB->PCR[23] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;

	// Entr�e analogique
	// Courant H Bridge left
	PORTB->PCR[0] = PORT_PCR_MUX(0);
	// Courant H Bridge right
	PORTB->PCR[1] = PORT_PCR_MUX(0);
	// UBatt
	PORTC->PCR[8] = PORT_PCR_MUX(0);

	// Erreur H Bridge gauche
	PORTB->PCR[16] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
	// Erreur H Bridge droit
	PORTB->PCR[17] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;

	// Config IO pour le reset du module wifi
	PORTC->PCR[13] = PORT_PCR_MUX(1);

	// Config des IO pour le sens de rotation
	PORTA->PCR[5] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;  // Sens moteur A --> gauche
	PORTA->PCR[14] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; // Sens moteur B --> droite

	// Setup des IO en sortie
	GPIOC->PDDR |= ((UInt32)(1 << 13));

	// Disable du pont en H
	GPIOA->PSOR |= ((UInt32)(1 << 17));

	// Reset du wifi OFF
	GPIOC->PSOR |= ((UInt32)(1 << 13));
}

//------------------------------------------------------------
// Peripherals clock init
//------------------------------------------------------------
void iCpu_PeriphClkInit(void)
{
	// System Clock Gating Control Register 1 (SIM_SCGC1)
	// Enable clock UART4 --> monitoring
	SIM->SCGC1 |= SIM_SCGC1_UART4_MASK;

	// System Clock Gating Control Register 2 (SIM_SCGC2)
	// Enable du Clock du DAC0 --> LEDs
	SIM->SCGC2 |= SIM_SCGC2_DAC0_MASK;

	// System Clock Gating Control Register 3 (SIM_SCGC3)
	// Enable du Clock du FTM2 --> Servo et ADC1
	SIM->SCGC3 |= (SIM_SCGC3_FTM2_MASK | SIM_SCGC3_ADC1_MASK);
	SIM->SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// System Clock Gating Control Register 4 (SIM_SCGC4)
	// UART1 3 clock enable --> wifi
	// I2C0 enable acc�l�rom�tre
	SIM->SCGC4 |= (SIM_SCGC4_I2C0_MASK | SIM_SCGC4_UART3_MASK);

	// System Clock Gating Control Register 6 (SIM_SCGC6)
	// Clock enable for ADC0, PIT, FTM0 et FTM1
	SIM->SCGC6 |= (SIM_SCGC6_DAC0_MASK | SIM_SCGC6_ADC0_MASK | SIM_SCGC6_PIT_MASK | SIM_SCGC6_SPI0_MASK | SIM_SCGC6_FTM0_MASK | SIM_SCGC6_FTM1_MASK);

	// System Clock Gating Control Register 7 (SIM_SCGC7)
	// Clock enable for MPU,DMA, FLEXBUS
	SIM->SCGC7 |= (SIM_SCGC7_DMA_MASK | SIM_SCGC7_MPU_MASK);
}

//------------------------------------------------------------
// Internal voltage reference init (VREF0)
//------------------------------------------------------------
static void iCpu_VREFInit(void)
{
	UInt8 *aVREFTRM = (UInt8 *)(0x40074000);

	// VREF Status and Control Register (VREF_SC)
	// Voltage Reference enabled,VREF bandgap and tight regulation buffer on
	// VREF_OUT available for internal and external use
	VREF->SC |= (VREF_SC_VREFEN_MASK | (VREF_SC_MODE_LV(2)));

	*aVREFTRM |= 0x3f;

	// Wait till the VREFSC is stable
	while (!(VREF->SC & VREF_SC_VREFST_MASK))
		;
}

//------------------------------------------------------------
// CPU configuration
//------------------------------------------------------------
void iCpu_SysInit(void)
{
	// Peripherals clock init
	iCpu_PeriphClkInit();

	// Disable MPU
	SYSMPU->CESR &= (~SYSMPU_CESR_VLD_MASK);

	// Core and peripheral clock init
	iCpu_PllInit();

	// K10 pin functions configuration
	iCpu_PinFctCfg();

	// Internal voltage reference init (VREF0)
	iCpu_VREFInit();
}
