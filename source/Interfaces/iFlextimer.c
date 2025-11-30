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
File name :	iFlextimer.c
Author and date :	Monnerat Serge 29 ao�t 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "iFlextimer.h"
#include "MK64F12.h"

//------------------------------------------------------------
// Constantes
//------------------------------------------------------------
// FTM0 pour la commande des PWM des moteurs DC
#define TFC_MOTOR_SWITCHING_FREQUENCY ((float)(20000.0))
#define FTM0_MOD_VALUE (int)((float)(PERIPHERAL_BUS_CLOCK) / TFC_MOTOR_SWITCHING_FREQUENCY)
#define FTM0_CLOCK PERIPHERAL_BUS_CLOCK
#define FTM0_CLK_PRESCALE 0
#define FTM0_OVERFLOW_FREQUENCY 5000

// FTM1 pour la mesure de vitesse
#define FTM1_CLK_PRESCALE 3 // Division par 8 du clock de 50MHz
#define FTM1_OVERFLOW_FREQUENCY (4)
#define kFreqTCNT ((float)6250000.0)

// Nombre de p�le des aimants
#define kNbPole 2

// FTM2 pour la commandes des PWM des servos
#define FTM2_CLOCK CORE_CLOCK										  // Le clock est le Cpu Clock
#define FTM2_CLK_PRESCALE 7											  // Prescaler -> 7 = divide by 128
#define FTM2_OVERFLOW_FREQUENCY 100.0								  // Fr�quence du PWM de 100Hz
#define SERVO_MIN_DUTY_CYCLE (float)(.0010 * FTM2_OVERFLOW_FREQUENCY) // Largeur de pulse min = 1ms
#define SERVO_MAX_DUTY_CYCLE (float)(.0020 * FTM2_OVERFLOW_FREQUENCY) // Largeur de pulse max = 2ms

//------------------------------------------------------------
// Globales
//------------------------------------------------------------
static UInt16 sOldTimerValueChL;
static float sDeltaChL = 0;
static UInt16 sOldTimerValueChR;
static float sDeltaChR = 0;

static float sSpeedLeft;
static float sSpeedRight;
static UInt32 sOverFlowCounterL;
static UInt32 sOverFlowCounterR;

//------------------------------------------------------------
// Flextimer  setup
// FTM0 --> moteur DC
// FTM1 --> mesure vitesse
// FTM2 --> servos
//------------------------------------------------------------
void iFlextimer_Config(void)
{
	// Enable the Clock to the FTM0 Module
	SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;

	// Enable the FTM functions on the the port
	PORTC->PCR[1] = PORT_PCR_MUX(4);
	PORTC->PCR[2] = PORT_PCR_MUX(4);
	PORTC->PCR[3] = PORT_PCR_MUX(4);
	PORTC->PCR[4] = PORT_PCR_MUX(4);

	// Features Mode Selection (FTMx_MODE)
	// Disable Write Protection - enables changes to QUADEN, DECAPEN, etc.
	FTM0->MODE |= FTM_MODE_WPDIS_MASK;
	//FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
	FTM0->MODE &= (~FTM_MODE_FTMEN_MASK);

	// Quadrature Decoder Control and Status (FTMx_QDCTRL)
	// Set Edge Aligned PWM
	FTM0->QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;

	// Counter (FTMx_CNT)
	// FTM Counter Value - reset counter to zero
	FTM0->CNT = 0x0; //FTM Counter Value - reset counter to zero

	// Modulo (FTMx_MOD)
	//Set the overflow rate
	FTM0->MOD = (FTM0_CLOCK / (1 << FTM0_CLK_PRESCALE)) / TFC_MOTOR_SWITCHING_FREQUENCY;

	// Counter Initial Value (FTMx_CNTIN)
	// Set the Counter Initial Value to 0
	FTM0->CNTIN = 0;

	// Channel (n) Status and Control (FTMx_CSC)
	// CHIE=0, Disable channel interrupts. Use software polling
	// MSB/MSA/ELSB/ELSA --> Edge-aligned PWM
	// DMA=0, Disable DMA transfers
	FTM0->CONTROLS[0].CnSC = 0;
	FTM0->CONTROLS[0].CnSC |= (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK);
	FTM0->CONTROLS[1].CnSC = 0;
	FTM0->CONTROLS[1].CnSC |= (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK);
	FTM0->CONTROLS[2].CnSC = 0;
	FTM0->CONTROLS[2].CnSC |= (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK);
	FTM0->CONTROLS[3].CnSC = 0;
	FTM0->CONTROLS[3].CnSC |= (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK);

	// Status and Control (FTMx_SC)
	// Edit registers when no clock is fed to timer so the MOD value, gets pushed in immediately
	FTM0->SC = 0;

	// Channel (n) Value (FTMx_CV)
	// FTMx_CnV contains the captured FTM counter value, this value determines the pulse width
	// Duty at 50%
	FTM0->CONTROLS[0].CnV = 0;
	FTM0->CONTROLS[1].CnV = 0;
	FTM0->CONTROLS[2].CnV = 0;
	FTM0->CONTROLS[3].CnV = 0;

	// Status and Control (FTMx_SC)
	// CPWMS=0,Center-aligned PWM Select -> FTM counter operates in up counting mode
	// CLKS=1, Clock Source Selection -> System clock -> 50Mhz
	// PS:3,Prescale Factor Selection -> Divide by 8
	FTM0->SC |= (FTM_SC_CLKS(1) | FTM_SC_PS(FTM0_CLK_PRESCALE));

	// FTM PWM Load (FTMx_PWMLOAD)
	FTM0->PWMLOAD |= FTM_PWMLOAD_LDOK_MASK;

	//------------------------------------------------------------
	// FTM1 --> Input Capture
	//------------------------------------------------------------
	// Input capture si:
	//� DECAPEN = 0
	//� COMBINE = 0
	//� CPWMS = 0
	//� MSnB:MSnA = 0:0, and
	//� ELSnB:ELSnA != 0:0
	// Vitesse max moteur 16'000 +- 3200 tr/mn --> ~ 20'000 tr/mn --> ~ 340 tr/s
	// Si 2 aimants --> fr�q. max = 340Hz * 4 = 1360 Hz
	// Si 4 aimants --> fr�q. max = 340Hz * 8 = 2720 Hz

	// Enable du clock
	SIM->SCGC6 |= SIM_SCGC6_FTM1_MASK;

	// Pin attribu�es au Flextimer
	PORTA->PCR[12] = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; // Moteur A vitesse A
	PORTA->PCR[13] = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK; // Moteur B vitesse A

	// Features Mode Selection (FTMx_MODE)
	// Disable Write Protection - enables changes to QUADEN, DECAPEN, etc.
	FTM1->MODE |= FTM_MODE_WPDIS_MASK;
	//FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
	FTM1->MODE &= (~FTM_MODE_FTMEN_MASK);

	// Counter (FTMx_CNT)
	// FTM Counter Value - reset counter to zero
	FTM1->CNT = 0x0; //FTM Counter Value - reset counter to zero

	// Modulo (FTMx_MOD)
	// Set the overflow rate
	FTM1->MOD = 65535;

	// Counter Initial Value (FTMx_CNTIN)
	//Set the Counter Initial Value to 0
	FTM1->CNTIN = 0;

	// Channel (n) Status and Control (FTMx_CSC)
	// CHIE=0, Disable channel interrupts. Use software polling
	// MSB/MSA/ELSB/ELSA --> Input Capture --> rising edge only
	// DMA=0, Disable DMA transfers
	FTM1->CONTROLS[0].CnSC = 0;
	FTM1->CONTROLS[0].CnSC |= (FTM_CnSC_ELSA_MASK | FTM_CnSC_CHIE_MASK);
	FTM1->CONTROLS[1].CnSC = 0;
	FTM1->CONTROLS[1].CnSC |= (FTM_CnSC_ELSA_MASK | FTM_CnSC_CHIE_MASK);

	// Status and Control (FTMx_SC)
	// CLKS=1, Clock Source Selection -> System clock -> 50Mhz
	// PS:3,Prescale Factor Selection -> Divide by 8 --> fr�quence de comptage de 50MHz/8
	// --> overflow � 50MHz/8/65535 => Fr�q min de 95Hz avec 2 aimants 1425 tr/mn min
	// Interruption overflow enable
	FTM1->SC = 0 | FTM_SC_TOIE_MASK | (FTM_SC_CLKS(1) | FTM_SC_PS(FTM1_CLK_PRESCALE));

	// IRQ Config
	// IRQ Config
	// Interrupt Clear-pending Registers
	// Clear prending interrupt
	NVIC_ClearPendingIRQ(FTM1_IRQn);
	// Interrupt Set-enable Registers
	// Interrupt enable
	NVIC_EnableIRQ(FTM1_IRQn);
	// Interrupt Priority Registers
	// Set interrupt priority
	NVIC_SetPriority(FTM1_IRQn, kFTM1_IntPriority);

	//------------------------------------------------------------
	// FTM2 --> PWM servo
	//------------------------------------------------------------
	// Features Mode Selection (FTMx_MODE)
	// Disable Write Protection - enables changes to QUADEN, DECAPEN, etc.
	FTM2->MODE |= FTM_MODE_WPDIS_MASK;
	//FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
	FTM2->MODE &= (~FTM_MODE_FTMEN_MASK);

	// Quadrature Decoder Control and Status (FTMx_QDCTRL)
	// Set Edge Aligned PWM
	FTM2->QDCTRL &= ~FTM_QDCTRL_QUADEN_MASK;

	// Counter (FTMx_CNT)
	// FTM Counter Value - reset counter to zero
	FTM2->CNT = 0x0; //FTM Counter Value - reset counter to zero

	// Modulo (FTMx_MOD)
	// Set the overflow rate
	// 50Mhz/128/100Hz=3906
	FTM2->MOD = (PERIPHERAL_BUS_CLOCK / (1 << FTM2_CLK_PRESCALE)) / FTM2_OVERFLOW_FREQUENCY;

	// Counter Initial Value (FTMx_CNTIN)
	//Set the Counter Initial Value to 0
	FTM2->CNTIN = 0;

	// Channel (n) Status and Control (FTMx_CSC)
	// CHIE=0, Disable channel interrupts. Use software polling
	// MSB/MSA/ELSB/ELSA --> Edge-aligned PWM
	// DMA=0, Disable DMA transfers
	FTM2->CONTROLS[0].CnSC = 0;
	FTM2->CONTROLS[0].CnSC |= (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK);
	FTM2->CONTROLS[1].CnSC = 0;
	FTM2->CONTROLS[1].CnSC |= (FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK);

	// Status and Control (FTMx_SC)
	// Edit registers when no clock is fed to timer so the MOD value, gets pushed in immediately
	FTM2->SC = 0;

	// Channel (n) Value (FTMx_CV)
	// FTMx_CnV contains the captured FTM counter value, this value determines the pulse width
	// Duty at 50%
	FTM2->CONTROLS[0].CnV = FTM2->MOD / 10;
	FTM2->CONTROLS[1].CnV = FTM2->MOD / 10;

	// Status and Control (FTMx_SC)
	// CPWMS=0,Center-aligned PWM Select -> FTM counter operates in up counting mode
	// CLKS=1, Clock Source Selection -> System clock -> 50Mhz
	// PS:7,Prescale Factor Selection -> Divide by 128
	FTM2->SC |= (FTM_SC_CLKS(1) | FTM_SC_PS(FTM2_CLK_PRESCALE));

	// FTM PWM Load (FTMx_PWMLOAD)
	// K10 Sub-Family Reference Manual, Rev. 6, Nov 2011 page 933
	FTM2->PWMLOAD |= FTM_PWMLOAD_LDOK_MASK;
}

//-----------------------------------------------------------------------------
// Lecture de la vitesse de rotation des moteurs
// Moteur A = moteur gauche --> valeur n�gative = en arri�re, valeur pos=en avant
// Moteur B = moteur droite
//-----------------------------------------------------------------------------
void iFlextimer_GetSpeed(float *aSpeedMotLeft, float *aSpeedMotRight)
{
	*aSpeedMotLeft = sSpeedLeft;
	*aSpeedMotRight = sSpeedRight;
}

//------------------------------------------------------------
// Interrupt routine FTM1
//------------------------------------------------------------
void FTM1_IRQHandler(void)
{
	UInt32 aVal;

	PIT->CHANNEL[3].LDVAL = 2000;

	// Test si le compteur � fait un overflow
	if ((FTM1->SC & FTM_SC_TOF_MASK) == FTM_SC_TOF_MASK)
	{
		// Reset du flag
		FTM1->SC &= (~FTM_SC_TOF_MASK);

		// Incr�mentation des compteurs pour la calcul
		// de la vitesse
		sOverFlowCounterL++;
		sOverFlowCounterR++;
	}

	// Calcul de la vitesse du moteur gauche --> teste si flanc sur input capture
	if ((FTM1->CONTROLS[0].CnSC & FTM_CnSC_CHF_MASK) == FTM_CnSC_CHF_MASK)
	{
		// Reset du flag
		FTM1->CONTROLS[0].CnSC &= (~FTM_CnSC_CHF_MASK);

		// Lecture du registre INPUT CAPTURE/OUTPUT COMPARE REGISTER 1
		aVal = FTM1->CONTROLS[0].CnV;

		if (sOverFlowCounterL > 0)
		{
			sDeltaChL = (sOverFlowCounterL * 65535.0) - sOldTimerValueChL + aVal;
		}
		else
		{
			sDeltaChL = aVal - sOldTimerValueChL;
		}

		// Calcul de la vitesse en tr/mn
		if (sDeltaChL > 0)
		{
			sSpeedLeft = ((kFreqTCNT / sDeltaChL) / (kNbPole * 2)) * 60;
		}
		else
		{
			sSpeedLeft = 0;
		}

		// Test l'entr�e B (PortA pin5) afin de d�terminer le sens
		if ((GPIOA->PDIR & 0x20) == 0x0)
		{
			sSpeedLeft = sSpeedLeft * (-1);
		}

		// On sauve l'ancienne valeur du compteur
		sOldTimerValueChL = aVal;

		// Reset du compteur d'overflow
		sOverFlowCounterL = 0;
	}

	// Calcul de la vitesse du moteur droit --> teste si flanc sur input capture
	if ((FTM1->CONTROLS[1].CnSC & FTM_CnSC_CHF_MASK) == FTM_CnSC_CHF_MASK)
	{
		// Reset du flag
		FTM1->CONTROLS[1].CnSC &= (~FTM_CnSC_CHF_MASK);

		// Lecture du registre INPUT CAPTURE/OUTPUT COMPARE REGISTER 1
		aVal = FTM1->CONTROLS[1].CnV;

		if (sOverFlowCounterR > 0)
		{
			sDeltaChR = (sOverFlowCounterR * 65535.0) - sOldTimerValueChR + aVal;
		}
		else
		{
			sDeltaChR = aVal - sOldTimerValueChR;
		}

		// Calcul de la vitesse en tr/mn
		if (sDeltaChR > 0)
		{
			sSpeedRight = ((kFreqTCNT / sDeltaChR) / (kNbPole * 2)) * 60;
		}
		else
		{
			sSpeedRight = 0;
		}

		// Test l'entr�e B (PortA pin14) afin de d�terminer le sens
		// A v�rifier, pas encore test�!
		if ((GPIOA->PDIR & 0x4000) == 00)
		{
			sSpeedRight = sSpeedRight * (-1);
		}

		// On sauve l'ancienne valeur du compteur
		sOldTimerValueChR = aVal;

		// Reset du compteur d'overflow
		sOverFlowCounterR = 0;
	}
}

//---------------------------------------------------------------------------
// Modifie la position du servo
// Position entre -1.0 to 1.0.
//---------------------------------------------------------------------------
// --> Utilisez les constantes SERVO_X_MIN_DUTY_CYCLE and SERVO_MAX_DUTY_CYCLE  pour calibrer les extr�mes
void iFlextimer_SetServoDuty(UInt8 aServoNumber, float aPosition)
{
	float aDutyCycle;

	// Saturation
	if (aPosition > 1)
	{
		aPosition = 1;
	}
	else if (aPosition < -1)
	{
		aPosition = -1;
	}

	// Calcul du duty
	aDutyCycle = (((aPosition + 1.0) / 2) * (SERVO_MAX_DUTY_CYCLE - SERVO_MIN_DUTY_CYCLE)) + SERVO_MIN_DUTY_CYCLE;

	switch (aServoNumber)
	{
	default:
	case 0:
		FTM2->CONTROLS[0].CnV = FTM2->MOD * aDutyCycle;
		break;

	case 1:
		FTM2->CONTROLS[1].CnV = FTM2->MOD * aDutyCycle;
		break;
	}
}

//---------------------------------------------------------------------------
// Modifie le duty du PWM
// Moteur A = moteur gauche
// Moteur B = moteur droite
//---------------------------------------------------------------------------
void iFlextimer_SetMotorDuty(float aMotorLeft, float aMotorRight)
{
	// Saturation du duty
	if (aMotorLeft > 1.0)
	{
		aMotorLeft = 1.0;
	}
	else if (aMotorLeft < -1.0)
	{
		aMotorLeft = -1.0;
	}

	if (aMotorRight > 1.0)
	{
		aMotorRight = 1.0;
	}
	else if (aMotorRight < -1.0)
	{
		aMotorRight = -1.0;
	}

	// Sens de rotation
	if (aMotorLeft > 0)
	{
		// Config du duty --> moteur A
		FTM0->CONTROLS[0].CnV = (uint16_t)((float)FTM0->MOD * aMotorLeft); // IN1
		FTM0->CONTROLS[3].CnV = 0;										   // IN2
	}
	else
	{
		aMotorLeft = aMotorLeft * (-1);
		// Config du duty --> moteur A
		FTM0->CONTROLS[3].CnV = (uint16_t)((float)FTM0->MOD * aMotorLeft); // IN1
		FTM0->CONTROLS[0].CnV = 0;										   // IN2
	}

	if (aMotorRight > 0)
	{
		FTM0->CONTROLS[2].CnV = (uint16_t)((float)FTM0->MOD * aMotorRight); // IN2
		FTM0->CONTROLS[1].CnV = 0;											// IN1
	}
	else
	{
		aMotorRight = aMotorRight * (-1);
		FTM0->CONTROLS[1].CnV = (uint16_t)((float)FTM0->MOD * aMotorRight); // IN1
		FTM0->CONTROLS[2].CnV = 0;											// IN2
	}
}
