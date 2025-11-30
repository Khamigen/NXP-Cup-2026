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
File name :				mGyro.c
Author and date :	Monnerat Serge 28.2.201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "Interfaces/iI2C.h"
#include "mGyro.h"
//#include "math.h"
#include "Interfaces/iPit.h"
#include "MK64F12.h"

//-----------------------------------------------------------------------------
//  Constantes
//-----------------------------------------------------------------------------
#define kGyroFact (62.5 / 1000.0) // sensibilit�
#define kGyroWithInterrupt 0	  // Avec/sans interrupt du FXAS21002

// Offset des 3 axes du gyro
static float sXoffset = 0;
static float sYoffset = 0;
static float sZoffset = 0;

// Etat n�cessaire � la calibration du gyroscope
typedef enum
{
	kGetGyroDelay,
	kWaitGyro
} sGyroCalibStateEnum;
static sGyroCalibStateEnum sGyroCalibState = kGetGyroDelay;

// Nu du Delay
static unsigned int sDelayGyro = 0;

//-----------------------------------------------------------------------------
//  Variables
//-----------------------------------------------------------------------------
static UInt8 sGyroDataReady = 0;
static UInt8 sAngVelocityData[6];

//-----------------------------------------------------------------------------
//  Setup FXAS21002C
//-----------------------------------------------------------------------------
void mGyro_Setup(void)
{
	// I2C config
	iI2C_Config();
}

//-----------------------------------------------------------------------------
// Open FXAS21002C
//-----------------------------------------------------------------------------
void mGyro_Open(void)
{
	// Enable I2C
	iI2C_Enable();
}

//-----------------------------------------------------------------------------
// Close FXAS21002C
//-----------------------------------------------------------------------------
void mGyro_Close(void)
{
}

//-----------------------------------------------------------------------------
// Reset FXAS21002C
//-----------------------------------------------------------------------------
void mGyro_Reset(void)
{
}

//-----------------------------------------------------------------------------
// Read a FXAS21002C register
//-----------------------------------------------------------------------------
bool mGyro_GetData(UInt8 aReg, UInt8 aData[], UInt8 aSize, UInt8 aAddr)
{
	bool aNoAck = FALSE;
	UInt8 i;

	if (aSize > 1)
	{
		// Disable auto ACK
		iI2C_SetAckMode(kAckAuto);
	}
	else
	{
		// Disable auto ACK
		iI2C_SetAckMode(kNoAck);
	}

	// Wait for the I2C bus ready
	while (TRUE == iI2C_ReadStatus(kBUSY))
		;

	// Set TX mode
	iI2C_TxRxSelect(kTxMode);
	// Set START
	iI2C_SetStartState();

	// Send TAS5731 TX address
	iI2C_SendData(aAddr);

	// Wait end of TX
	iI2C_WaitEndOfRxOrTx();

	// Check if ACK received
	aNoAck = iI2C_ReadStatus(RxAK);
	if (aNoAck == TRUE)
	{
		// Disable transmit ACK
		iI2C_SetAckMode(kNoAck);
		// Set TX mode
		iI2C_TxRxSelect(kTxMode);
		// Set STOP state
		iI2C_SetStopState();
		return FALSE;
	}

	// Send FXAS21002C register address
	iI2C_SendData(aReg);

	// Wait end of TX
	iI2C_WaitEndOfRxOrTx();

	// Check if ACK received
	aNoAck = iI2C_ReadStatus(RxAK);
	if (aNoAck == TRUE)
	{
		// Disable transmit ACK
		iI2C_SetAckMode(kNoAck);
		// Set TX mode
		iI2C_TxRxSelect(kTxMode);
		// Set STOP state
		iI2C_SetStopState();
		return FALSE;
	}

	//-----------------------------------------------------------------------------
	// RX Mode
	//-----------------------------------------------------------------------------
	// Set repeated START
	iI2C_SetRepeatedStartSate();
	// Send TAS5731 read address
	iI2C_SendData(aAddr + 1);

	// Wait end of TX
	iI2C_WaitEndOfRxOrTx();

	// Check if ACK received
	aNoAck = iI2C_ReadStatus(RxAK);
	if (aNoAck == TRUE)
	{
		// Disable transmit ACK
		iI2C_SetAckMode(kNoAck);
		// Set TX mode
		iI2C_TxRxSelect(kTxMode);
		// Set STOP state
		iI2C_SetStopState();
		return FALSE;
	}

	// Set RX mode
	iI2C_TxRxSelect(kRxMode);

	// Dummy read to send the first clock
	iI2C_ReadData();

	// Wait end of TX
	iI2C_WaitEndOfRxOrTx();

	for (i = 0; aSize > 1; aSize--, i++)
	{
		if (aSize == 2)
		{
			// Disable transmit ACK
			iI2C_SetAckMode(kNoAck);
		}

		// Get the TAS5731 register value
		aData[i] = iI2C_ReadData();
		// Wait end of TX
		iI2C_WaitEndOfRxOrTx();
	}

	// Disable transmit ACK
	iI2C_SetAckMode(kNoAck);
	// Set TX mode
	iI2C_TxRxSelect(kTxMode);
	// Set STOP state
	iI2C_SetStopState();

	// Get the TAS5731 register value
	aData[i] = iI2C_ReadData();

	return TRUE;
}

//-----------------------------------------------------------------------------
// Set a FXAS21002C register
//-----------------------------------------------------------------------------
bool mGyro_SetData(UInt8 aReg, UInt8 aValTab[], UInt8 aSize, UInt8 aAddr)
{
	bool aNoAck = FALSE;
	UInt8 i;

	// Disable auto ACK
	iI2C_SetAckMode(kNoAck);

	// Wait for the I2C bus ready
	while (TRUE == iI2C_ReadStatus(kBUSY))
		;

	// Set TX mode
	iI2C_TxRxSelect(kTxMode);
	// Set START
	iI2C_SetStartState();

	// Send TAS5731 address
	iI2C_SendData(aAddr);

	// Wait end of TX
	iI2C_WaitEndOfRxOrTx();

	// Check if ACK received
	aNoAck = iI2C_ReadStatus(RxAK);
	if (aNoAck == TRUE)
	{
		// Disable transmit ACK
		iI2C_SetAckMode(kNoAck);
		// Set TX mode
		iI2C_TxRxSelect(kTxMode);
		// Set STOP state
		iI2C_SetStopState();
		return FALSE;
	}

	// Send FXAS21002C register address
	iI2C_SendData(aReg);

	// Wait end of TX
	iI2C_WaitEndOfRxOrTx();

	// Check if ACK received
	aNoAck = iI2C_ReadStatus(RxAK);
	if (aNoAck == TRUE)
	{
		// Disable transmit ACK
		iI2C_SetAckMode(kNoAck);
		// Set TX mode
		iI2C_TxRxSelect(kTxMode);
		// Set STOP state
		iI2C_SetStopState();
		return FALSE;
	}

	for (i = 0; aSize > 0; aSize--, i++)
	{
		// Write register data
		iI2C_SendData(aValTab[i]);

		// Wait end of TX
		iI2C_WaitEndOfRxOrTx();

		// Check if ACK received
		aNoAck = iI2C_ReadStatus(RxAK);
		if (aNoAck == TRUE)
		{
			// Disable transmit ACK
			iI2C_SetAckMode(kNoAck);
			// Set TX mode
			iI2C_TxRxSelect(kTxMode);
			// Set STOP state
			iI2C_SetStopState();
			return FALSE;
		}
	}
	// Disable transmit ACK
	iI2C_SetAckMode(kNoAck);
	// Set TX mode
	iI2C_TxRxSelect(kTxMode);
	// Set STOP state
	iI2C_SetStopState();
	return TRUE;
}

//-----------------------------------------------------------------------------
// Init du FXAS21002C
//-----------------------------------------------------------------------------
bool mGyro_Init(void)
{
	UInt8 aDataAccel[20];
	bool aRet;
	Int16 aDelayNb;

	// Reset all registers to POR values
	aDataAccel[0] = 0x40;
	//aRet=mGyro_SetData(CTRL_REG2,aDataAccel,1,(kFXAS21002CAddr<<1));

	// Attente de 5ms
	aDelayNb = iPit_GetDelay(kPit1, 5 / kPit1Period);
	while (!iPit_IsDelayDone(kPit1, aDelayNb))
		;
	iPit_DelayRelease(kPit1, aDelayNb);

	// Lecture du registre WHo am I qui doit-�tre �gale � 0xD7
	aRet = mGyro_GetData(WHO_AM_I, aDataAccel, 1, (kFXAS21002CAddr << 1));

	// CTRL_REG1
	// [7]: ZR_cond=0
	// [6]: RST=0
	// [5]: ST=0 self test disabled
	// [4-2]: DR[2-0]=000 for 800Hz ODR
	// [1-0]: Active=0, Ready=0 for Standby mode
	aDataAccel[0] = 0x00;
	aRet = mGyro_SetData(CTRL_GREG1, aDataAccel, 1, (kFXAS21002CAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// CTRL_REG0
	// [7-6]: BW=10, low pass cutoff frequency = 16Hz � 200 d'output data rate ODR
	// [5]: SPIW=0 4 wire SPI (irrelevant)
	// [4-3]: SEL[1-0]=00 for 15Hz HPF at 800Hz ODR, high pass cutoff frequency
	// [2]: HPF_EN=0 disable HPF
	// [1-0]: FS[1-0]=00 for 2000dps
	aDataAccel[0] = 0x80;
	aRet = mGyro_SetData(CTRL_GREG0, aDataAccel, 1, (kFXAS21002CAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// CTRL_REG3
	// The auto-increment pointer rolls over to address 0x01 (X-axis MSB) in order to facilitate the faster
	// read out of the FIFO data in a single burst read operation (STATUS register only needs to be read once).
	aDataAccel[0] = 0x08;
	aRet = mGyro_SetData(CTRL_GREG3, aDataAccel, 1, (kFXAS21002CAddr << 1));
	if (aRet == false)
	{
		return false;
	}
	// Lecture du registre WHo am I qui doit-�tre �gale � 0xD7
	aRet = mGyro_GetData(CTRL_GREG3, aDataAccel, 1, (kFXAS21002CAddr << 1));

	// CTRL_REG2
#if (kGyroWithInterrupt)
	// Data ready interrupt enable, pushpull and active high
	aDataAccel[0] = 0x0e;
#else
	// Data ready interrupt disable, pushpull and active high
	aDataAccel[0] = 0x00;
#endif
	aRet = mGyro_SetData(CTRL_GREG2, aDataAccel, 1, (kFXAS21002CAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// CTRL_REG1
	// [7]: ZR_cond=0
	// [6]: RST=0
	// [5]: ST=0 self test disabled
	// [4-2]: DR[2-0]=010 for 200Hz ODR
	// [1-0]: Active=1, Ready=0 for Active mode
	aDataAccel[0] = 0x0A;
	aRet = mGyro_SetData(CTRL_GREG1, aDataAccel, 1, (kFXAS21002CAddr << 1));
	if (aRet == false)
	{
		return false;
	}

#if (kGyroWithInterrupt)
	// Gyro interrupt enable
	iI2C_GyroIntEnable();
#endif

	return true;
}

//-----------------------------------------------------------------------------
// Calibration de l'offset du gyroscope
//-----------------------------------------------------------------------------
bool mGyro_Calibration(void)
{
	UInt16 i = 500;
	float aX;
	float aY;
	float aZ;

	// Calcul des offsets
	do
	{
		switch (sGyroCalibState)
		{
		case kGetGyroDelay:
			// Prise d'un delay
			sDelayGyro = iPit_GetDelay(kPit1, 10 / kPit1Period);
			// Prochain �tat
			sGyroCalibState = kWaitGyro;
			break;

		case kWaitGyro:
			// Attente que le delay soit �chu --> temps de mesure
			if (iPit_IsDelayDone(kPit1, sDelayGyro))
			{
				iPit_DelayRelease(kPit1, sDelayGyro);
				// Lecture des mesures du capteur gyroscopique
				mGyro_ReadData(&aX, &aY, &aZ);
				// On accumule les mesures
				sXoffset += aX;
				sYoffset += aY;
				sZoffset += aZ;
				i--;
				// Prochain �tat
				sGyroCalibState = kGetGyroDelay;
			}

			break;
		}
	} while (i > 0);

	// Calcul des offset
	sXoffset /= 500;
	sYoffset /= 500;
	sZoffset /= 500;

	// Read data output registers 0x01-0x06 and 0x33 - 0x38
	mGyro_GetData(OUT_X_MSB, sAngVelocityData, 6, (kFXAS21002CAddr << 1));

	return true;
}

//-----------------------------------------------------------------------------
// Lecture du gyroscope
//-----------------------------------------------------------------------------
bool mGyro_ReadData(float *aX, float *aY, float *aZ)
{
	bool aRet;
	UInt8 aDataAccel[12];

#if (kGyroWithInterrupt == 0)
	// Lecture du registre de statut
	aRet = mGyro_GetData(DR_STATUS, aDataAccel, 1, (kFXAS21002CAddr << 1));
	sGyroDataReady = ((aDataAccel[0] & 0x08) == 0x08);
#endif

	if (sGyroDataReady) // Is a new set of data ready?
	{
		// Read data output registers 0x01-0x06 and 0x33 - 0x38
		aRet = mGyro_GetData(OUT_X_MSB, sAngVelocityData, 6, (kFXAS21002CAddr << 1));
		if (aRet == true)
		{
			// Concat�nation des deux bytes
			*aX = (float)((Int16)((sAngVelocityData[0] << 8) | sAngVelocityData[1]));
			*aY = (float)((Int16)((sAngVelocityData[2] << 8) | sAngVelocityData[3]));
			*aZ = (float)((Int16)((sAngVelocityData[4] << 8) | sAngVelocityData[5]));
		}
		// Reset du flag
		sGyroDataReady = 0;
	}

	return aRet;
}

//-----------------------------------------------------------------------------
// Lecture du gyroscope avec correction en degr�s/s
//-----------------------------------------------------------------------------
bool mGyro_ReadData_mDPS(float *aX, float *aY, float *aZ)
{
	bool aRet;
	UInt8 aDataAccel[12];

#if (kGyroWithInterrupt == 0)
	// Lecture du registre WHo am I qui doit-�tre �gale � 0xD7
	aRet = mGyro_GetData(DR_STATUS, aDataAccel, 1, (kFXAS21002CAddr << 1));
	sGyroDataReady = (aDataAccel[0] & 0x08) == 0x08;
#else
	// N�cessaire pour effacer le flag du FXAS21002...
	// Read data output registers 0x01-0x06 and 0x33 - 0x38
	aRet = mGyro_GetData(OUT_X_MSB, aDataAccel, 6, (kFXAS21002CAddr << 1));
#endif

	// Is a new set of data ready?
	if (sGyroDataReady)
	{
		// Read data output registers 0x01-0x06 and 0x33 - 0x38
		aRet = mGyro_GetData(OUT_X_MSB, sAngVelocityData, 6, (kFXAS21002CAddr << 1));
		if (aRet == true)
		{
			// Concat�nation des deux bytes
			*aX = (float)((Int16)((sAngVelocityData[0] << 8) | sAngVelocityData[1]));
			*aY = (float)((Int16)((sAngVelocityData[2] << 8) | sAngVelocityData[3]));
			*aZ = (float)((Int16)((sAngVelocityData[4] << 8) | sAngVelocityData[5]));
			// Application de l'offset
			*aX -= sXoffset;
			*aY -= sYoffset;
			*aZ -= sZoffset;
			// to mdps
			*aX *= kGyroFact;
			*aY *= kGyroFact;
			*aZ *= kGyroFact;
		}

		// Reset du flag
		sGyroDataReady = 0;
	}

	return aRet;
}

//-----------------------------------------------------------------------------
// PORT D Interrupt handler --> FXAS21002C datas ready --> pas utilis� --> flag
//-----------------------------------------------------------------------------
void PORTD_IRQHandler(void)
{

	PORTD->PCR[0] |= PORT_PCR_ISF_MASK; // Clear the interrupt flag
	PORTD->PCR[1] |= PORT_PCR_ISF_MASK; // Clear the interrupt flag

	// Set du flag utile � la fonction de lecture!
	sGyroDataReady = 1;
}
