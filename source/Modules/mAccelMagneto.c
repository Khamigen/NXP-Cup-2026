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
File name :				mAccelMagneto.c
Author and date :	Monnerat Serge 28.2.201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "Interfaces/iI2C.h" // KM
#include "mAccelMagneto.h"
#include "math.h"
#include "Interfaces/iPit.h" //KM
#include "MK64F12.h"

//-----------------------------------------------------------------------------
//  Constantes
//-----------------------------------------------------------------------------
#define PI 3.14159

//-----------------------------------------------------------------------------
//  Variables
//-----------------------------------------------------------------------------
static UInt8 sDataReady = 0;
static UInt8 AccelMagData[12];
static Int16 Xout_Accel_14_bit;
static Int16 Yout_Accel_14_bit;
static Int16 Zout_Accel_14_bit;
static float Xout_g;
static float Yout_g;
static float Zout_g;
static Int16 Xout_Mag_16_bit;
static Int16 Yout_Mag_16_bit;
static Int16 Zout_Mag_16_bit;
static float Xout_uT;
static float Yout_uT;
static float Zout_uT;
static float Heading;

//-----------------------------------------------------------------------------
//  Setup FXOS8700CQ
//-----------------------------------------------------------------------------
void mAccelMagneto_Setup(void)
{
	// I2C config
	iI2C_Config();
}

//-----------------------------------------------------------------------------
// Open FXOS8700CQ
//-----------------------------------------------------------------------------
void mAccelMagneto_Open(void)
{
	// Enable I2C
	iI2C_Enable();
}

//-----------------------------------------------------------------------------
// Close FXOS8700CQ
//-----------------------------------------------------------------------------
void mAccelMagneto_Close(void)
{
}

//-----------------------------------------------------------------------------
// Reset FXOS8700CQ
//-----------------------------------------------------------------------------
void mAccelMagneto_Reset(void)
{
}

//-----------------------------------------------------------------------------
// Read a FXOS8700CQ register
//-----------------------------------------------------------------------------
bool mAccelMagneto_GetData(UInt8 aReg, UInt8 aData[], UInt8 aSize, UInt8 aAddr)
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

	// Send FXOS8700CQ register address
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
// Set a FXOS8700CQ register
//-----------------------------------------------------------------------------
bool mAccelMagneto_SetData(UInt8 aReg, UInt8 aValTab[], UInt8 aSize, UInt8 aAddr)
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

	// Send FXOS8700CQ register address
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
// Init du FXOS8700CQ
//-----------------------------------------------------------------------------
bool mAccelMagneto_Init(void)
{
	UInt8 aDataAccel[20];
	bool aRet;
	Int16 aDelayNb;

	// Reset all registers to POR values
	aDataAccel[0] = 0x40;
	//aRet=mAccelMagneto_SetData(CTRL_REG2,aDataAccel,1,(kFXOS8700CQAddr<<1));

	// Attente de 5ms
	aDelayNb = iPit_GetDelay(kPit1, 5 / kPit1Period);
	while (!iPit_IsDelayDone(kPit1, aDelayNb))
		;
	iPit_DelayRelease(kPit1, aDelayNb);

	// ----
	aDataAccel[0] = 0x00;
	aRet = mAccelMagneto_SetData(CTRL_REG1, aDataAccel, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// +/-2g range with 0.244mg/LSB
	aDataAccel[0] = 0x00;
	aRet = mAccelMagneto_SetData(XYZ_DATA_CFG_REG, aDataAccel, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// Hybrid mode (accelerometer + magnetometer), max OSR
	aDataAccel[0] = 0x1F;
	aRet = mAccelMagneto_SetData(M_CTRL_REG1, aDataAccel, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// M_OUT_X_MSB register 0x33 follows the OUT_Z_LSB register 0x06 (used for burst read)
	aDataAccel[0] = 0x20;
	aRet = mAccelMagneto_SetData(M_CTRL_REG2, aDataAccel, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// High Resolution mode
	aDataAccel[0] = 0x02;
	aRet = mAccelMagneto_SetData(CTRL_REG2, aDataAccel, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// Push-pull, active low interrupt
	aDataAccel[0] = 0x00;
	aRet = mAccelMagneto_SetData(CTRL_REG3, aDataAccel, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// Enable DRDY interrupt
	aDataAccel[0] = 0x01;
	aRet = mAccelMagneto_SetData(CTRL_REG4, aDataAccel, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// DRDY interrupt routed to INT1 - PTD4
	aDataAccel[0] = 0x01;
	aRet = mAccelMagneto_SetData(CTRL_REG5, aDataAccel, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// ODR = 3.125Hz, Reduced noise, Active mode
	aDataAccel[0] = 0x0d; //0x35;
	aRet = mAccelMagneto_SetData(CTRL_REG1, aDataAccel, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	return true;
}

//-----------------------------------------------------------------------------
// Calibration de l'offset de l'acc�l�rom�tre
//-----------------------------------------------------------------------------
bool mAccelMagneto_Accel_Calibration(void)
{
	UInt8 X_Accel_offset, Y_Accel_offset, Z_Accel_offset;
	bool aRet;
	UInt8 aData[4];

	//	aRet=mAccelMagneto_GetData(STATUS_REG,aData,1,(kFXOS8700CQAddr<<1));
	//	aRet=mAccelMagneto_GetData(STATUS_REG,aData,1,(kFXOS8700CQAddr<<1));

	//	sDataReady = 0;
	//	while (!sDataReady);		// Is a first set of data ready?
	//	sDataReady = 0;

	do
	{
		aRet = mAccelMagneto_GetData(STATUS_REG, aData, 1, (kFXOS8700CQAddr << 1));
	} while ((aData[0] & 0x08) != 0x08);

	// Standby mode
	AccelMagData[0] = 0x00;
	aRet = mAccelMagneto_SetData(CTRL_REG1, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// Read data output registers 0x01-0x06
	aRet = mAccelMagneto_GetData(OUT_X_MSB_REG, AccelMagData, 6, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	Xout_Accel_14_bit = ((short)(AccelMagData[0] << 8 | AccelMagData[1])) >> 2; // Compute 14-bit X-axis acceleration output value
	Yout_Accel_14_bit = ((short)(AccelMagData[2] << 8 | AccelMagData[3])) >> 2; // Compute 14-bit Y-axis acceleration output value
	Zout_Accel_14_bit = ((short)(AccelMagData[4] << 8 | AccelMagData[5])) >> 2; // Compute 14-bit Z-axis acceleration output value

	X_Accel_offset = Xout_Accel_14_bit / 8 * (-1);					  // Compute X-axis offset correction value
	Y_Accel_offset = Yout_Accel_14_bit / 8 * (-1);					  // Compute Y-axis offset correction value
	Z_Accel_offset = (Zout_Accel_14_bit - SENSITIVITY_2G) / 8 * (-1); // Compute Z-axis offset correction value

	// X accel offset
	AccelMagData[0] = X_Accel_offset;
	aRet = mAccelMagneto_SetData(OFF_X_REG, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// Y accel offset
	AccelMagData[0] = Y_Accel_offset;
	aRet = mAccelMagneto_SetData(OFF_Y_REG, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// Z accel offset
	AccelMagData[0] = Z_Accel_offset;
	aRet = mAccelMagneto_SetData(OFF_Z_REG, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// Active mode again
	AccelMagData[0] = 0x35;
	aRet = mAccelMagneto_SetData(CTRL_REG1, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	return true;
}

//-----------------------------------------------------------------------------
// Calibration de l'offset du magn�tom�tre
//-----------------------------------------------------------------------------
bool mAccelMagneto_Mag_Calibration(void)
{
	short Xout_Mag_16_bit_avg, Yout_Mag_16_bit_avg, Zout_Mag_16_bit_avg;
	short Xout_Mag_16_bit_max, Yout_Mag_16_bit_max, Zout_Mag_16_bit_max;
	short Xout_Mag_16_bit_min, Yout_Mag_16_bit_min, Zout_Mag_16_bit_min;
	int i = 0;
	bool aRet;
	UInt8 aData[4];

	//sDataReady = 0;

	while (i < 150) // This will take ~30s (94 samples * 1/3.125)
	{
		aRet = mAccelMagneto_GetData(STATUS_REG, aData, 1, (kFXOS8700CQAddr << 1));
		if ((aData[0] & 0x8) == 0x8) // Is a new set of data ready?
		{
			sDataReady = 0;

			// Read data output registers 0x33 - 0x38
			aRet = mAccelMagneto_GetData(MOUT_X_MSB_REG, AccelMagData, 6, (kFXOS8700CQAddr << 1));
			if (aRet == false)
			{
				return false;
			}

			Xout_Mag_16_bit = (short)(AccelMagData[0] << 8 | AccelMagData[1]); // Compute 16-bit X-axis magnetic output value
			Yout_Mag_16_bit = (short)(AccelMagData[2] << 8 | AccelMagData[3]); // Compute 16-bit Y-axis magnetic output value
			Zout_Mag_16_bit = (short)(AccelMagData[4] << 8 | AccelMagData[5]); // Compute 16-bit Z-axis magnetic output value

			if (i == 0)
			{
				Xout_Mag_16_bit_max = Xout_Mag_16_bit;
				Xout_Mag_16_bit_min = Xout_Mag_16_bit;

				Yout_Mag_16_bit_max = Yout_Mag_16_bit;
				Yout_Mag_16_bit_min = Yout_Mag_16_bit;

				Zout_Mag_16_bit_max = Zout_Mag_16_bit;
				Zout_Mag_16_bit_min = Zout_Mag_16_bit;
			}

			// Check to see if current sample is the maximum or minimum X-axis value
			if (Xout_Mag_16_bit > Xout_Mag_16_bit_max)
			{
				Xout_Mag_16_bit_max = Xout_Mag_16_bit;
			}
			if (Xout_Mag_16_bit < Xout_Mag_16_bit_min)
			{
				Xout_Mag_16_bit_min = Xout_Mag_16_bit;
			}

			// Check to see if current sample is the maximum or minimum Y-axis value
			if (Yout_Mag_16_bit > Yout_Mag_16_bit_max)
			{
				Yout_Mag_16_bit_max = Yout_Mag_16_bit;
			}
			if (Yout_Mag_16_bit < Yout_Mag_16_bit_min)
			{
				Yout_Mag_16_bit_min = Yout_Mag_16_bit;
			}

			// Check to see if current sample is the maximum or minimum Z-axis value
			if (Zout_Mag_16_bit > Zout_Mag_16_bit_max)
			{
				Zout_Mag_16_bit_max = Zout_Mag_16_bit;
			}
			if (Zout_Mag_16_bit < Zout_Mag_16_bit_min)
			{
				Zout_Mag_16_bit_min = Zout_Mag_16_bit;
			}

			i++;
		}
	}

	Xout_Mag_16_bit_avg = (Xout_Mag_16_bit_max + Xout_Mag_16_bit_min) / 2; // X-axis hard-iron offset
	Yout_Mag_16_bit_avg = (Yout_Mag_16_bit_max + Yout_Mag_16_bit_min) / 2; // Y-axis hard-iron offset
	Zout_Mag_16_bit_avg = (Zout_Mag_16_bit_max + Zout_Mag_16_bit_min) / 2; // Z-axis hard-iron offset

	// Left-shift by one as magnetometer offset registers are 15-bit only, left justified
	Xout_Mag_16_bit_avg <<= 1;
	Yout_Mag_16_bit_avg <<= 1;
	Zout_Mag_16_bit_avg <<= 1;

	// Standby mode to allow writing to the offset registers
	AccelMagData[0] = 0x00;
	aRet = mAccelMagneto_SetData(CTRL_REG1, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// Setup des offsets
	AccelMagData[0] = (char)(Xout_Mag_16_bit_avg & 0xFF);
	aRet = mAccelMagneto_SetData(MOFF_X_LSB_REG, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	AccelMagData[0] = (char)((Xout_Mag_16_bit_avg >> 8) & 0xFF);
	aRet = mAccelMagneto_SetData(MOFF_X_MSB_REG, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	AccelMagData[0] = (char)(Yout_Mag_16_bit_avg & 0xFF);
	aRet = mAccelMagneto_SetData(MOFF_Y_LSB_REG, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	AccelMagData[0] = (char)((Yout_Mag_16_bit_avg >> 8) & 0xFF);
	aRet = mAccelMagneto_SetData(MOFF_Y_MSB_REG, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	AccelMagData[0] = (char)(Zout_Mag_16_bit_avg & 0xFF);
	aRet = mAccelMagneto_SetData(MOFF_Z_LSB_REG, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	AccelMagData[0] = (char)((Zout_Mag_16_bit_avg >> 8) & 0xFF);
	aRet = mAccelMagneto_SetData(MOFF_Z_MSB_REG, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	// Active mode again
	AccelMagData[0] = 0x35;
	aRet = mAccelMagneto_SetData(CTRL_REG1, AccelMagData, 1, (kFXOS8700CQAddr << 1));
	if (aRet == false)
	{
		return false;
	}

	return true;
}

//-----------------------------------------------------------------------------
// Lecture de l'acc�l�rom�tre et du magn�tom�tre
//-----------------------------------------------------------------------------
bool mAccelMagneto_ReadData(SRAWDATAEnum *aAccel, SRAWDATAEnum *aMagneto, float *aYaw, float *aRoll, float *aPitch)
{
	bool aRet;
	UInt8 aData[4];

	aRet = mAccelMagneto_GetData(STATUS_REG, aData, 1, (kFXOS8700CQAddr << 1));
	if ((aData[0] & 0x8) == 0x8) // Is a new set of data ready?
	{
		// Read data output registers 0x01-0x06 and 0x33 - 0x38
		aRet = mAccelMagneto_GetData(OUT_X_MSB_REG, AccelMagData, 12, (kFXOS8700CQAddr << 1));
		if (aRet == true)
		{

			// 14-bit accelerometer data
			Xout_Accel_14_bit = ((short)(AccelMagData[0] << 8 | AccelMagData[1])) >> 2; // Compute 14-bit X-axis acceleration output value
			Yout_Accel_14_bit = ((short)(AccelMagData[2] << 8 | AccelMagData[3])) >> 2; // Compute 14-bit Y-axis acceleration output value
			Zout_Accel_14_bit = ((short)(AccelMagData[4] << 8 | AccelMagData[5])) >> 2; // Compute 14-bit Z-axis acceleration output value

			// Accelerometer data converted to g's
			Xout_g = ((float)Xout_Accel_14_bit) / SENSITIVITY_2G; // Compute X-axis output value in g's
			Yout_g = ((float)Yout_Accel_14_bit) / SENSITIVITY_2G; // Compute Y-axis output value in g's
			Zout_g = ((float)Zout_Accel_14_bit) / SENSITIVITY_2G; // Compute Z-axis output value in g's

			// 16-bit magnetometer data
			Xout_Mag_16_bit = (short)(AccelMagData[6] << 8 | AccelMagData[7]);   // Compute 16-bit X-axis magnetic output value
			Yout_Mag_16_bit = (short)(AccelMagData[8] << 8 | AccelMagData[9]);   // Compute 16-bit Y-axis magnetic output value
			Zout_Mag_16_bit = (short)(AccelMagData[10] << 8 | AccelMagData[11]); // Compute 16-bit Z-axis magnetic output value

			// Magnetometer data converted to microteslas
			Xout_uT = (float)(Xout_Mag_16_bit) / SENSITIVITY_MAG; // Compute X-axis output magnetic value in uT
			Yout_uT = (float)(Yout_Mag_16_bit) / SENSITIVITY_MAG; // Compute Y-axis output magnetic value in uT
			Zout_uT = (float)(Zout_Mag_16_bit) / SENSITIVITY_MAG; // Compute Z-axis output magnetic value in uT

			// Compute Yaw angle en degr�s, axe Z
			*aYaw = (atan2(Yout_uT, Xout_uT)) * (180 / PI);
			// Compute Roll angle en degr�s, axe X
			*aRoll = (atan2(Yout_g, sqrt((Xout_g * Xout_g) + (Zout_g * Zout_g)))) * (180 / PI);
			// Compute Yaw angle en degr�s, axe Y
			*aPitch = (atan2(Xout_g, sqrt((Yout_g * Yout_g) + (Zout_g * Zout_g)))) * (180 / PI);

			// Retourne les infos � l'appelant
			aAccel->x = Xout_g;
			aAccel->y = Yout_g;
			aAccel->z = Zout_g;
			aMagneto->x = Xout_uT;
			aMagneto->y = Yout_uT;
			aMagneto->z = Zout_uT;
		}
		// Reset du flag
		//sDataReady = 0;
	}

	return aRet;
}

//-----------------------------------------------------------------------------
// PORT B Interrupt handler --> FXOS8700CQ datas ready
//-----------------------------------------------------------------------------
void PORTB_IRQHandler(void)
{
	bool aRet;

	PORTB->PCR[2] |= PORT_PCR_ISF_MASK; // Clear the interrupt flag
	PORTB->PCR[3] |= PORT_PCR_ISF_MASK; // Clear the interrupt flag

	//aRet=mAccelMagneto_GetData(OUT_X_MSB_REG,AccelMagData,6,(kFXOS8700CQAddr<<1));
	//aRet=mAccelMagneto_GetData(MOUT_X_MSB_REG,AccelMagData,6,(kFXOS8700CQAddr<<1));

	sDataReady = 1;
}
