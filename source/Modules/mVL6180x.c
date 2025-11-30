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
/*---------------------------------------------------------------------------
File name 			: mVL6180x.c
Author				: Baertschi Jonas & Cestele Yves
Date				: 13 janv. 2017
Description			: Description in mVL6180x.h file

Modifications		: -

---------------------------------------------------------------------------*/

#include "Interfaces/iI2C.h" //KM
#include "Utils/def.h"		 //KM
#include "Modules/mVL6180x.h"

static bool mI2C_MPL_GetData(UInt8 aReg, UInt8 *aData);
static bool mI2C_QQCH_SetData(UInt16 aReg, UInt8 aVal);

//--------------------------------------------------------------
// 				DESCRIPTION DES FUNCTIONS
//--------------------------------------------------------------

//--------------------------------------------------------------
// Name			: mVL6180x_Setup
// Author		: Baertschi Jonas & Cestele Yves
// Date			: 13 janv. 2017
// Descritpion	: I2C bus setup
//
// Return		: void
//
// Modification	:
//--------------------------------------------------------------
void mVL6180x_Setup(void)
{
	iI2C_Config();
}

//--------------------------------------------------------------
// Name			: mVL6180x_Open
// Author		: Baertschi Jonas & Cestele Yves
// Date			: 13 janv. 2017
// Descritpion	: Open the I2C bus
//
// Return		: void
//
// Modification	:
//--------------------------------------------------------------
void mVL6180x_Open(void)
{
	iI2C_Enable();
}

//--------------------------------------------------------------
// Name			: mVL6180x_Close
// Author		: Baertschi Jonas & Cestele Yves
// Date			: 13 janv. 2017
// Descritpion	: Close the bus
//
// Return		: void
//
// Modification	:
//--------------------------------------------------------------
void mVL6180x_Close(void)
{
}

//--------------------------------------------------------------
// Name			: mVL6180x_StartRange
// Author		: Baertschi Jonas & Cestele Yves
// Date			: 13 janv. 2017
// Descritpion	: Setup the VL61 module
//
// Return		: bool
//
// Modification	:
//--------------------------------------------------------------
bool mVL6180x_StartRange(void)
{
	mI2C_QQCH_SetData(0x0207, 0x01);
	mI2C_QQCH_SetData(0x0208, 0x01);
	mI2C_QQCH_SetData(0x0096, 0x00);
	mI2C_QQCH_SetData(0x0097, 0xfd);
	mI2C_QQCH_SetData(0x00e3, 0x00);
	mI2C_QQCH_SetData(0x00e4, 0x04);
	mI2C_QQCH_SetData(0x00e5, 0x02);
	mI2C_QQCH_SetData(0x00e6, 0x01);
	mI2C_QQCH_SetData(0x00e7, 0x03);
	mI2C_QQCH_SetData(0x00f5, 0x02);
	mI2C_QQCH_SetData(0x00d9, 0x05);
	mI2C_QQCH_SetData(0x00db, 0xce);
	mI2C_QQCH_SetData(0x00dc, 0x03);
	mI2C_QQCH_SetData(0x00dd, 0xf8);
	mI2C_QQCH_SetData(0x009f, 0x00);
	mI2C_QQCH_SetData(0x00a3, 0x3c);
	mI2C_QQCH_SetData(0x00b7, 0x00);
	mI2C_QQCH_SetData(0x00bb, 0x3c);
	mI2C_QQCH_SetData(0x00b2, 0x09);
	mI2C_QQCH_SetData(0x00ca, 0x09);
	mI2C_QQCH_SetData(0x0198, 0x01);
	mI2C_QQCH_SetData(0x01b0, 0x17);
	mI2C_QQCH_SetData(0x01ad, 0x00);
	mI2C_QQCH_SetData(0x00ff, 0x05);
	mI2C_QQCH_SetData(0x0100, 0x05);
	mI2C_QQCH_SetData(0x0199, 0x05);
	mI2C_QQCH_SetData(0x01a6, 0x1b);
	mI2C_QQCH_SetData(0x01ac, 0x3e);
	mI2C_QQCH_SetData(0x01a7, 0x1f);
	mI2C_QQCH_SetData(0x0030, 0x00);

	mI2C_QQCH_SetData(0x0011, 0x10);
	mI2C_QQCH_SetData(0x010A, 0x30);
	mI2C_QQCH_SetData(0x003F, 0x46);
	mI2C_QQCH_SetData(0x0031, 0xFF);
	mI2C_QQCH_SetData(0x0040, 0x63);
	mI2C_QQCH_SetData(0x002E, 0x01);
	mI2C_QQCH_SetData(0x001B, 0x09);
	mI2C_QQCH_SetData(0x003E, 0x00); // 0=10mS
	mI2C_QQCH_SetData(0x0014, 0x24);

	mI2C_QQCH_SetData(0x0016, 0x00);

	// D�marrage

	//	mI2C_QQCH_SetData(kVL6180X_REG_SYSRANGE_START,0x03);	//Continuous mode
}

//--------------------------------------------------------------
// Name			: mVL6180x_GetRange
// Author		: Baertschi Jonas & Cestele Yves
// Date			: 13 janv. 2017
// Descritpion	: Return the distance measured by the captor
//
// Parameters	: UInt8*	distance in mm
//
// Return		: bool
//
// Modification	:
//--------------------------------------------------------------
bool mVL6180x_GetRange(UInt8 *aData)
{
	bool aRet;
	static UInt8 aState = 0;
	UInt8 aDataReady = 0;

	switch (aState)
	{
	case 0: // On demande la lecture de la distance
		mI2C_QQCH_SetData(0x0018, 0x01);
		aState = 1;
		break;

	case 1: // On attend qu'elle soit pr�te
		mI2C_MPL_GetData(0x4F, &aDataReady);
		if ((aDataReady & 0x04) == 0x04)
		{
			aState = 2;
		}
		break;

	case 2: // On lit et renvoie la valeur mesuree
		mI2C_MPL_GetData(0x62, aData);
		mI2C_QQCH_SetData(0x15, 0x07);
		aState = 0;
		break;
	}
	return true;
}

//--------------------------------------------------------------
// Static functions to send / receive Datas on the I2C Bus
//--------------------------------------------------------------

static bool mI2C_MPL_GetData(UInt8 aReg, UInt8 *aData)
{

	// Attente que le bus soit libre
	while (iI2C_ReadStatus(kBUSY))
		;

	// on passe en Tx
	iI2C_SetAckMode(kNoAck);

	iI2C_TxRxSelect(kTxMode);

	// ADR
	iI2C_SetStartState();
	iI2C_SendData(kVL6180x_w_addr | 0x00);
	iI2C_WaitEndOfRxOrTx();
	if (iI2C_ReadStatus(RxAK))
	{
		iI2C_TxRxSelect(kTxMode);
		iI2C_SetAckMode(kNoAck);
		iI2C_SetStopState();
		return false;
	}

	// on demande un registre MSB
	iI2C_SendData(0x00);
	iI2C_WaitEndOfRxOrTx();
	if (iI2C_ReadStatus(RxAK))
	{
		iI2C_TxRxSelect(kTxMode);
		iI2C_SetAckMode(kNoAck);
		iI2C_SetStopState();
		return false;
	}

	// on demande un registre LSB
	iI2C_SendData(aReg);
	iI2C_WaitEndOfRxOrTx();
	if (iI2C_ReadStatus(RxAK))
	{
		iI2C_TxRxSelect(kTxMode);
		iI2C_SetAckMode(kNoAck);
		iI2C_SetStopState();
		return false;
	}

	// on donne le droit de parler

	iI2C_SetRepeatedStartSate();

	iI2C_SendData(kVL6180x_w_addr | 0x01);

	// Wait end of transmission
	iI2C_WaitEndOfRxOrTx();
	if (iI2C_ReadStatus(RxAK))
	{
		iI2C_TxRxSelect(kTxMode);
		iI2C_SetAckMode(kNoAck);
		iI2C_SetStopState();
		return false;
	}

	// on veut lire qu'une donn�e:
	iI2C_TxRxSelect(kRxMode);

	//nack
	*aData = iI2C_ReadData();

	iI2C_WaitEndOfRxOrTx();

	iI2C_TxRxSelect(kTxMode);
	iI2C_SetAckMode(kNoAck);
	// Stop
	iI2C_SetStopState();

	// lecture
	*aData = iI2C_ReadData();

	return true;
}

static bool mI2C_QQCH_SetData(UInt16 aReg, UInt8 aVal)
{
	// Pas de ack:
	iI2C_SetAckMode(kNoAck);

	// Attente que le bus soit libre:
	while (iI2C_ReadStatus(kBUSY))
		;

	// On passe en master et d�but de la transmission:
	iI2C_TxRxSelect(kTxMode);
	iI2C_SetStartState();

	// Envois de l'adr en �criture:
	iI2C_SendData(kVL6180x_w_addr | 0x00);
	iI2C_WaitEndOfRxOrTx();
	if (iI2C_ReadStatus(RxAK))
	{
		// FIN de la lecture
		iI2C_TxRxSelect(kTxMode);
		iI2C_SetAckMode(kNoAck);
		iI2C_SetStopState();
		return false;
	}

	// Transmission des donn�es: MSB
	iI2C_SendData((aReg >> 8) & 0x00FF);
	iI2C_WaitEndOfRxOrTx();
	if (iI2C_ReadStatus(RxAK))
	{
		// FIN de la lecture
		iI2C_TxRxSelect(kTxMode);
		iI2C_SetAckMode(kNoAck);
		iI2C_SetStopState();
		return false;
	}

	// Transmission des donn�es: LSB
	iI2C_SendData(aReg & 0x00FF);
	iI2C_WaitEndOfRxOrTx();
	if (iI2C_ReadStatus(RxAK))
	{
		// FIN de la lecture
		iI2C_TxRxSelect(kTxMode);
		iI2C_SetAckMode(kNoAck);
		iI2C_SetStopState();
		return false;
	}

	// Envoi de la valeur a �crire dans le registre:
	iI2C_SendData(aVal);
	iI2C_WaitEndOfRxOrTx();

	// Fin de la transmission:
	iI2C_TxRxSelect(kTxMode);
	iI2C_SetAckMode(kNoAck);
	iI2C_SetStopState();

	return true;
}
