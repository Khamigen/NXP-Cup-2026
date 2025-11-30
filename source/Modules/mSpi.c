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
File name :	mSpi.c
Author and date :	Monnerat Serge 29 ao�t 201x

Description in the header (.h)

-----------------------------------------------------------------------------
History:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "Interfaces/iSpi.h"
#include "Interfaces/iDio.h"
#include "Modules/mSpi.h"
#include "MK64F12.h"

typedef enum
{
	kStartIntegration,
	kWaitEndIntegration,
	kGetImage
} CamStateEnum;

static CamStateEnum sCam1PotState = kStartIntegration;
static CamStateEnum sCam2PotState = kStartIntegration;
static CamStateEnum sCam1State = kStartIntegration;
static CamStateEnum sCam2State = kStartIntegration;

//-----------------------------------------------------------------------
// SPI setup
//-----------------------------------------------------------------------
void mSpi_Setup(void)
{
	// Setup du clock et de la fonction des pins
	iSpi_ConfigClockAndPins();

	// SPI config interrupt & DMA
	//iSpi_ConfigInterrupAndDMA(kSpi0);
	iSpi_ConfigInterrupAndDMA(kSpi1);

	// SPI module configuration
	//iSpi_ConfigSPIModule(kSpi0); // Camera 2
	iSpi_ConfigSPIModule(kSpi1); // Camera 1

	// SPI clock and transfer configuration
	//iSpi_ConfigClockAndTransfert(kSpi0,kCTAR1); // Camera 2
	iSpi_ConfigClockAndTransfert(kSpi1, kCTAR1); // Pixy 2 camera wit SPI Mode 3

	// SPI CS inactive state
	//iSpi_ChangeInactiveCSState(kSpi0,kCS4,true);	// Linescan camera 2
	iSpi_ChangeInactiveCSState(kSpi1, kCS0, true); // Pixy 2 camera wit SPI Mode 3
}

//-----------------------------------------------------------------------
// SPI open
//-----------------------------------------------------------------------
void mSpi_Open(void)
{
	// Start TX
	iSpi_StartTX(kSpi0);
	iSpi_StartTX(kSpi1);

	// Spi status flag reset --> EOQF
	iSpi_ResetStatus(kSpi0, kEOQF);
	iSpi_ResetStatus(kSpi1, kEOQF);
}

//-----------------------------------------------------------------------
// SPI close
//-----------------------------------------------------------------------
void mSpi_Close(void)
{
	// Stop TX
	//iSpi_StopTX(kSpi0);
	iSpi_StopTX(kSpi1);
}

//-----------------------------------------------------------------------
// SPI 1 send and read one frame --> Camera 1
//-----------------------------------------------------------------------
UInt16 mSpi_SP1ReadWrite(UInt16 aData)
{
	UInt16 aRxData;

	// Push Data to the TX FIFO
	iSpi_PushDataIntoFifo(kSpi1, kCTAR1, kCONTDis, true, kCS4, aData);

	// Wait end of transmission
	while (iSpi_GetStatus(kSpi1, kRFDF) == false)
		;

	// Read data from the RX FIFO
	aRxData = iSpi_ReadData(kSpi1);

	// Reset SPI flags
	iSpi_ResetStatus(kSpi1, kTCF);
	iSpi_ResetStatus(kSpi1, kRFDF);
	iSpi_ResetStatus(kSpi1, kEOQF);

	return aRxData;
}

//-----------------------------------------------------------------------
// SPI 0 send and read one frame --> Camera 2
//-----------------------------------------------------------------------
UInt16 mSpi_SP0ReadWrite(UInt16 aData)
{
	UInt16 aRxData;

	// Push Data to the TX FIFO
	iSpi_PushDataIntoFifo(kSpi0, kCTAR1, kCONTDis, true, kCS4, aData);

	// Wait end of transmission
	while (iSpi_GetStatus(kSpi0, kRFDF) == false)
		;

	// Read data from the RX FIFO
	aRxData = iSpi_ReadData(kSpi0);

	// Reset SPI flags
	iSpi_ResetStatus(kSpi0, kTCF);
	iSpi_ResetStatus(kSpi0, kRFDF);
	iSpi_ResetStatus(kSpi0, kEOQF);

	return aRxData;
}

//-----------------------------------------------------------------------
// Sending and receiving an order to the camera MLX75306
//-----------------------------------------------------------------------
UInt32 mSpi_MLX75306_RXTX(CameraEnum aCam, UInt32 aData)
{
	UInt16 aRet;
	UInt32 aVal = 0;
	UInt16 aCmd;

	if (aCam == kCamera1)
	{
		// CS camera OFF
		iDio_SetPort(kPortD, kMaskIo4, kIoOff);
		aCmd = (aData & 0x00ff0000) >> 16;
		aRet = mSpi_SP1ReadWrite(aCmd);
		aVal = (aRet & 0x00FF) << 16;
		aCmd = (aData & 0x0000ff00) >> 8;
		aRet = mSpi_SP1ReadWrite(aCmd);
		aVal |= ((aRet & 0x00FF) << 8);
		aCmd = (aData & 0x000000ff);
		aRet = mSpi_SP1ReadWrite(aCmd);
		aVal |= (aRet & 0x00FF);
		// CS camera ON
		iDio_SetPort(kPortD, kMaskIo4, kIoOn);
	}
	else if (aCam == kCamera2)
	{
		// CS camera OFF
		iDio_SetPort(kPortC, kMaskIo0, kIoOff);
		aCmd = (aData & 0x00ff0000) >> 16;
		aRet = mSpi_SP0ReadWrite(aCmd);
		aVal = (aRet & 0x00FF) << 16;
		aCmd = (aData & 0x0000ff00) >> 8;
		aRet = mSpi_SP0ReadWrite(aCmd);
		aVal |= ((aRet & 0x00FF) << 8);
		aCmd = (aData & 0x000000ff);
		aRet = mSpi_SP0ReadWrite(aCmd);
		aVal |= (aRet & 0x00FF);
		// CS camera ON
		iDio_SetPort(kPortC, kMaskIo0, kIoOn);
	}
	return aVal;
}

//-----------------------------------------------------------------------
// Lecture d'une image de la cam�ra dig MLX75306
//-----------------------------------------------------------------------
void mSpi_MLX75306_ReadPicture(CameraEnum aCam, UInt8 *aTab)
{
	UInt16 i;

	if (aCam == kCamera1)
	{
		// CS camera OFF
		iDio_SetPort(kPortD, kMaskIo4, kIoOff);
		// Read out command 8-bit resolution
		aTab[0] = mSpi_SP1ReadWrite(0x99);
		// 1er pixel --> 2
		aTab[1] = mSpi_SP1ReadWrite(0x02);
		// latest pixel --> 143
		aTab[2] = mSpi_SP1ReadWrite(0x8F);

		for (i = 0; i < 156; i++)
			// read out pixel
			aTab[i + 3] = mSpi_SP1ReadWrite(0x0);

		// CS camera ON
		iDio_SetPort(kPortD, kMaskIo4, kIoOn);
	}
	else if (aCam == kCamera2)
	{
		// CS camera OFF
		iDio_SetPort(kPortC, kMaskIo0, kIoOff);
		// Commande read out r�solution 8 bits
		aTab[0] = mSpi_SP0ReadWrite(0x99);
		// 1er pixel --> 2
		aTab[1] = mSpi_SP0ReadWrite(0x02);
		// dernier pixel --> 143
		aTab[2] = mSpi_SP0ReadWrite(0x8F);

		for (i = 0; i < 156; i++)
			// read out pixel
			aTab[i + 3] = mSpi_SP0ReadWrite(0x0);

		// CS camera ON
		iDio_SetPort(kPortC, kMaskIo0, kIoOn);
	}
}

//-----------------------------------------------------------------------
// Lecture d'une image de la cam�ra dig MLX75306
// Tableau de 16 bits pour l'app Labview
//-----------------------------------------------------------------------
void mSpi_MLX75306_ReadPictureTest(CameraEnum aCam, UInt16 *aTab)
{
	UInt16 i;

	if (aCam == kCamera1)
	{
		// CS camera OFF
		iDio_SetPort(kPortD, kMaskIo4, kIoOff);
		// Commande read out r�solution 8 bits
		aTab[0] = mSpi_SP0ReadWrite(0x99);
		// 1er pixel --> 2
		aTab[1] = mSpi_SP0ReadWrite(0x02);
		// dernier pixel --> 143
		aTab[2] = mSpi_SP0ReadWrite(0x8F);

		for (i = 0; i < 156; i++)
			// read out pixel
			aTab[i + 3] = mSpi_SP0ReadWrite(0x0);

		// CS camera ON
		iDio_SetPort(kPortD, kMaskIo4, kIoOn);
	}
	else if (aCam == kCamera2)
	{
		// CS camera OFF
		iDio_SetPort(kPortC, kMaskIo0, kIoOff);
		// Commande read out r�solution 8 bits
		aTab[0] = mSpi_SP0ReadWrite(0x99);
		// 1er pixel --> 2
		aTab[1] = mSpi_SP0ReadWrite(0x02);
		// dernier pixel --> 143
		aTab[2] = mSpi_SP0ReadWrite(0x8F);

		for (i = 0; i < 156; i++)
			// read out pixel
			aTab[i + 3] = mSpi_SP0ReadWrite(0x0);

		// CS camera ON
		iDio_SetPort(kPortC, kMaskIo0, kIoOn);
	}
}

//-----------------------------------------------------------------------
// Reset de la cam�ra dig MLX75306
//-----------------------------------------------------------------------
void mSpi_MLX75306_Reset(void)
{
	// Reset --> attendre Twakeup de 500us apr�s le reset
	mSpi_MLX75306_RXTX(kCamera1, 0xF00000);
	mSpi_MLX75306_RXTX(kCamera2, 0xF00000);
}

//-----------------------------------------------------------------------
// Acquisition of light -> camera MLX75306
// Integration time: aIntVal (between 0 and 65535) / 10MHz
// 32765 -> 6.5 ms IF normal
// 65535 -> 104.8 ms Long SI
//-----------------------------------------------------------------------
void mSpi_MLX75306_StartIntegration(CameraEnum aCam, float aMs)
{
	UInt32 aVal;
	if (aCam == kCamera1)
	{
		if (aMs <= 6.5)
		{
			aVal = (aMs * 10000.0) + 4;
			aVal &= 0xFFFF;
			aVal = 0xB80000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera1, aVal);
		}
		else
		{
			aVal = (((aMs * 10000.0) - 11.0) / 16.0);
			aVal &= 0xFFFF;
			aVal = 0xB40000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera1, aVal);
		}

		// Attend que l'image soit pr�te
		while ((GPIOD->PDIR & 0x08) == 0x00)
			;
	}
	else if (aCam == kCamera2)
	{
		if (aMs <= 6.5)
		{
			aVal = (aMs * 10000.0) + 4;
			aVal &= 0xFFFF;
			aVal = 0xB80000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera2, aVal);
		}
		else
		{
			aVal = (((aMs * 10000.0) - 11.0) / 16.0);
			aVal &= 0xFFFF;
			aVal = 0xB40000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera2, aVal);
		}

		// Attend que l'image soit pr�te
		while ((GPIOD->PDIR & 0x04) == 0x00)
			;
	}
}

//-----------------------------------------------------------------------
// Acquisition de la lumi�re --> cam�ra dig MLX75306
// Le temps d'int�gration: aIntVal (entre 0 et 65535) / 10MHz
// 65535 --> 6.5 ms SI normal
// 65535 --> 104.8 ms Long SI
//-----------------------------------------------------------------------
void mSpi_MLX75306_StartIntegration_old(CameraEnum aCam, float aAdVal)
{
	UInt32 aVal;
	float aMs;

	if (aCam == kCamera1)
	{
		if (aAdVal <= 32765.0)
		{
			aMs = ((aAdVal * 6.5) / 32765.0);
			aVal = (aMs * 10000.0) + 4;
			aVal &= 0xFFFF;
			aVal = 0xB80000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera1, aVal);
		}
		else
		{
			aAdVal -= 32766;
			aMs = aAdVal * 0.003 + 6.5;
			aVal = (((aMs * 10000.0) - 11.0) / 16.0);
			aVal &= 0xFFFF;
			aVal = 0xB40000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera1, aVal);
		}

		// Attend que l'image soit pr�te
		while ((GPIOD->PDIR & 0x08) == 0x00)
			;
	}
	else if (aCam == kCamera2)
	{
		if (aAdVal <= 32765.0)
		{
			aMs = ((aAdVal * 6.5) / 32765.0);
			aVal = (aMs * 10000.0) + 4;
			aVal &= 0xFFFF;
			aVal = 0xB80000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera2, aVal);
		}
		else
		{
			aAdVal -= 32766;
			aMs = aAdVal * 0.003 + 6.5;
			aVal = (((aMs * 10000.0) - 11.0) / 16.0);
			aVal &= 0xFFFF;
			aVal = 0xB40000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera2, aVal);
		}

		// Attend que l'image soit pr�te
		while ((GPIOD->PDIR & 0x04) == 0x00)
			;
	}
}

//-----------------------------------------------------------------------
// Acquiring the non-blocking image of camera 1
// The acquisition time comes from the potentiometers
//-----------------------------------------------------------------------
bool mSpi_GetImageCam1_Pot(float aAdVal, UInt16 *aTab, UInt8 *aPixelValMoy)
{
	UInt32 aVal;
	float aMs;
	bool aOk = false;

	switch (sCam1PotState)
	{
	// Statr de l'exposition � la lumi�re
	case kStartIntegration:

		// image pas pr�te
		aOk = false;

		if (aAdVal <= 32765.0)
		{
			aMs = ((aAdVal * 6.5) / 32765.0);
			aVal = (aMs * 10000.0) + 4;
			aVal &= 0xFFFF;
			aVal = 0xB80000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera1, aVal);
		}
		else
		{
			aAdVal -= 32766;
			aMs = aAdVal * 0.003 + 6.5;
			aVal = (((aMs * 10000.0) - 11.0) / 16.0);
			aVal &= 0xFFFF;
			aVal = 0xB40000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera1, aVal);
		}

		// Next state
		sCam1PotState = kWaitEndIntegration;

		break;
	// Attend la fin de l'exposition
	case kWaitEndIntegration:

		// image pas pr�te
		aOk = false;

		// Attend que l'image soit pr�te
		if ((GPIOD->PDIR & 0x08) == 0x08)
		{
			// Next state
			sCam1PotState = kGetImage;
		}

		break;
	case kGetImage:
		// image pr�te
		aOk = true;
		// Test de la cam�ra
		mSpi_MLX75306_ReadPictureTest(kCamera1, aTab);
		// Valeur moyenne des pixels
		*aPixelValMoy = aTab[156];
		// Next state
		sCam1PotState = kStartIntegration;
		break;
	}
	return aOk;
}

//-----------------------------------------------------------------------
// Acquiring the non-blocking image of camera 2
// The acquisition time comes from the potentiometers
//-----------------------------------------------------------------------
bool mSpi_GetImageCam2_Pot(float aAdVal, UInt16 *aTab, UInt8 *aPixelValMoy)
{
	UInt32 aVal;
	float aMs;
	bool aOk = false;

	switch (sCam2PotState)
	{
	// Statr de l'exposition � la lumi�re
	case kStartIntegration:

		// image pas pr�te
		aOk = false;

		if (aAdVal <= 32765.0)
		{
			aMs = ((aAdVal * 6.5) / 32765.0);
			aVal = (aMs * 10000.0) + 4;
			aVal &= 0xFFFF;
			aVal = 0xB80000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera2, aVal);
		}
		else
		{
			aAdVal -= 32766;
			aMs = aAdVal * 0.003 + 6.5;
			aVal = (((aMs * 10000.0) - 11.0) / 16.0);
			aVal &= 0xFFFF;
			aVal = 0xB40000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera2, aVal);
		}

		// Next state
		sCam2PotState = kWaitEndIntegration;

		break;
	// Attend la fin de l'exposition
	case kWaitEndIntegration:

		// image pas pr�te
		aOk = false;
		// Attend que l'image soit pr�te
		if ((GPIOD->PDIR & 0x04) == 0x04)
		{
			// Next state
			sCam2PotState = kGetImage;
		}

		break;
	case kGetImage:

		// image pr�te
		aOk = true;
		// Test de la cam�ra
		mSpi_MLX75306_ReadPictureTest(kCamera2, aTab);
		// Valeur moyenne des pixels
		*aPixelValMoy = aTab[156];
		// Next state
		sCam2PotState = kStartIntegration;

		break;
	}
	return aOk;
}

//-----------------------------------------------------------------------
// Acquisition de l'image non bloquante de la cam�ra 1
// Le temps d'acquisition est donn� en ms
// Le temps d'int�gration: aIntVal (entre 0 et 65535) / 10MHz
// 32765 --> 6.5 ms SI normal
// 65535 --> 104.8 ms Long SI
//-----------------------------------------------------------------------
bool mSpi_GetImageCam1(float aAdVal, UInt16 *aTab, UInt8 *aPixelValMoy)
{
	UInt32 aVal;
	float aMs;
	bool aOk = false;

	switch (sCam1State)
	{
	// Statr de l'exposition � la lumi�re
	case kStartIntegration:

		// image pas pr�te
		aOk = false;

		if (aMs <= 6.5)
		{
			aVal = (aMs * 10000.0) + 4;
			aVal &= 0xFFFF;
			aVal = 0xB80000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera1, aVal);
		}
		else
		{
			aVal = (((aMs * 10000.0) - 11.0) / 16.0);
			aVal &= 0xFFFF;
			aVal = 0xB40000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera1, aVal);
		}

		// Next state
		sCam1State = kWaitEndIntegration;

		break;
	// Attend la fin de l'exposition
	case kWaitEndIntegration:

		// image pas pr�te
		aOk = false;

		// Attend que l'image soit pr�te
		if ((GPIOD->PDIR & 0x08) == 0x08)
		{
			// Next state
			sCam1State = kGetImage;
		}

		break;
	case kGetImage:

		// image pr�te
		aOk = true;
		// Test de la cam�ra
		mSpi_MLX75306_ReadPictureTest(kCamera1, aTab);
		// Valeur moyenne des pixels
		*aPixelValMoy = aTab[156];
		// Next state
		sCam1State = kStartIntegration;

		break;
	}
	return aOk;
}

//-----------------------------------------------------------------------
// Acquisition de l'image non bloquante de la cam�ra 2
// Le temps d'acquisition est donn� en ms
// Le temps d'int�gration: aIntVal (entre 0 et 65535) / 10MHz
// 32765 --> 6.5 ms SI normal
// 65535 --> 104.8 ms Long SI
//-----------------------------------------------------------------------
bool mSpi_GetImageCam2(float aAdVal, UInt16 *aTab, UInt8 *aPixelValMoy)
{
	UInt32 aVal;
	float aMs;
	bool aOk = false;

	switch (sCam2State)
	{
	// Statr de l'exposition � la lumi�re
	case kStartIntegration:

		// image pas pr�te
		aOk = false;

		if (aMs <= 6.5)
		{
			aVal = (aMs * 10000.0) + 4;
			aVal &= 0xFFFF;
			aVal = 0xB80000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera2, aVal);
		}
		else
		{
			aVal = (((aMs * 10000.0) - 11.0) / 16.0);
			aVal &= 0xFFFF;
			aVal = 0xB40000 | (aVal);
			// Start int�gration
			mSpi_MLX75306_RXTX(kCamera2, aVal);
		}

		// Next state
		sCam2State = kWaitEndIntegration;

		break;
	// Attend la fin de l'exposition
	case kWaitEndIntegration:

		// image pas pr�te
		aOk = false;
		// Attend que l'image soit pr�te
		if ((GPIOD->PDIR & 0x04) == 0x04)
		{
			// Next state
			sCam2State = kGetImage;
		}

		break;
	case kGetImage:

		// image pr�te
		aOk = true;
		// Test de la cam�ra
		mSpi_MLX75306_ReadPictureTest(kCamera2, aTab);
		// Valeur moyenne des pixels
		*aPixelValMoy = aTab[156];
		// Next state
		sCam2State = kStartIntegration;

		break;
	}
	return aOk;
}
