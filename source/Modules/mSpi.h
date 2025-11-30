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
Nom du fichier : 	mSpi.h	
Auteur et Date :	Monnerat Serge 29 ao�t 201x

But : 

-----------------------------------------------------------------------------
Historique:
-----------------------------------------------------------------------------

$History: $


-----------------------------------------------------------------------------
*/

#ifndef __mSpi__
#define __mSpi__

#include "Utils/def.h" //KM

// Type des devices sur le SPI0
typedef enum
{
	kCamera1,
	kCamera2
} CameraEnum;

//-----------------------------------------------------------------------
// SPI setup
//-----------------------------------------------------------------------
void mSpi_Setup(void);

//-----------------------------------------------------------------------
// SPI open
//-----------------------------------------------------------------------
void mSpi_Open(void);

//-----------------------------------------------------------------------
// SPI close
//-----------------------------------------------------------------------
void mSpi_Close(void);

//-----------------------------------------------------------------------
// SPI send and read one frame
//-----------------------------------------------------------------------
UInt16 mSpi_SP1ReadWrite(UInt16 aData);

//-----------------------------------------------------------------------
// SPI send and read one frame
//-----------------------------------------------------------------------
UInt16 mSpi_SP0ReadWrite(UInt16 aData);

//-----------------------------------------------------------------------
// Digital camera sending and receiving MLX75306
//-----------------------------------------------------------------------
UInt32 mSpi_MLX75306_RXTX(CameraEnum aCam, UInt32 aData);

//-----------------------------------------------------------------------
// Reset the digital camera MLX75306
//-----------------------------------------------------------------------
void mSpi_MLX75306_Reset(void);

//-----------------------------------------------------------------------
// Acquisition de la lumi�re --> cam�ra dig MLX75306
// Le temps d'int�gration: aIntVal (entre 0 et 65535) / 10MHz
// 32765 --> 6.5 ms SI normal
// 65535 --> 104.8 ms Long SI
//-----------------------------------------------------------------------
void mSpi_MLX75306_StartIntegration(CameraEnum aCam, float aMs);

//-----------------------------------------------------------------------
// Acquisition de la lumi�re --> cam�ra dig MLX75306
// Le temps d'int�gration: aIntVal (entre 0 et 65535) / 10MHz
// 65535 --> 6.5 ms
//-----------------------------------------------------------------------
void mSpi_MLX75306_StartIntegration_old(CameraEnum aCam, float aAdVal);

//-----------------------------------------------------------------------
// Lecture d'une image de la cam�ra dig MLX75306
//-----------------------------------------------------------------------
void mSpi_MLX75306_ReadPicture(CameraEnum aCam, UInt8 *aTab);

//-----------------------------------------------------------------------
// Lecture d'une image de la cam�ra dig MLX75306
// Tableau de 16 bits pour l'app Labview
//-----------------------------------------------------------------------
void mSpi_MLX75306_ReadPictureTest(CameraEnum aCam, UInt16 *aTab);

//-----------------------------------------------------------------------
// Acquiring the non-blocking image of the camera 1
// The acquisition time comes from the potentiometers
//-----------------------------------------------------------------------
bool mSpi_GetImageCam1_Pot(float aAdVal, UInt16 *aTab, UInt8 *aPixelValMoy);

//-----------------------------------------------------------------------
// Acquiring the non-blocking image of the camera 2
// The acquisition time comes from the potentiometers
//-----------------------------------------------------------------------
bool mSpi_GetImageCam2_Pot(float aAdVal, UInt16 *aTab, UInt8 *aPixelValMoy);

//-----------------------------------------------------------------------
// Acquisition de l'image non bloquante de la cam�ra 1
// Le temps d'acquisition est donn� en ms
// Le temps d'int�gration: aIntVal (entre 0 et 65535) / 10MHz
// 32765 --> 6.5 ms SI normal
// 65535 --> 104.8 ms Long SI
//-----------------------------------------------------------------------
bool mSpi_GetImageCam1(float aAdVal, UInt16 *aTab, UInt8 *aPixelValMoy);

//-----------------------------------------------------------------------
// Acquisition de l'image non bloquante de la cam�ra 2
// Le temps d'acquisition est donn� en ms
// Le temps d'int�gration: aIntVal (entre 0 et 65535) / 10MHz
// 32765 --> 6.5 ms SI normal
// 65535 --> 104.8 ms Long SI
//-----------------------------------------------------------------------
bool mSpi_GetImageCam2(float aAdVal, UInt16 *aTab, UInt8 *aPixelValMoy);

#endif
