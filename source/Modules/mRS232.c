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
Nom du fichier :	mRS232.c
Auteur et Date :	Monnerat Serge 26 janv. 201x

Description dans le fichier d'ent�te (.h)

-----------------------------------------------------------------------------
Historique:
-----------------------------------------------------------------------------

$History: $

-----------------------------------------------------------------------------
*/

#include "mRS232.h"
#include "Interfaces/iUart.h"

//-----------------------------------------------------------------------
// UART module configuration
//-----------------------------------------------------------------------
void mRs232_Setup(void)
{
}

//-----------------------------------------------------------------------
// UART module open
//-----------------------------------------------------------------------
void mRs232_Open(void)
{
}

//-----------------------------------------------------------------------
// UART 2 module close
//-----------------------------------------------------------------------
void mRs232_Close(void)
{
}

//-----------------------------------------------------------------------------
// Uart4, String send
// The transmission stop at the char NULL
// *aDataPtr	: string address
//-----------------------------------------------------------------------------
void mRs232_Uart4WriteString(Int8 *aDataPtr)
{
	// Send char until NULL char
	while (*aDataPtr != 0)
	{
		// Wait end of transmit
		while (!iUart_Uart4GetStatus(kSciTransmitComplete))
			;

		// Write data
		iUart_Uart4SetData(*aDataPtr);

		// Next char
		aDataPtr++;
	}
}

//-----------------------------------------------------------------------------
// Uart4 Byte send
// aData	: byte to send
//-----------------------------------------------------------------------------
void mRs232_Uart4WriteChar(Int8 aData)
{
	// Wait end of transmit
	while (!iUart_Uart4GetStatus(kSciTransmitComplete))
		;

	// Write data
	iUart_Uart4SetData(aData);
}
