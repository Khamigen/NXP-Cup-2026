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
Nom du fichier : 	mRS232.h	
Auteur et Date :	Monnerat Serge 26 janv. 201x

But : 

-----------------------------------------------------------------------------
Historique:
-----------------------------------------------------------------------------

$History: $


-----------------------------------------------------------------------------
*/
#ifndef __mRS232__
#define __mRS232__

#include "Utils/def.h" //KM

//-----------------------------------------------------------------------------
// RS232 module setup
//-----------------------------------------------------------------------------
void mRs232_Setup(void);

//-----------------------------------------------------------------------------
// RS232 module close
//-----------------------------------------------------------------------------
void mRs232_Open(void);

//-----------------------------------------------------------------------------
// RS232 module close
//-----------------------------------------------------------------------------
void mRs232_Close(void);

//-----------------------------------------------------------------------------
// String send
// The transmission stop at the char NULL
// *aDataPtr	: string address
//-----------------------------------------------------------------------------
void mRs232_Uart4WriteString(Int8 *aDataPtr);

//-----------------------------------------------------------------------------
// Byte send
// aData	: byte to send
//-----------------------------------------------------------------------------
void mRs232_Uart4WriteChar(Int8 aData);

//-----------------------------------------------------------------------------
// Buffer byte read. Rx interrupt fill the buffer
// *aBytePtr	  : pointer to return the byte
// Return 	    : true --> the buffer is empty, false the buffer is not empty
//-----------------------------------------------------------------------------
bool mRs232_Uart4ReadDataFromBuffer(Int8 *aBytePtr);

#endif
