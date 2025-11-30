/****************************************************************************
 *
 * Copyright 2020 Charmed Labs.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
//
// Arduino SPI with slave select link class

#ifndef _PIXY2SPI_SS_H
#define _PIXY2SPI_SS_H

#include "TPixy2.h"
extern "C"
{
#include "Interfaces/iSpi.h"
#include "Interfaces/iDio.h"
#include "Modules/mSpi.h"
}
#define PIXY_SPI_CLOCKRATE 2000000

class Link2SPI_SS
{
public:
  int8_t open(uint32_t arg)
  {
    /* if (arg==PIXY_DEFAULT_ARGVAL)
      ssPin = SS; // default slave select pin
	else
	  ssPin = arg;
    pinMode(ssPin, OUTPUT);
    SPI.begin();
    SPI.beginTransaction(SPISettings(PIXY_SPI_CLOCKRATE, MSBFIRST, SPI_MODE1));
    */
    return 0;
  }

  void close()
  {
    mSpi_Close();
  }

  int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL)
  {
    uint8_t i;
    if (cs)
    {
      *cs = 0;
    }
    // CS camera OFF
    iDio_SetPort(kPortD, kMaskIo4, kIoOff);

    for (i = 0; i < len; i++)
    {
      buf[i] = (uint8_t)mSpi_SP1ReadWrite(0x00);
      if (cs)
        *cs += buf[i];
    }
    // CS camera ON
    iDio_SetPort(kPortD, kMaskIo4, kIoOn);

    return len;
  }

  int16_t send(uint8_t *buf, uint8_t len)
  {
    uint8_t i;

    // CS camera OFF
    iDio_SetPort(kPortD, kMaskIo4, kIoOff);
    for (i = 0; i < len; i++)
    {
      mSpi_SP1ReadWrite(buf[i]);
    }
    // CS camera ON
    iDio_SetPort(kPortD, kMaskIo4, kIoOn);
    return len;
  }

  void setArg(uint16_t arg)
  {
  }

private:
  uint16_t ssPin;
};

typedef TPixy2<Link2SPI_SS> Pixy2SPI_SS;

#endif
