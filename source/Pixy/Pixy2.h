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
// Arduino ICSP SPI link class

#ifndef _PIXY2_H
#define _PIXY2_H

#include "TPixy2.h"
#include "Interfaces/iSpi.h"

#define PIXY_SPI_CLOCKRATE 2000000

class Link2SPI
{
public:
  int8_t open(uint32_t arg)
  {
    SPI.begin();
    SPI.beginTransaction(SPISettings(PIXY_SPI_CLOCKRATE, MSBFIRST, SPI_MODE3));
    return 0;
  }

  void close()
  {
    SPI.endTransaction();
  }

  int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL)
  {
    uint8_t i;
    if (cs)
      *cs = 0;
    for (i = 0; i < len; i++)
    {
      buf[i] = SPI.transfer(0x00);
      if (cs)
        *cs += buf[i];
    }
    return len;
  }

  int16_t send(uint8_t *buf, uint8_t len)
  {
    uint8_t i;
    for (i = 0; i < len; i++)
      SPI.transfer(buf[i]);
    return len;
  }
};

typedef TPixy2<Link2SPI> Pixy2;

#endif
