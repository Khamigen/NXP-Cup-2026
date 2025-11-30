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
// Arduino I2C link class

#ifndef _PIXY2I2C_H
#define _PIXY2I2C_H

#include "TPixy2.h"
//#include "Wire.h"
extern "C"
{
#include "Interfaces/iI2C.h"
#include "Interfaces/iDio.h"
}

#define PIXY_I2C_DEFAULT_ADDR 0x54
#define PIXY_I2C_MAX_SEND 16 // don't send any more than 16 bytes at a time

class Link2I2C
{
public:
  int8_t open(uint32_t arg) // take I2C address as argument to open
  {
    /*if (arg==PIXY_DEFAULT_ARGVAL)
      m_addr = PIXY_I2C_DEFAULT_ADDR;
    else
      m_addr = arg;
    Wire.begin();*/

    // I2C config
    iI2C_Config();
    iI2C_Enable();
    return 0;
  }

  void close()
  {
  }

  int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL)
  {
    uint8_t i, j, n;
    if (cs)
      *cs = 0;
    for (i = 0; i < len; i += n)
    {
      // n is the number read -- it most likely won't be equal to len
      n = Wire.requestFrom((uint8_t)m_addr, (uint8_t)(len - i));
      for (j = 0; j < n; j++)
      {
        buf[j + i] = Wire.read();
        if (cs)
          *cs += buf[j + i];
      }
    }
    return len;
  }

  int16_t send(uint8_t *buf, uint8_t len)
  {
    int8_t i, packet;
    for (i = 0; i < len; i += PIXY_I2C_MAX_SEND)
    {
      if (len - i < PIXY_I2C_MAX_SEND)
        packet = len - i;
      else
        packet = PIXY_I2C_MAX_SEND;
      Wire.beginTransmission(m_addr);
      Wire.write(buf + i, packet);
      Wire.endTransmission();
    }
    return len;
  }

private:
  uint8_t m_addr;
};

typedef TPixy2<Link2I2C> Pixy2I2C;

#endif
