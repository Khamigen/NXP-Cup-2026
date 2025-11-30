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
// Arduino UART link class, intended to be used with an Arduino with more than 1 UART,
// like the Arduino MEGA 2560.

#ifndef _PIXY2UART_H
#define _PIXY2UART_H

#include "TPixy2.h"
extern "C"
{
#include "Modules/mDelay.h"
#include "Modules/mWifi.h"
#include "Interfaces/iUart.h"
}
//#include "Arduino.h"

#define PIXY_UART_BAUDRATE 19200

class Link2UART
{
public:
  int8_t open(uint32_t arg)
  {
    /*if (arg==PIXY_DEFAULT_ARGVAL)
      Serial1.begin(PIXY_UART_BAUDRATE);
    else
      Serial1.begin(arg);*/
    mWifi_Setup();
    mWifi_Open();
    return 0;
  }

  void close()
  {
  }

  int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL)
  {
    uint8_t i, j;
    int16_t c;
    if (cs)
      *cs = 0;
    for (i = 0; i < len; i++)
    {
      // wait for byte, timeout after 2ms
      // note for a baudrate of 19.2K, each byte takes about 500us
      for (j = 0; true; j++)
      {
        if (j == 200)
          return -1;
        c = iUart_Uart4ReadData();
        if (c >= 0)
          break;
        //msleep(1);
      }
      buf[i] = c;

      if (cs)
        *cs += buf[i];
    }
    return len;
  }

  int16_t send(uint8_t *buf, uint8_t len)
  {
    // Send char until NULL char
    while (len > 0)
    {
      // Wait end of transmit
      while (!iUart_Uart4GetStatus(kSciTransmitComplete))
        ;

      // Write data
      iUart_Uart4SetData(*buf);

      // Next char
      buf++;
      len--;
    }
    return len;
  }

private:
  uint8_t m_addr;
};

typedef TPixy2<Link2UART> Pixy2UART;

#endif
