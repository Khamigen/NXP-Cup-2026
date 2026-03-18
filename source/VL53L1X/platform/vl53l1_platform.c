/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include <Interfaces/iI2C.h>
#include <Modules/mDelay.h>
#include "MK64F12.h"

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	//uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
    while(iI2C_ReadStatus(kBUSY));

    iI2C_TxRxSelect(kTxMode);
    I2C0->S = I2C_S_IICIF_MASK;//clear flag
    iI2C_SetStartState();

    iI2C_SendData((dev) | 0);
    iI2C_WaitEndOfRxOrTx();

    iI2C_SendData(index >> 8);
    iI2C_WaitEndOfRxOrTx();

    iI2C_SendData(index & 0xFF);
    iI2C_WaitEndOfRxOrTx();

    for(uint32_t i = 0; i < count; i++)
    {
        iI2C_SendData(pdata[i]);
        iI2C_WaitEndOfRxOrTx();
    }

    iI2C_SetStopState();

    return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	//uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
    while(iI2C_ReadStatus(kBUSY));

    /* write index */
    iI2C_TxRxSelect(kTxMode);
    iI2C_SetStartState();

    iI2C_SendData((dev) | 0);
    iI2C_WaitEndOfRxOrTx();

    iI2C_SendData(index >> 8);
    iI2C_WaitEndOfRxOrTx();

    iI2C_SendData(index & 0xFF);
    iI2C_WaitEndOfRxOrTx();

    /* repeated start */
    iI2C_SetRepeatedStartSate();

    iI2C_SendData((dev) | 1);
    iI2C_WaitEndOfRxOrTx();

    iI2C_TxRxSelect(kRxMode);

    /* dummy read */
    volatile uint8_t dummy = iI2C_ReadData();

    for(uint32_t i = 0; i < count; i++)
    {
        if(i == count - 1)
        {
            iI2C_SetAckMode(kNoAck);
            iI2C_SetStopState();
        }

        iI2C_WaitEndOfRxOrTx();
        pdata[i] = iI2C_ReadData();
    }

    return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	//uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
    while(iI2C_ReadStatus(kBUSY));

    iI2C_TxRxSelect(kTxMode);
    I2C0->S = I2C_S_IICIF_MASK;//clear flag
    iI2C_SetStartState();

    iI2C_SendData((dev) | 0);
    iI2C_WaitEndOfRxOrTx();

    iI2C_SendData(index >> 8);
    iI2C_WaitEndOfRxOrTx();

    iI2C_SendData(index & 0xFF);
    iI2C_WaitEndOfRxOrTx();

    iI2C_SendData(data);
    iI2C_WaitEndOfRxOrTx();

    iI2C_SetStopState();

    return 0;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	//uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	VL53L1_WrByte(dev, index, (data >> 8) & 0xFF);
	VL53L1_WrByte(dev, index + 1, data & 0xFF);
	return 0;

}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	//uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	VL53L1_WrByte(dev, index, (data >> 24) & 0xFF);
	VL53L1_WrByte(dev, index + 1, (data >> 16) & 0xFF);
	VL53L1_WrByte(dev, index + 2, (data >> 8) & 0xFF);
	VL53L1_WrByte(dev, index + 3, data & 0xFF);

	return 0;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	//uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
    while (I2C0->S & I2C_S_BUSY_MASK);

    I2C0->S = I2C_S_IICIF_MASK;

    /* TX mode */
    iI2C_TxRxSelect(kTxMode);
    iI2C_SetStartState();

    iI2C_SendData(dev | 0);
    iI2C_WaitEndOfRxOrTx();

    iI2C_SendData(index >> 8);
    iI2C_WaitEndOfRxOrTx();

    iI2C_SendData(index & 0xFF);
    iI2C_WaitEndOfRxOrTx();

    /* repeated start */
    iI2C_SetRepeatedStartSate();

    iI2C_SendData(dev | 1);
    iI2C_WaitEndOfRxOrTx();

    /* RX mode */
    iI2C_TxRxSelect(kRxMode);

    /* dummy read */
    (void)I2C0->D;

    /* NACK */
    iI2C_SetAckMode(kNoAck);

    /* wait for data ready */
    while (!(I2C0->S & I2C_S_IICIF_MASK));

    /* stop */
    iI2C_SetStopState();

    /* clear flag */
    I2C0->S |= I2C_S_IICIF_MASK;

    /* read data） */
    *data = I2C0->D;

    return 0;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	//uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
	uint8_t msb, lsb;

	VL53L1_RdByte(dev, index, &msb);
    VL53L1_RdByte(dev, index + 1, &lsb);

    *data = (msb << 8) | lsb;

    return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	//uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	
    uint8_t b0,b1,b2,b3;

    VL53L1_RdByte(dev, index, &b0);
    VL53L1_RdByte(dev, index+1, &b1);
    VL53L1_RdByte(dev, index+2, &b2);
    VL53L1_RdByte(dev, index+3, &b3);

    *data = (b0<<24) | (b1<<16) | (b2<<8) | b3;

    return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	//uint8_t status = 255;
	
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	usleep(wait_ms*1000);

	return 0;
}
