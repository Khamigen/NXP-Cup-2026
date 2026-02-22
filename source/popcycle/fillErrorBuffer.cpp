/*
 * fillErrorBuffer.cpp
 *
 *  Created on: 22 Feb 2026
 *      Author: brunofigura
 */

#include <Popcycle/fillErrorBuffer.h>

static int bufferIndex = 0;

void fillErrorBuffer(float horizontalError, eBuffer &eb){

	eb.errors[bufferIndex] = horizontalError;
	    //bufferIndex point to next element, use modulo to loop to element0 if window size is reached
	bufferIndex = (bufferIndex + 1) % MA_WINDOW_SIZE;
}

