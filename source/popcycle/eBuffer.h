/*
 * errorBuffer.h
 *
 *  Created on: 22 Feb 2026
 *      Author: brunofigura
 */

#ifndef POPCYCLE_EBUFFER_H_
#define POPCYCLE_EBUFFER_H_

#define MA_WINDOW_SIZE 2

typedef struct{
	float errors[MA_WINDOW_SIZE];
} eBuffer;


#endif /* POPCYCLE_ERRORBUFFER_H_ */
