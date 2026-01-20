/*
 * lineVectors.h
 *
 *  Created on: 20 Jan 2026
 *      Author: brunofigura
 */
#include <Pixy/Pixy2SPI_SS.h>


#ifndef POPCYCLE_LINEVECTORS_H_
#define POPCYCLE_LINEVECTORS_H_


typedef struct {
	bool useSingleVectorLogic;
	bool noValidVectors;
	Vector v1;
	Vector v2;
} LineVectors;


#endif /* POPCYCLE_LINEVECTORS_H_ */
