/*
 * logger.h
 *
 *  Created on: 1 Mar 2026
 *      Author: brunofigura
 */

#ifndef POPCYCLE_LOGGER_H_
#define POPCYCLE_LOGGER_H_

typedef struct {
	bool singleVectorDetected;
	int laneCenterOffsetX;
} SimpleLogger;

typedef struct {
	bool singleVectorDetected;
	int laneCenterOffsetX;
	int laneCenteroffsetY;
	int vector1X;
	int vector1Y;
	int vector2X;
	int vector2Y;
} DetailedLogger;

#endif /* POPCYCLE_LOGGER_H_ */
