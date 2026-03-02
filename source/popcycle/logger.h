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
	int vector1X0;
	int vector1X1;
	int vector1Y0;
	int vector1Y1;
	int vector2X0;
	int vector2X1;
	int vector2Y0;
	int vector2Y1;
} DetailedLogger;

#endif /* POPCYCLE_LOGGER_H_ */
