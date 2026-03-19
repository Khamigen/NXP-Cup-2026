/*
 * getLineVectorsFeature.h
 *
 *  Created on: 20 Jan 2026
 *      Author: brunofigura
 */
#include <Pixy/Pixy2SPI_SS.h>
#include <stdbool.h>


#include<Popcycle/lineVectors.h>


#ifndef POPCYCLE_GETLINEVECTORSFEATURE_H_
#define POPCYCLE_GETLINEVECTORSFEATURE_H_



void getLineVectorsFeature(Pixy2SPI_SS &pixy, LineVectors &lv, bool *finishDetected);


#endif /* POPCYCLE_GETLINEVECTORSFEATURE_H_ */
