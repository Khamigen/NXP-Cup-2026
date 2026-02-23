/*
 * getLineVectorsFeature.cpp
 *
 *  Created on: 20 Jan 2026
 *      Author: brunofigura
 */

#include <Pixy/Pixy2SPI_SS.h>
#include <Popcycle/getLineVectorsFeature.h>
#include <stdbool.h>


void getLineVectorsFeature(Pixy2SPI_SS &pixy, LineVectors &lv){
	pixy.line.getAllFeatures(LINE_VECTOR, 1);

	if (pixy.line.numVectors >= 2){
		lv.useSingleVectorLogic = false;
		lv.v1 = pixy.line.vectors[0];
		lv.v2 = pixy.line.vectors[1];
	} else if(pixy.line.numVectors == 1) {
		lv.useSingleVectorLogic = true;
		lv.v1 = pixy.line.vectors[0];
	} else {
		lv.noValidVectors = true;
	};
};

