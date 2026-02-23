/*
 * preprocessingLineVectors.cpp
 *
 *  Created on: 20 Jan 2026
 *      Author: brunofigura
 */
#include <stdbool.h>
#include <math.h>
#include <popcycle/preprocessingLineVectors.h>
#include <algorithm>

#include <Popcycle/lineVectors.h>


const float RAD_TO_DEG_CONSTANT = 57.2958;
const float MIN_DEG_DIFF = 50.0;
const int MIN_MIDDLE_DISTANCE = 40;

bool dualVectorValid(LineVectors &lv)
{
	// atan2 calculates angle to x axis when given a 2d point
	float angle1 = atan2f(lv.v1.m_y1 - lv.v1.m_y0, lv.v1.m_x1 - lv.v1.m_x0) * RAD_TO_DEG_CONSTANT;
	float angle2 = atan2f(lv.v2.m_y1 - lv.v2.m_y0, lv.v2.m_x1 - lv.v2.m_x0) * RAD_TO_DEG_CONSTANT;
	float angleDiff = fabsf(angle1 - angle2);
	// --- compute center x ---
	int midX1 = (lv.v1.m_x0 + lv.v1.m_x1) / 2;
	int midX2 = (lv.v2.m_x0 + lv.v2.m_x1) / 2;
	//condition 1: angle difference too big
	if (angleDiff > MIN_DEG_DIFF)//TODO: replace float with constant or makro
		{return false;}
	//condition 2: 2 vectors too close to each other
	if (abs(midX1 - midX2) < MIN_MIDDLE_DISTANCE)//TODO: replace float with constnat or makro
		{return false;}
	return true;
}

bool singleVectorValid(LineVectors &lv){
	//placeholder till more perfomative algo is being found
	return true;
};

void processSingleVector(LineVectors &lv){
	//TODO: not implemented
	return;
};

void preprocessingLineVectors(LineVectors &lv, bool forceSingleVectorLogic){
	if (forceSingleVectorLogic){
		lv.useSingleVectorLogic = true;
	};

	if (lv.useSingleVectorLogic == false){
		if(dualVectorValid(lv)){
			return;
		}else{
			lv.useSingleVectorLogic = true;
		};
	};
	//
	//TODO: can we assume to always use first vector when going singleVectorLogic
	//
	if ( lv.useSingleVectorLogic == true){
		//problem: singleVectorValid huge perfomance problem
		if(singleVectorValid(lv)){
			processSingleVector(lv);
		}else{
			lv.noValidVectors = true;
			return;
		};
	};



};




