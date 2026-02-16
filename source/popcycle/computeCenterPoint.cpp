/*
 * computeCenterPoint.cpp
 *
 *  Created on: 20 Jan 2026
 *      Author: brunofigura
 */
#include <Popcycle/computeCenterPoint.h>
#include <stdbool.h>
#include <math.h>
#include <algorithm>
#include <Popcycle/lineVectors.h>
#include <Popcycle/centerPoint.h>

#define FRAME_MIDDLE_X 39 //78 Frame-Heigth
#define FRAME_MIDDLE_Y 25 //51 Frame-Width
//Took from old version - why does this exist
#define LANE_HALF_WIDTH_PIXEL 25

static CenterPoint lastCp = {FRAME_MIDDLE_X, FRAME_MIDDLE_Y};


void estimateCenterPointFromSingleVector(CenterPoint &currentCp, LineVectors &lv){
	//make sure the vector is pointing upward
		if(lv.v1.m_y0 > lv.v1.m_y1)
		{
		    std::swap(v.m_x0, v.m_x1);
		    std::swap(v.m_y0, v.m_y1);
		};
	//	int len = root((v.m_y1-v.m_y0)^2+(v.m_x1-v.m_x0)^2);
	//	if (len<)
		//
		// What does this exactly do
		//
		float angle = atan2f(lv.v1.m_y1 - lv.v1.m_y0, lv.v1.m_x1 - lv.v1.m_x0) * 57.2958f; //TODO: replace float with constant or makro
		float slope = atan2f(lv.v1.m_y1 - lv.v1.m_y0, lv.v1.m_x1 - lv.v1.m_x0);
		//check if vector valid. If invalid, return last lane center
		//if(!singleVectorValid(v)){return lastLaneCenterX;}
		//angle calculation

		//TODO: explain why is this necessary after swapping vectors so they always point upwards?
		if ( lv.v1.m_x1 < lv.v1.m_x0)
		{
			angle = -angle;
		}

	    //middle X coordinate
		//TODO: maybe nice to also have y coordinate estimation: mixX and midY for later speed stuff
	    int midX = (lv.v1.m_x0 + lv.v1.m_x1) / 2;
	    //angle = 0 --> horizontal line
	    bool rightTurn = (angle > 25);   // vector points rightward TODO: replace int with constant or makro
	    bool leftTurn  = (angle < -25);  // vector points leftward

	    // case 1: right turn
	    if(rightTurn)
	        currentCp.x = midX - LANE_HALF_WIDTH_PIXEL;

	    // case 2: left turn
	    if(leftTurn)
	        currentCp.x = midX + LANE_HALF_WIDTH_PIXEL;

	    // case 3: angle close to 0, almost horizontal line, use the position of the line to calculate lane center
	    // in this scenario, right outer line would be on the right side and vice versa

	    if(slope < 0){
	        currentCp.x + LANE_HALF_WIDTH_PIXEL;  // vector located at the right, turn right
	    } else {
	        currentCp.x - LANE_HALF_WIDTH_PIXEL;  // vector located at the left, turn left
	    }
};

CenterPoint computeCenterPoint(LineVectors &lv){
	CenterPoint currentCp;
	//Set y frameCenter - TODO: computeLogic not yet implemented
	currentCp.y = FRAME_MIDDLE_Y;

	if (lv.useSingleVectorLogic){
		estimateCenterPointFromSingleVector(currentCp, lv);
	} else { //else do twoVectorLogic
		int midX1 = (v1.m_x0 + v1.m_x1) / 2;
		int midX2 = (v2.m_x0 + v2.m_x1) / 2;
		//compare midX1 and midX2, the smaller one is leftX and the bigger one is rightX
		int leftX = (midX1 < midX2) ? midX1 : midX2;
		int rightX = (midX1 < midX2) ? midX2 : midX1;

		currentCp.x = (leftX + rightX)/2;
		lastCp = currentCp;
	}
};





