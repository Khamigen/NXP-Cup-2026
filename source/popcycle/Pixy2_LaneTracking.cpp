/*
 * Pixy2_Lane_Tracking.c
 *
 *  Created on: 29 Nov 2025
 *      Author: j6895
 */
#include <Pixy/Pixy2SPI_SS.h>
#include <Popcycle/Pixy2_LaneTracking.h>
#include <stdbool.h>
#include <math.h>
#include <algorithm>
extern "C"{
#include "Modules/mTimer.h"
}

#define MA_WINDOW_SIZE 5 // window used for moving average

//static because these are "state" saved from last loop. shouldn't be reset during each loop.
// moving average
static float errorBuffer[MA_WINDOW_SIZE];
static int bufferIndex = 0;
static int bufferCount = 0;

// Proportional–Derivative Controller
static float lastAvgError = 0.0f;
const float kD = 0.01f;	//derivative, bigger kd, faster steer
const float kP = -0.14f;	//proportion, bigger kp, bigger steer

// Limit maximum steer
const float steerMax = 0.7f;

// Limit steering rate
static float lastSteer = 0.0f;
const float steerStepLimit = 0.5f;

// missed Vector
static int lastLaneCenterX = 39;
//static int lastHadTwoLines = 0;

const int frameCenterX = 39; // Pixy2 line mode width/2
const int laneHalfWidthPx = 25;   //
const int jumpThreshold = 25;     // 若新估跳超過此值則暫不採用
int singleLineStableCount = 0;    // 單線穩定計數器
const int stabilityFrames = 3;    // 要連續多少幀才接受估值

//check whether the 2 vectors from pixy are valid
bool twoVectorsValid(const Vector &v1, const Vector &v2)
{
	float angle1 = atan2f(v1.m_y1 - v1.m_y0, v1.m_x1 - v1.m_x0) * 57.2958f;
	float angle2 = atan2f(v2.m_y1 - v2.m_y0, v2.m_x1 - v2.m_x0) * 57.2958f;
	float angleDiff = fabsf(angle1 - angle2);
	// --- compute center x ---
	int midX1 = (v1.m_x0 + v1.m_x1) / 2;
	int midX2 = (v2.m_x0 + v2.m_x1) / 2;
	//condition 1: angle difference too big
	if (angleDiff > 60.0f)
		{return false;}
	//condition 2: 2 vectors too close to each other
	if (abs(midX1 - midX2) < 40)
		{return false;}
	return true;
}
//lane center calculation for single line case
float singleVectorLogic(Vector &v)
{
	//make sure the vector is pointing upward
	if(v.m_y0 < v.m_y1)
	{
	    std::swap(v.m_x0, v.m_x1);
	    std::swap(v.m_y0, v.m_y1);
	}
	//angle calculation
    float angle = atan2f(v.m_y1 - v.m_y0, v.m_x1 - v.m_x0) * 57.2958f;
    //middle X coordinate
    int midX = (v.m_x0 + v.m_x1) / 2;
    //angle = 0 --> horizontal line
    bool rightTurn = (angle > 20);   // vector points rightward
    bool leftTurn  = (angle < -20);  // vector points leftward

    // case 1: right turn
    if(rightTurn)
        return midX + laneHalfWidthPx;

    // case 2: left turn
    if(leftTurn)
        return midX - laneHalfWidthPx;

    // case 3: angle close to 0, almost horizontal line, use the position of the line to calculate lane center
    // in this scenario, right outer line would be on the right side and vice versa
    if(midX < frameCenterX)
        return midX + laneHalfWidthPx;  // vector located at the right, turn right
    else
        return midX - laneHalfWidthPx;  // vector located at the left, turn left
}

float singleVectorSmooth(Vector &v, int laneCenterEstimate)
{
	// --- compute vector length ---
	float len = hypotf(v.m_x1 - v.m_x0, v.m_y1 - v.m_y0);
	const float minLen = 15.0f;  // minimal vector length to consider, adjust empirically
	const float normalLen = 60.0f; // typical full-length vector in pixels, adjust for your camera
	if(len < minLen)
		// too short, ignore it and return lastLaneCenter
		return lastLaneCenterX;
	// longer vector gets more weight
	float lengthWeight = (len < normalLen) ? (len / normalLen) : 1.0f;
	// calculate the Y coordinate of the vector, upper vector gets more weight
	int midY = (v.m_y0 + v.m_y1) / 2;
	const int frameHeight = 80;  // Pixy2 line-mode height
	float verticalWeight = 1.0f - ((float)midY / (float)frameHeight);  // 0 at bottom, 1 at top
	//combine length and vertical weight
	float combinedWeight = lengthWeight * verticalWeight;
	// --- blend with previous lane center ---
	int blended = lastLaneCenterX + (int)((laneCenterEstimate - lastLaneCenterX) * combinedWeight);
	// --- jump protection ---
	if (abs(blended - lastLaneCenterX) > jumpThreshold)
		blended = lastLaneCenterX + (int)((laneCenterEstimate - lastLaneCenterX) * 0.2f);
	return blended;
}

float Pixy2_LaneTracking(Pixy2SPI_SS &pixy){
	int laneCenterX;
	pixy.line.getAllFeatures(LINE_VECTOR, 1);
	// if detects more than 2 vectors, calculate the center
	if(pixy.line.numVectors >= 2)
	    {
	        // Determine left and right lines
	        auto v1 = pixy.line.vectors[0];
	        auto v2 = pixy.line.vectors[1];
	        //check if the 2 vectors valid
	        //if valid, calculate using 2 vector logic
		    if (twoVectorsValid(v1,v2)){
		    	//2 vector logic
		    	//calculate mid x coordinates
		    	int midX1 = (v1.m_x0 + v1.m_x1) / 2;
		    	int midX2 = (v2.m_x0 + v2.m_x1) / 2;
		    	//compare midX1 and midX2, the smaller one is leftX and the bigger one is rightX
		    	int leftX = (midX1 < midX2) ? midX1 : midX2;
		    	int rightX = (midX1 < midX2) ? midX2 : midX1;

		    	laneCenterX = (leftX + rightX)/2;
		    	lastLaneCenterX = laneCenterX;
		    }
		    //if not valid, fallback to 1 vector logic
		    else{
		    	// use single vector logic
		    	int laneCenterEstimate = singleVectorLogic(v1);
		    	// smooth the vector
		    	lastLaneCenterX = singleVectorSmooth(v1,laneCenterEstimate);
		    	laneCenterX = lastLaneCenterX;
		    }
	    }
	// only detects 1 vector
	else if (pixy.line.numVectors == 1)
	{
	    auto v = pixy.line.vectors[0];
	    // use single vector logic
	    int laneCenterEstimate = singleVectorLogic(v);
	    // smooth the vector
	    lastLaneCenterX = singleVectorSmooth(v,laneCenterEstimate);
	    laneCenterX = lastLaneCenterX;
	}
	//detects no vector
	else{
		// use last lane center
		laneCenterX = lastLaneCenterX;
	}

	// error calculation
	float error = (float)(laneCenterX - frameCenterX);
    // Moving Average
	//add error to the buffer array
    errorBuffer[bufferIndex] = error;
    //bufferIndex point to next element, use modulo to loop to element0 if window size is reached
    bufferIndex = (bufferIndex + 1) % MA_WINDOW_SIZE;

    if(bufferCount < MA_WINDOW_SIZE)
        {bufferCount++;}

    float sum = 0;
    for(int i=0; i<bufferCount; i++)
        {sum += errorBuffer[i];}

    float avgError = sum / bufferCount;

    //PD Controller, kP and kD defined as constant
    float dError = avgError - lastAvgError;
    lastAvgError = avgError;

    float steer = kP * avgError + kD * dError;

    // Limit the maximum range of steer, steerMax defined as constant
    // to do: use clampf instead
    if(steer > steerMax) steer = steerMax;
    if(steer < -steerMax) steer = -steerMax;

    // Limit turn rate. steerStepLimit defined as constant
    float delta = steer - lastSteer;
    if(delta > steerStepLimit)
        {steer = lastSteer + steerStepLimit;}
    else if(delta < -steerStepLimit)
        {steer = lastSteer - steerStepLimit;}

    lastSteer = steer;

    return steer;
}
