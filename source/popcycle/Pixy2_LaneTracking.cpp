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
const int laneHalfWidthPx = 25;   // 預估半車道寬（可微調）
const int jumpThreshold = 25;     // 若新估跳超過此值則暫不採用
int singleLineStableCount = 0;    // 單線穩定計數器
const int stabilityFrames = 3;    // 要連續多少幀才接受估值

bool twoLinesValid;

float singleLineCenterX(const Vector &v)
{
    float angle = atan2f(v.m_y1 - v.m_y0, v.m_x1 - v.m_x0) * 57.2958f;

    int mid = (v.m_x0 + v.m_x1) / 2;

    bool rightTurn = (angle > 20);   // 線偏向右 → 車道右彎
    bool leftTurn  = (angle < -20);  // 線偏向左 → 車道左彎

    // case 1: 線偏右 → 右彎 → 假設看到的是外側左線
    if(rightTurn)
        return mid + laneHalfWidthPx;

    // case 2: 線偏左 → 左彎 → 假設看到的是外側右線
    if(leftTurn)
        return mid - laneHalfWidthPx;

    // case 3: angle 接近 0 → 直線 → 回到原本邏輯
    if(mid < frameCenterX)
        return mid + laneHalfWidthPx;  // 左線 → 中心
    else
        return mid - laneHalfWidthPx;  // 右線 → 中心
}

float Pixy2_LaneTracking(Pixy2SPI_SS &pixy){
	int laneCenterX;
	pixy.line.getAllFeatures(LINE_VECTOR, 1);
	// if detects more than 2 vectors, calculate the center
	if(pixy.line.numVectors >= 2)
	    {
			//lastHadTwoLines = 1;
	        // Determine left and right lines
	        auto v1 = pixy.line.vectors[0];
	        auto v2 = pixy.line.vectors[1];
	        // --- compute angle ---
	        float angle1 = atan2f(v1.m_y1 - v1.m_y0, v1.m_x1 - v1.m_x0) * 57.2958f;
	        float angle2 = atan2f(v2.m_y1 - v2.m_y0, v2.m_x1 - v2.m_x0) * 57.2958f;
	        float angleDiff = fabsf(angle1 - angle2);
	        // --- compute length ---
	       	float len1 = hypotf(v1.m_x1 - v1.m_x0, v1.m_y1 - v1.m_y0);
	       	float len2 = hypotf(v2.m_x1 - v2.m_x0, v2.m_y1 - v2.m_y0);
	        //float ratio = (len1 < len2) ? (len1 / len2) : (len2 / len1);
	        // --- compute center x ---
	        int mid1 = (v1.m_x0 + v1.m_x1) / 2;
	        int mid2 = (v2.m_x0 + v2.m_x1) / 2;
	        if (angleDiff > 60.0f)
	            {twoLinesValid = false;}
//	        else if (ratio < 0.35f)
//	            {twoLinesValid = false;}
	        else if (abs(mid1 - mid2) < 40)
	        	{twoLinesValid = false;}
	        else
	        	{twoLinesValid = true;}
		    if (twoLinesValid){
	        //compare mid1 and mid2, the smaller one is leftX and the bigger one is rightX
		    	int leftX = (mid1 < mid2) ? mid1 : mid2;
		    	int rightX = (mid1 < mid2) ? mid2 : mid1;

		    	laneCenterX = (leftX + rightX)/2;
		    	lastLaneCenterX = laneCenterX;
		    }
		    else{

			    int mid = (v1.m_x0 + v1.m_x1) / 2;
			    // --- compute vector length ---
			    float len = hypotf(v1.m_x1 - v1.m_x0, v1.m_y1 - v1.m_y0);
			    const float minLen = 15.0f;  // minimal vector length to consider, adjust empirically
			    const float normalLen = 60.0f; // typical full-length vector in pixels, adjust for your camera

			    if (len < minLen)
			    {
			        // vector too short → ignore
			    	laneCenterX = lastLaneCenterX;
			    }
			    else
			    {
			    	int laneCenterEstimate = singleLineCenterX(v1);

			        // --- weight by vector length ---
			        float lengthWeight = (len < normalLen) ? (len / normalLen) : 1.0f;

			        // --- vertical position factor ---
			        float vecMidY = (v1.m_y0 + v1  .m_y1) / 2.0f;
			        const float frameHeight = 80.0f;  // Pixy2 line-mode height
			        float verticalFactor = 1.0f - (vecMidY / frameHeight);  // 0 at bottom, 1 at top

			        float combinedWeight = lengthWeight * verticalFactor;
			        // --- blend with previous lane center ---
			        int blended = lastLaneCenterX + (int)((laneCenterEstimate - lastLaneCenterX) * combinedWeight);
			        // --- jump protection ---
			        if (abs(blended - lastLaneCenterX) > jumpThreshold)
			        	blended = lastLaneCenterX + (int)((laneCenterEstimate - lastLaneCenterX) * 0.2f);

			    	lastLaneCenterX = blended;
			    	laneCenterX = lastLaneCenterX;
			    }
		    }
	    }
	// only detects 1 vector
	else if (pixy.line.numVectors == 1)
	{
	    // 只有一條線：做投影向內側
	    auto v = pixy.line.vectors[0];
	    int mid = (v.m_x0 + v.m_x1) / 2;

	    			    // --- compute vector length ---
	    			    float len = hypotf(v.m_x1 - v.m_x0, v.m_y1 - v.m_y0);
	    			    const float minLen = 15.0f;  // minimal vector length to consider, adjust empirically
	    			    const float normalLen = 60.0f; // typical full-length vector in pixels, adjust for your camera

	    			    if (len < minLen)
	    			    {
	    			        // vector too short → ignore
	    			    	laneCenterX = lastLaneCenterX;
	    			    }
	    			    else
	    			    {
	    			    	int laneCenterEstimate = singleLineCenterX(v);
	    			        // --- weight by vector length ---
	    			        float lengthWeight = (len < normalLen) ? (len / normalLen) : 1.0f;

	    			        // --- vertical position factor ---
	    			        float vecMidY = (v.m_y0 + v.m_y1) / 2.0f;
	    			        const float frameHeight = 80.0f;  // Pixy2 line-mode height
	    			        float verticalFactor = 1.0f - (vecMidY / frameHeight);  // 0 at bottom, 1 at top

	    			        float combinedWeight = lengthWeight * verticalFactor;
	    			        // --- blend with previous lane center ---
	    			        int blended = lastLaneCenterX + (int)((laneCenterEstimate - lastLaneCenterX) * combinedWeight);
	    			        // --- jump protection ---
	    			        if (abs(blended - lastLaneCenterX) > jumpThreshold)
	    			        	blended = lastLaneCenterX + (int)((laneCenterEstimate - lastLaneCenterX) * 0.2f);

	    			    	lastLaneCenterX = blended;
	    			    	laneCenterX = lastLaneCenterX;
	    			    }

	}
	else{

		laneCenterX = lastLaneCenterX;
		//lastHadTwoLines = 0;
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
