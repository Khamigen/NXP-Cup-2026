/*
 * Pixy2_Lane_Tracking.c
 *
 *  Created on: 29 Nov 2025
 *      Author: j6895
 */
#include <Pixy/Pixy2SPI_SS.h>
#include <Popcycle/Pixy2_LaneTracking.h>
extern "C"{
#include "Modules/mTimer.h"
}

#define MA_WINDOW_SIZE 10 // window used for moving average

//static because these are "state" saved from last loop. shouldn't be reset during each loop.
// moving average
static float errorBuffer[MA_WINDOW_SIZE];
static int bufferIndex = 0;
static int bufferCount = 0;

// Proportional–Derivative Controller
static float lastAvgError = 0.0f;
const float kD = 0.006f;	//derivative, bigger kd, faster steer
const float kP = -0.07f;	//proportion, bigger kp, bigger steer

// Limit maximum steer
const float steerMax = 0.7f;

// Limit steering rate
static float lastSteer = 0.0f;
const float steerStepLimit = 0.3f;

// missed Vector
static int lastLaneCenterX = 39;
//static int lastHadTwoLines = 0;

const int frameCenterX = 39; // Pixy2 line mode width/2
const int laneHalfWidthPx = 30;   // 預估半車道寬（可微調）
const int jumpThreshold = 25;     // 若新估跳超過此值則暫不採用
int singleLineStableCount = 0;    // 單線穩定計數器
const int stabilityFrames = 3;    // 要連續多少幀才接受估值

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

	        // calculate the middle x coordinate with the start and end point x coordinates
	        int mid1 = (v1.m_x0 + v1.m_x1)/2;
	        int mid2 = (v2.m_x0 + v2.m_x1)/2;



	        //compare mid1 and mid2, the smaller one is leftX and the bigger one is rightX
	        int leftX = (mid1 < mid2) ? mid1 : mid2;
	        int rightX = (mid1 < mid2) ? mid2 : mid1;

	        laneCenterX = (leftX + rightX)/2;
	        lastLaneCenterX = laneCenterX;
	    }
	// only detects 1 vector
	else if (pixy.line.numVectors == 1)
	{
	    // 只有一條線：做投影向內側
	    auto v = pixy.line.vectors[0];
	    int mid = (v.m_x0 + v.m_x1) / 2;

	    int laneCenterEstimate;
	    if (mid < frameCenterX)
	    {
	        // 看到的是左線 -> lane center 在右邊
	        laneCenterEstimate = mid + laneHalfWidthPx;
	    }
	    else
	    {
	        // 看到的是右線 -> lane center 在左邊
	        laneCenterEstimate = mid - laneHalfWidthPx;
	    }

	    // 若估值大幅跳動，暫不直接採用，使用緩和或等待穩定
	    if (abs(laneCenterEstimate - lastLaneCenterX) > jumpThreshold)
	    {
	        // 不立即採用，增加穩定計數或採用緩和移動
	        singleLineStableCount = 0;
	        // 輕微緩和（向 estimate 前進 20%）
	        int blended = lastLaneCenterX + (int)((laneCenterEstimate - lastLaneCenterX) * 0.2f);
	        lastLaneCenterX = blended;
	    }
	    else
	    {
	        // 若估值接近歷史，則累積穩定次數，達到門檻再正式接受
	        singleLineStableCount++;
	        if (singleLineStableCount >= stabilityFrames)
	        {
	            lastLaneCenterX = laneCenterEstimate;
	            singleLineStableCount = stabilityFrames; // cap
	        }
	        else
	        {
	            // 暫時緩和移動一點
	            int blended = lastLaneCenterX + (int)((laneCenterEstimate - lastLaneCenterX) * 0.4f);
	            lastLaneCenterX = blended;

	        }

	    }
	    laneCenterX = lastLaneCenterX;
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


