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

#define MA_WINDOW_SIZE 5 // window used for moving average

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
const float steerMax = 0.6f;

// Limit steering rate
static float lastSteer = 0.0f;
const float steerStepLimit = 0.05f;

// missed Vector
static int lastLaneCenterX = 157; //middle of video width
static int lastHadTwoLines = 0;

const int ySamplingCoordinate = 120; //can be adjusted
const int brightnessDifferenceThreshhold = 60; //needs to be adjusted
static uint8_t previousBrightness = 255;

float Pixy2_LaneTracking(Pixy2SPI_SS &pixy, float bThresh){
	int xSamplingCoordinate = lastLaneCenterX; //starts in the middle of the frame
	int laneCenterX;
	uint8_t r, g,b;
	uint8_t currentBrightness = 0;
	int rightOuterLinePosition, leftOuterLinePosition;

	//finding right xPos of lane
	while(xSamplingCoordinate < 315){
		pixy.video.getRGB(xSamplingCoordinate,  ySamplingCoordinate, &r, &g, &b);
		currentBrightness = 0.2126 * r + 0.7152 * g + 0.0722 * b;
		if(abs(previousBrightness - currentBrightness) >= bThresh){
			rightOuterLinePosition = xSamplingCoordinate;
			xSamplingCoordinate = 315;
		}
		previousBrightness = currentBrightness;
		xSamplingCoordinate += 4; //jumps for pixels //can be adjusted //maybe define pixelSampleOffset
	}

	xSamplingCoordinate = 157;
	previousBrightness = 255;

	//finding left xPos of lane
	while (xSamplingCoordinate > 0){
		pixy.video.getRGB(xSamplingCoordinate,  ySamplingCoordinate, &r, &g, &b);
		currentBrightness = 0.2126 * r + 0.7152 * g + 0.0722 * b;
		if(abs(previousBrightness - currentBrightness) >= bThresh){
					leftOuterLinePosition = xSamplingCoordinate;
					xSamplingCoordinate = 0;
		}
		previousBrightness = currentBrightness;
		xSamplingCoordinate -= 4;
	}

	//calculating laneCenter based on assumptions that it is exactly between to outer lines
	laneCenterX = rightOuterLinePosition - leftOuterLinePosition;
	// error calculation
	int frameCenterX = 157; // Pixy video mode width / 2
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


