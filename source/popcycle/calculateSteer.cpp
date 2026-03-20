/*
 * calculateSteer.cpp
 *
 *  Created on: 22 Feb 2026
 *      Author: brunofigura
 */

#include <PopCycle/calculateSteer.h>
#include <PopCycle/eBuffer.h>

static int bufferCount = 0;
const float steerMax = 0.75f;

static float lastSteer = 0.0f;
const float steerStepLimit = 0.5f;

const float kD = 0.01f;
const float maxKp = -0.06f;
const float minKp = -0.02f;
float kP = -0.05f;

float multiplierPot1 = 1.0f;
float alteredKp = -0.05f;

static float lastAvgError = 0.0f;

float calculateSteer(eBuffer &eb){
	if(bufferCount < MA_WINDOW_SIZE)
	        {bufferCount++;}

	    float sum = 0;
	    for(int i=0; i<bufferCount; i++)
	        {sum += eb.errors[i];}

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

float calculateSteer(eBuffer &eb, float *kPot1){
	if(bufferCount < MA_WINDOW_SIZE)
	        {bufferCount++;}

	    float sum = 0;
	    for(int i=0; i<bufferCount; i++)
	        {sum += eb.errors[i];}

	    float avgError = sum / bufferCount;

	    //PD Controller, kP and kD defined as constant
	    float dError = avgError - lastAvgError;
	    lastAvgError = avgError;

		multiplierPot1 = (*kPot1 + 1.0f) * 0.5f;
		kP = (multiplierPot1 * (maxKp - minKp)) + minKp;


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

