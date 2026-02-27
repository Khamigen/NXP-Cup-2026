/*
 * speedControl.cpp
 *
 *  Created on: 26 Feb 2026
 *      Author: j6895
 */
#include <Popcycle/Motor_Control.h>
#include "Modules/mTimer.h"
#include "algorithm"
#include "math.h"
#include <Popcycle/speedParam.h>

//static const float speedMax = 0.0f;
//static const float speedMin = -0.6f;
//static const float speedCruise = -0.32f;
//static const float speedTurn = -0.4f;
//
////placeholders before rpm is measured from hallsensor
//static const float rpmMax = 0;
//static const float rpmMin = 0;
//static const float rpmCruise = 0;
//static const float rpmTurn = 0;

static const float kP = 0.0004f;
static const float kI = 0.00015f;
static float iTerm = 0.0f;
static float rpmFromSteer (float steer)
{
	float s = fabsf(steer) / 0.75f;  // 0..1
	s = std::clamp(s, 0.0f, 1.0f);
	return rpmCruise + (rpmTurn - rpmCruise) * s;
}

static float speedFromRpm (float rpm)
{
	float nenner = (rpmCruise - rpmTurn);
	if (fabsf(nenner) <= 1e-6f)
	return speedMin;

    float t = (rpm - rpmTurn) / nenner;   // 0..1
    t = std::clamp(t, 0.0f, 1.0f);

    // map to servo command range [speedTurn .. speedCruise]
    return speedTurn + t * (speedCruise - speedTurn);
}

static float UpdateSpeedCurve (float steer, float dt, float rpmCurrent)
{
	    // --- 2) Compute target rpm from steer ---
	    float rpm = rpmFromSteer(steer);

	    // --- 3) Convert rpmRef -> baseline servo command ---
	    float speedBase = speedFromRpm(rpm);

	    // --- 4) PI controller on rpm error ---
	    float error = rpm - rpmCurrent;
	    iTerm += error * dt;

	    float delta = kP * error + kI * iTerm;

	    float speedCmd = speedBase + delta;

	    // --- 5) Clamp within driving range ---
	    float speedClamp = std::clamp(speedCmd, speedMin, speedMax);

	    // Anti-windup
	    if (speedCmd != speedClamp)
	        iTerm *= 0.9f;

	    return speedClamp;
}





