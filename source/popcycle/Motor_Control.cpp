/*
 * Motor_Control.c
 *
 *  Created on: 30 Nov 2025
 *      Author: j6895
 */
#include <Popcycle/Motor_Control.h>
#include "fsl_common.h"
#include "math.h"
#include "algorithm"
#include <Popcycle/speedParam.h>
extern "C"{
#include "config.h"
#include "Modules/mTimer.h"
}
//constants for speed contorl
//static const float speedMax = 0.0f;
//static const float speedMin = -0.6f;
//static const float speedCruise = -0.32f;
//static const float speedTurn = -0.4f;
static const float kCurve = 0.8f;// relation between steer and speed, bigger kurve -> slower when steering.

//EMA smoothing
static const float alpha = 0.2f;// used in EMA, bigger alpha->faster changes, smaller alpha->slower but smoother changes
static float speedEMA = speedMax;// used as result of current EMA calculation and buffer from last EMA

static float multiplierPot2;

void Motor_Init(void)
{
    mTimer_SetServoDuty(SERVO_MOTOR, speedMax);   // Max
    SDK_DelayAtLeastUs(15000000, SystemCoreClock); // 15,000,000 us = 15s?
    mTimer_SetServoDuty(SERVO_MOTOR, speedMin); // Min
    SDK_DelayAtLeastUs(15000000, SystemCoreClock);
}

void Motor_SetSpeed(float speed)
{
    speed = std::clamp(speed, speedMin, speedMax);
    mTimer_SetServoDuty(SERVO_MOTOR, speed);
}

float Motor_SetSpeedCurve(float steer, float *Pot2)
{
	/*
	//determine the target speed will steering, bigger steer -> slower target speed
	float speedTarget = speedMax - kCurve * fabsf(steer);
	speedTarget = std::clamp(speedTarget, speedMin, speedMax);

	//Exponential Moving Average, smooth out the change of speed so it don't accel/break instnatly
	speedEMA = alpha * speedTarget + (1.0f - alpha) * speedEMA;
	speedEMA = std::clamp(speedEMA, speedMin, speedMax);
	//get speed from MotorB
	//float rpmL = 0.0f, rpmR = 0.0f;
	//mTimer_GetSpeed(&rpmL, &rpmR);
	*/
	float s = fabsf(steer) / 0.75f;  // 0..1
	s = std::clamp(s, 0.0f, 1.0f);

	multiplierPot2 = (*Pot2 + 1.0f) * 0.5f;
	speedCruiseFinal = (multiplierPot2 * (speedCruise - speedTurn)) + speedTurn;	//pot2 as speedCruise gain

	float speedTarget = speedCruiseFinal + (speedTurn - speedCruiseFinal) * s;

	speedEMA = alpha * speedTarget + (1.0f - alpha) * speedEMA;
	speedEMA = std::clamp(speedEMA, speedTurn, speedCruise);

	return speedEMA;
}

