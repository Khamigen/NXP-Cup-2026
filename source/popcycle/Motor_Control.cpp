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
extern "C"{
#include "Modules/mTimer.h"
}
//constants for speed contorl
static const float speedMax = 0.0f;
static const float speedMin = -0.6f;
static const float speedCruise = -0.32f;
static const float speedTurn = -0.4f;
static const float kCurve = 0.8f;// relation between steer and speed, bigger kurve -> slower when steering.

//EMA smoothing
static const float alpha = 0.2f;// used in EMA, bigger alpha->faster changes, smaller alpha->slower but smoother changes
static float speedEMA = speedMax;// used as result of current EMA calculation and buffer from last EMA
void Motor_Init(void)
{
    // 範例: 設定最大 / 最小速度
    mTimer_SetServoDuty(1, speedMax);   // Max
    SDK_DelayAtLeastUs(15000000, SystemCoreClock); // 15,000,000 us = 15s?

    mTimer_SetServoDuty(1, speedMin); // Min
    SDK_DelayAtLeastUs(15000000, SystemCoreClock);
}

void Motor_SetSpeed(float speed)
{
    speed = std::clamp(speed, speedMin, speedMax);
    mTimer_SetServoDuty(1, speed);
}

void Motor_SetSpeedCurve(float steer)
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

	float speedTarget = speedCruise + (speedTurn - speedCruise) * s;

	speedEMA = alpha * speedTarget + (1.0f - alpha) * speedEMA;
	speedEMA = std::clamp(speedEMA, speedTurn, speedCruise);

	Motor_SetSpeed(speedEMA);

}

