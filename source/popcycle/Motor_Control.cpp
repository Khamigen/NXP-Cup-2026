/*
 * Motor_Control.c
 *
 *  Created on: 30 Nov 2025
 *      Author: j6895
 */
#include "Motor_Control.h"
#include "fsl_common.h"
extern "C"{
#include "Modules/mTimer.h"
}
const float speedMax = 1.0f;
const float speedMin = -0.6f;
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
    if(speed > 1.0f) {speed = 1.0f;}
    if(speed < -1.0f) {speed = -1.0f;}
    mTimer_SetServoDuty(1, speed);
}



