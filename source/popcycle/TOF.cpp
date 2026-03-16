/*
 * TOF.cpp
 *
 *  Created on: 15 Mar 2026
 *      Author: j6895
 */

#include "algorithm"
extern "C"{
#include <VL53L1X/core/VL53L1X_api.h>
}
#define TOF_ADDR 0x29

const float distanceStop = 200.0f; //200mm
const float distanceSlow = 800.0f; //800mm

void TOF_init(void){
    uint8_t status;

    // Initialize sensor
    status = VL53L1X_SensorInit(TOF_ADDR);

    // Set long distance mode (better for obstacle detection)
    VL53L1X_SetDistanceMode(TOF_ADDR, 2);

    // Measurement timing budget (ms)
    // Lower = faster but less accurate
    VL53L1X_SetTimingBudgetInMs(TOF_ADDR, 50);

    // Inter measurement period (ms)
    VL53L1X_SetInterMeasurementInMs(TOF_ADDR, 50);

    // Start continuous ranging
    VL53L1X_StartRanging(TOF_ADDR);
}

float TOF_update(void)
{
    uint8_t ready;
    uint16_t distance;

    VL53L1X_CheckForDataReady(TOF_ADDR, &ready);

    if(ready)
    {
        VL53L1X_GetDistance(TOF_ADDR, &distance);
        VL53L1X_ClearInterrupt(TOF_ADDR);

        //filter to avoid jitter
        float alpha = 0.3f;
        static float distanceFiltered = alpha * distance + (1.0f-alpha) * distanceFiltered;
        //linear slowdown
        float multiplierTOF = (distanceFiltered - distanceStop) / (distanceSlow - distanceStop);

        multiplierTOF = std::clamp(multiplierTOF,0.0f,1.0f);

        return multiplierTOF;
    }
}

