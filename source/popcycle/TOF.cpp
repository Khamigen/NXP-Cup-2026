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
#define TOF_ADDR 0x52

const float distanceStop = 250.0f; //200mm
const float distanceSlow = 800.0f; //800mm
static float distanceFiltered = 0.0f;

void TOF_init(void){
    uint8_t status;
    uint8_t boot = 0;
    uint8_t state = 0;

    VL53L1_WaitMs(TOF_ADDR, 10);
    do {
        VL53L1X_BootState(TOF_ADDR, &state);
    } while(state == 0);
    // Initialize sensor
    status = VL53L1X_SensorInit(TOF_ADDR);

    // Set long distance mode (better for obstacle detection)
    VL53L1X_SetDistanceMode(TOF_ADDR, 1);

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
    uint8_t ready = 0;
    uint16_t distance = 0;

    static float distanceFiltered = 0.0f;
    static float multiplierFiltered = 1.0f;

    float alpha_d = 0.3f;
    float alpha_m = 0.1f;

    VL53L1X_CheckForDataReady(TOF_ADDR, &ready);

    if (ready)
    {
        VL53L1X_GetDistance(TOF_ADDR, &distance);
        VL53L1X_ClearInterrupt(TOF_ADDR);

        // Distance EMA
        if (distanceFiltered == 0.0f)
            distanceFiltered = distance;
        else
            distanceFiltered = alpha_d * distance + (1.0f - alpha_d) * distanceFiltered;
    }

    // Always compute multiplier
    float multiplierRaw;

    if (distanceFiltered > distanceSlow)
    {
        multiplierRaw = 1.0f;
    }
    else if (distanceFiltered < distanceStop)
    {
        multiplierRaw = 0.0f;
    }
    else
    {
        multiplierRaw = (distanceFiltered - distanceStop) / (distanceSlow - distanceStop);
    }

    // EMA on multiplier
    multiplierFiltered = alpha_m * multiplierRaw + (1.0f - alpha_m) * multiplierFiltered;

    // Clamp
    multiplierFiltered = std::clamp(multiplierFiltered, 0.0f, 1.0f);

    return multiplierFiltered;
}

bool TOF_thresh(void){
		uint8_t ready = 0;
	    uint16_t distance = 0;

	    static float distanceFiltered = 0.0f;

	    float alpha_d = 0.3f;

	    VL53L1X_CheckForDataReady(TOF_ADDR, &ready);

	    if (ready)
	    {
	        VL53L1X_GetDistance(TOF_ADDR, &distance);
	        VL53L1X_ClearInterrupt(TOF_ADDR);

	        // Distance EMA
	        if (distanceFiltered == 0.0f)
	            distanceFiltered = distance;
	        else
	            distanceFiltered = alpha_d * distance + (1.0f - alpha_d) * distanceFiltered;
	    }
	    else{
	    	distanceFiltered = distanceStop +1;
	    }

	    if (distanceFiltered < distanceStop)
	    {
	        return true;
	    }
	    else {
	    	return false;
	    }

}

