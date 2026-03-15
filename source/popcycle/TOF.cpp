/*
 * TOF.cpp
 *
 *  Created on: 15 Mar 2026
 *      Author: j6895
 */

#include <VL53L1X/core/VL53L1X_api.h>
#define TOF_ADDR 0x29

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

void TOF_update()
{
    uint8_t ready;
    uint16_t distance;

    VL53L1X_CheckForDataReady(TOF_ADDR, &ready);

    if(ready)
    {
        VL53L1X_GetDistance(TOF_ADDR, &distance);
        VL53L1X_ClearInterrupt(TOF_ADDR);

        // obstacle logic here
    }
}

