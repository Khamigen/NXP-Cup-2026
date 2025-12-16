/*
 * `Pixy2_Lane_Tracking.h
 *
 *  Created on: 30 Nov 2025
 *      Author: j6895
 */
#include <Pixy/Pixy2SPI_SS.h>

#ifndef POPCYCLE__PIXY2_LANETRACKING_H_
#define POPCYCLE__PIXY2_LANETRACKING_H_

#define MA_WINDOW_SIZE 3 // window used for moving average

// PD Controller
extern const float Kd;	//derivative, bigger kd, faster steer
extern const float Kp;	//proportion, bigger kp, bigger steer

// steer limit
extern const float steerMax;
extern const float steeringStepLimit;

typedef struct pixyLineVector {
    uint8_t m_x0;
    uint8_t m_y0;
    uint8_t m_x1;
    uint8_t m_y1;
} pixyLineVector;

float Pixy2_LaneTracking(Pixy2SPI_SS &pixy);

float Pixy2_LaneTrackingDebug(Pixy2SPI_SS &pixy, pixyLineVector (&vectorData)[2]);
#endif /* POPCYCLE__PIXY2_LANETRACKING_H_ */
