/*
 * Motor_Control.h
 *
 *  Created on: 30 Nov 2025
 *      Author: j6895
 */

#ifndef POPCYCLE_MOTOR_CONTROL_H_
#define POPCYCLE_MOTOR_CONTROL_H_


void Motor_Init(void);  // init ESC，set max/min speed
void Motor_SetSpeed(float speed); // set speed (-1.0 ~ 1.0)
float Motor_SetSpeedCurve(float steer);

#endif /* POPCYCLE_MOTOR_CONTROL_H_ */
