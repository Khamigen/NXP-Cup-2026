/*
 * Motor_Control.h
 *
 *  Created on: 30 Nov 2025
 *      Author: j6895
 */

#ifndef POPCYCLE_MOTOR_CONTROL_H_
#define POPCYCLE_MOTOR_CONTROL_H_


void Motor_Init(void);  // 初始化 ESC，設定最小/最大速度
void Motor_SetSpeed(float speed); // 設定車輪速度 (-1.0 ~ 1.0)

#endif /* POPCYCLE_MOTOR_CONTROL_H_ */
