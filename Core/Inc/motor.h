/*
 * motor.h
 *
 *  Created on: Nov 8, 2024
 *      Author: YEOH JETT
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1;

void Set_Speed(int left, int right);
void Motor_Init(void);
void Control_Motor(int pitch, int roll);
void Distance_Follow(void);


#endif /* INC_MOTOR_H_ */
