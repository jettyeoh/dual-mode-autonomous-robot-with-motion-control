/*
 * servo.h
 *
 *  Created on: Nov 28, 2024
 *      Author: YEOH JETT
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f1xx_hal.h"
extern TIM_HandleTypeDef htim4;

void Servo_Init(void);
void Set_FrontBack(int angle);
void Set_LeftRight(int angle);
void Set_UpDown(int angle);
void Open_Claw(void);
void Close_Claw(void);

#endif /* INC_SERVO_H_ */
