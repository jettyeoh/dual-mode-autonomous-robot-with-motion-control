/*
 * servo.c
 *
 *  Created on: Nov 28, 2024
 *      Author: YEOH JETT
 */
#include "servo.h"

//htim4.Instance->CCR1 : left right
//htim4.Instance->CCR2 : forward backward
//htim4.Instance->CCR3 : open close
//htim4.Instance->CCR4 : up down

#define ARM_LeftRight        htim4.Instance->CCR1
#define ARM_FrontBack        htim4.Instance->CCR2
#define ARM_Claw			 htim4.Instance->CCR3
#define ARM_UpDown			 htim4.Instance->CCR4

void Servo_Init(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

//max(forward): 1900
//min(backward): 600
//default: 1000
void Set_FrontBack(int angle)
{
	if(angle > 69){angle = 69;}
	else if(angle < -45){angle = -45;}
	ARM_FrontBack = 1000 + (angle * 13);
}

//max(left): 2400
//min(right): 1200
//default: 1800
void Set_LeftRight(int angle)
{
	if(angle > 70){angle = 70;}
	else if(angle < -70){angle = -70;}
	ARM_LeftRight = 1800 + (angle * 5);
}

//max(down): 1500
//min(up): 2500
//default: 2000
void Set_UpDown(int angle)
{
	if(angle > 65){angle = 65;}
	else if(angle < -65){angle = -65;}
	ARM_UpDown = 2000 + (angle * 7);
}

//max(open): 1500
void Open_Claw(void)
{
	ARM_Claw = 1500;
}

//min(close): 500
void Close_Claw(void)
{
	ARM_Claw = 500;
}
