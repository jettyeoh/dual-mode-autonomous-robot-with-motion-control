/*
 * motor.c
 *
 *  Created on: Nov 8, 2024
 *      Author: YEOH JETT
 */
#include "motor.h"

extern float distance;

void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

int abs(int p)
{
	if(p > 0)
		return p;
	else
		return -p;
}

void Set_Speed(int left, int right) //-7199 ~ 7199
{
	if(left < 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
	else if(left > 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	}
	htim1.Instance -> CCR4 = abs(left);

	if(right < 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	}
	htim1.Instance -> CCR2 = abs(right);
}

void Control_Motor(int pitch, int roll)
{
	pitch = (pitch > 65) ? 65 : (pitch < -65) ? -65 : (abs(pitch) < 10 ? 0 : pitch);
	roll = (roll > 65) ? 65 : (roll < -65) ? -65 : (abs(roll) < 10 ? 0 : roll);

	if (pitch == 0 && roll == 0) {
	    Set_Speed(0, 0);
	}

	if (abs(pitch) > abs(roll)) {
		if (distance < 30) {
			// Move backward when the distance is smaller than 40 cm
			Set_Speed(100 * abs(pitch), 100 * abs(pitch));
		}
		else{
		    // Control forward-backward
		    Set_Speed(70 * pitch, 50 * pitch);
		}
	}
	else if (abs(roll) > abs(pitch)) {
	    // Control left-right
	    Set_Speed(-50 * roll, 50 * roll);
	}
}

void Distance_Follow(void)
{
	if (distance > 20 && distance < 150)
	{
		Set_Speed(-2000, -2000);
	}
	else if (distance < 15)
	{
		Set_Speed(2000, 2000);
	}
	else
	{
		Set_Speed(0,0);
	}
}
