/*
 * linefollow.h
 *
 *  Created on: Nov 29, 2024
 *      Author: YEOH JETT
 */

#ifndef INC_LINEFOLLOW_H_
#define INC_LINEFOLLOW_H_

#include "stm32f1xx_hal.h"
#include "motor.h"

// Define macros for sensor states
#define BLACK 1
#define WHITE 0

// Sensor GPIO pins (adjust as needed)
#define SENSOR_LEFT 			GPIO_PIN_5
#define SENSOR_RIGHT 			GPIO_PIN_4
#define SENSOR_LEFT_JUNCTION 	GPIO_PIN_3
#define SENSOR_RIGHT_JUNCTION 	GPIO_PIN_15 //GPIOA

// Function prototypes
void LineFollowing_Update(void);

#endif /* INC_LINEFOLLOW_H_ */
