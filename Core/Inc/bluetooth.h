/*
 * bluetooth.h
 *
 *  Created on: Nov 29, 2024
 *      Author: YEOH JETT
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "stm32f1xx_hal.h"
#include "motor.h"
#include "servo.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;
extern char rxBuffer[50];
extern volatile uint8_t rxIndex;
extern volatile uint8_t messageReady;
extern uint8_t manual;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void ProcessReceivedData(char *data);

#endif /* INC_BLUETOOTH_H_ */
