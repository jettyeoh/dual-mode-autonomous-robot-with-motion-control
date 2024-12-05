/*
 * bluetooth.c
 *
 *  Created on: Nov 29, 2024
 *      Author: YEOH JETT
 */
#include "bluetooth.h"

int pitch, roll, yaw;
uint8_t open = 0;
uint8_t arm = 0;
uint8_t following = 0;

//handle received data
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2  && manual == 1) {
        char receivedChar = rxBuffer[rxIndex]; // Store the received character

        if (receivedChar == '\n') {
            // Message delimiter detected, null-terminate the string
            rxBuffer[rxIndex] = '\0';
            messageReady = 1; // Set flag to process the message
            rxIndex = 0;      // Reset buffer index for the next message
        }
        else {
            // Add character to the buffer
            rxIndex++;

            // Handle buffer overflow
            if (rxIndex >= 50) {
                rxIndex = 0; // Reset the index to prevent overflow
            }
        }

        // Re-enable interrupt to receive the next character
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rxBuffer[rxIndex], 1);
    }
}

//arm control, claw control, motor control, distance follow
void ProcessReceivedData(char *data) {
    if ((sscanf(data, "%d %d %d", &pitch, &roll, &yaw) == 3) && (following == 0)){
    	if(arm == 1){
        	Set_FrontBack(-pitch);
        	Set_LeftRight(yaw);
        	Set_UpDown(roll);
    	}
    	else if(arm == 0){
    		Control_Motor(pitch, roll);
    	}
    }
    else if(strcmp(data, "a") == 0){
    	open = !open;
    }
    else if(strcmp(data, "b") == 0){
    	arm = !arm;
    }
    else if(strcmp(data, "c") == 0){
    	following = !following;
    }

    if(open == 1){
    	Open_Claw();
    }
    else{
    	Close_Claw();
    }

    if(following == 1){
    	Distance_Follow();
    }
}
