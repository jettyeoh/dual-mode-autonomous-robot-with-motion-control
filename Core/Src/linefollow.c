/*
 * linefollow.c
 *
 *  Created on: Nov 29, 2024
 *      Author: YEOH JETT
 */
#include "linefollow.h"

//flag to determine whether the robot is turning or line following
uint8_t left_turn = 0;
uint8_t right_turn = 0;
uint8_t follow_line = 1;
uint8_t sequence = 0;

uint8_t left_sensor;
uint8_t right_sensor;
uint8_t left_junction;
uint8_t right_junction;

void Follow_Line(void);
void Turn_Right_Sequence(void);
void Turn_Left_Sequence(void);

void LineFollowing_Update(void) {
	// Read sensor states
	left_sensor = HAL_GPIO_ReadPin(GPIOB, SENSOR_LEFT);
	right_sensor = HAL_GPIO_ReadPin(GPIOB, SENSOR_RIGHT);
	left_junction = HAL_GPIO_ReadPin(GPIOB, SENSOR_LEFT_JUNCTION);
	right_junction = HAL_GPIO_ReadPin(GPIOA, SENSOR_RIGHT_JUNCTION);

	//priority logic, decide the robot follow line or not
	if(left_junction == BLACK && right_junction == BLACK){
		//stop
		Set_Speed(0,0);
		follow_line = 0;
		return;
	}
	else if(left_junction == WHITE && right_junction == WHITE){
		//start line following
		follow_line = 1;
	}


	if(follow_line == 1){
		//decide turn left, right or follow line
		if(left_junction == BLACK && right_junction == WHITE && right_turn == 0){
			//turn left
			left_turn = 1;
		}
		else if(left_junction == WHITE && right_junction == BLACK && left_turn == 0){
			//turn right
			right_turn = 1;
		}

		//turn left and turn right action
		if(left_turn == 1){
			Turn_Left_Sequence();
		}
		else if(right_turn == 1){
			Turn_Right_Sequence();
		}
		else{
			Follow_Line();
		}
	}


}
//left black then right black
void Turn_Left_Sequence(void){
	//left turn sequence
	if(sequence == 0){
		if(left_sensor == WHITE){sequence = 1;}
		Set_Speed(-2000, -2000);
	}
	else if(sequence == 1){
		if(right_sensor == BLACK){sequence = 0; left_turn = 0;}
		Set_Speed(0, -1700);
	}
}

void Turn_Right_Sequence(void){
	//right turn sequence
	if(sequence == 0){
		if(right_sensor == WHITE){sequence = 1;}
		Set_Speed(-2000, -2000);
	}
	else if(sequence == 1){
		if(left_sensor == BLACK){sequence = 0; right_turn = 0;}
		//turn right
		Set_Speed(-2000, 2000);
	}
}

void Follow_Line(void){
	if (left_sensor == WHITE && right_sensor == WHITE) {
		// Both sensors on the line
		Set_Speed(-2000, -2000);
	} else if (left_sensor == BLACK && right_sensor == WHITE) {
		// Turn left when left sensor detects black
		Set_Speed(0, -3000);
	} else if (left_sensor == WHITE && right_sensor == BLACK) {
		// Turn right when right sensor detects black
		Set_Speed(-2000, 0);
	} else {
		// Default forward motion
		Set_Speed(0, -2000);
	}
}
