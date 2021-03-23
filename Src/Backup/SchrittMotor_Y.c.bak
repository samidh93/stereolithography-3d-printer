/*
 * SchrittMotor_Y.c
 *
 *  Created on: 10.12.2019
 *      Author: Sami
 */

#include "SchrittMotor_Y.h"
/* Schrittmotor Y_achse */

void Stepper_Y_Achse(void){

	// Initialisierung Y_achse

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);				// Reset (inverted) PIN
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);				// Sleep (inverted) PIN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); 			// Enable (inverted) PIN

	// Rotation Schleife Y_achse
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,SET);   			//  Direction Pin (+sense rotation)

	for(int i=0; i<200; i++)				// 200schritte = 360°
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);	// STEP PIN High
		HAL_Delay(1); //1ms
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	// STEP PIN Low
		HAL_Delay(1);
	}
	HAL_Delay(2000);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,RESET);			//  Direction Pin (-sense rotation)

	for(int j=0; j < 200; j++)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_Delay(1);
	}
	HAL_Delay(2000);
}
