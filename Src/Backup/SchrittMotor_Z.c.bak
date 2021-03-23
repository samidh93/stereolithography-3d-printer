/*
 * SchrittMotor_Z.c
 *
 *  Created on: 10.12.2019
 *      Author: Sami
 */
#include "stm32f3xx_hal.h"
#include "SchrittMotor_Z.h"

/* Schrittmotor Z_achse */

void Stepper_Z_Achse(void){

	// Initialisierung Z_achse

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);			// Reset (inverted) PIN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);			// Sleep (inverted) PIN
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); 		// Enable (inverted) PIN

	// Rotation Schleife Z_achse
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);   			//  Direction Pin (+sense rotation)

	for(int i=0; i<200; i++)				// 200 schritte = 360°
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	// STEP PIN High : HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		HAL_Delay(1); //1ms
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	// STEP PIN Low
		HAL_Delay(1);
	}
	HAL_Delay(2000);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);			//  Direction Pin (-sense rotation)

	for(int j=0; j<200; j++)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // kann auch mit Toggle pin: HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(1);
	}
	HAL_Delay(2000);
}
