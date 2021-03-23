/*
 * H_Bridge_Sinwave.c
 *
 *  Created on: 14.12.2019
 *   Author: Sami
 *   Briefing: This Library contains all developed Algorithms for generation sinewave H-bridge Driver
 */
#include "H_Bridge_Sinwave.h"
#include "stm32f3xx_hal.h"

/*****************Sine wave Generation using 1 Timer (version 1)*****************************************/
/*****Initialisation H Bridge*******/
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // Disable Pin auf 0
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // SO Pin auf 0
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // Dir Pin auf 0
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Hbrucke pwm
/*************Variaben*************/
/*int i;
float t;
float Freq = 10.0f;
float Fpwm = 1280.0f;
float Tab[128]; // Fpwm/Freq
float PI = 3.141f;
uint8_t dir;
/********************User begin 2*********************
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // Disable Pin auf 0
	for(i=0;i<128;i++)
	{
		Tab[i] = sinf((i*2*PI)/128.0f);	// werte in die tabelle speichern
		printf("i=%d\n wert tab%f\n",i,Tab[i]);

	}
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1); // PWM_Hbrucke start mit IT
/*******************Callback Function user begin 4*******************************
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)	//PWM Pulse interrupt callbcak
{
	if (htim == &htim1) {

		static unsigned int i;

		for(i=0;i<128;i++)
		{
			t = TIM1->ARR*Tab[i];
			if (t>0)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); // Pin Dir SET
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, t*0.45);
				printf("t=%f\n",t);

			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // Pin Dir RESET
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -t*0.45);
				printf("t=%f\n",t);

			}
			dir=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);

		}
	}
}

/***************************** Sine wave with 2 Timers (version 2) *********************************************************/

/*Briefing: To get the sinwave do the following configuration in Cube Mx:
 *Timer 1: PSC=0;Counter Mode=UP; ARR=3599; NVIC=Enable;
 *Timer 6: PSC=9;Counter Mode=UP; ARR=1000; NVIC=Enable;//the value need to be proofed
 *This Programm generate a Sinus wave using 2 Timers
 *Timer 1 generate pwm and timer 6 switch the duty cycle and the direction pin for the hbridge
 */
/* USER CODE BEGIN PV */
/*float pi = 3.1416f;
float sinus [128];
float frequenz = 15.615;*/
/* USER CODE END PV */

/* USER CODE BEGIN 1 */
/*for(int i=0; i<128; i++)
{
	sinus [i] = sinf(pi*i/128);
}*/
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
/*HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);
HAL_TIM_Base_Start_IT (&htim6);
__HAL_TIM_SET_AUTORELOAD(&htim6, (uint16_t) (72000000/(2560.f*frequenz)-1));*/ // hier ändert sich die frequenz
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
/*int zaehler = 0;
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *ht)
{
	if (ht == &htim6)
	{
		zaehler++;
		int position = zaehler%128;
		if (position==0)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
		}
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, sinus[position]*3599*0.12);
	}
}*/
/* USER CODE END 4 */
