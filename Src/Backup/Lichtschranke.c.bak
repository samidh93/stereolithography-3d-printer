/*
 * Lichtschranke.c
 *  Created on: 11.12.2019
 *  Author: Sami
 *  Briefing: This Library contains all the developed Algorithms during the Test of "Lichtschranke"
 */
#include "stm32f3xx_hal.h"
#include "Lichtschranke.h"
/*
 * /******************** Lichtschranke version 1 *******************************************
 *
/*void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if interrput source is channel 1
	{
		if (Is_First_Captured==0)  // is the first value captured ?
		{
			IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture the first value
			Is_First_Captured =1;  // set the first value captured as true
		}

		else if (Is_First_Captured==1)  // if the first is captured
		{
			IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture second value

			if (IC_Value2 > IC_Value1)
			{
				Period = 2*((IC_Value2-IC_Value1)*(0.00001));   // calculate the Period
			}

			else if (IC_Value2 < IC_Value1)
			{
				Period = 2*((((0xffff-IC_Value1)+IC_Value2) +1)*(0.00001));
			}

			else
			{
				Error_Handler();
			}
			Frequency = (1/Period);
			//Frequency = HAL_RCC_GetPCLK2Freq()/(Period*htim4.Init.Prescaler);  // calculate frequency
			Is_First_Captured = 0;  // reset the first captured
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if interrput source is channel 1
	{
		if (Is_First_Captured_2==0)  // is the first value captured ?
		{
			IC_Value1_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // capture the first value
			Is_First_Captured_2 =1;  // set the first value captured as true
		}

		else if (Is_First_Captured_2==1)  // if the first is captured
		{
			IC_Value2_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // capture second value

			if (IC_Value2_2 > IC_Value1_2)
			{
				Period_2 = 2*((IC_Value2_2-IC_Value1_2)*(0.00001));   // calculate the Period
			}

			else if (IC_Value2_2 < IC_Value1_2)
			{
				Period_2 = 2*((((0xffff-IC_Value1_2)+IC_Value2_2) +1)*(0.00001));
			}

			else
			{
				Error_Handler();
			}
			Frequency_2 = (1/Period_2);
			//Frequency = HAL_RCC_GetPCLK2Freq()/(Period*htim4.Init.Prescaler);  // calculate frequency
			Is_First_Captured_2 = 0;  // reset the first captured

		}
	}
	if (IC_Value2 > IC_Value1 && IC_Value2_2 > IC_Value1_2)
	{
		Pulswidth = 2*(IC_Value2_2 - IC_Value2)*(0.00001);
		Duty_Cycle = (Pulswidth/((Period+Period_2)*0.5))*100;
	}


/******************** Lichtschranke version 2 *******************************************/
/* VaRIABLEN
 * int t_1 = 0;
float_t t_3 = 0;
float_t Period = 0;
float_t Frequency = 0;
float_t Is_First_Captured = 0;  // 0- not captured, 1- captured

float_t t_2 = 0;
float_t t_4 = 0;
int t_5 = 0;

float_t Period_2 = 0;
float_t Frequency_2 = 0;

float_t Is_First_Captured_2 = 0;  // 0- not captured, 1- captured

float_t Is_Second_Captured = 0;

float_t Pulswidth = 0;
float_t Duty_Cycle = 0;
 */

/*void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if interrput source is channel 1
	{
		if (Is_First_Captured==0 && Is_Second_Captured==0)  // is the first value captured ?
		{
			t_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture the first value
			Is_First_Captured =1;  // set the first value captured as true
		}

		else if
		(Is_First_Captured==1 && Is_Second_Captured==0)// if the first is capture
		{

			t_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture second value
			Is_Second_Captured = 1; // set the second value captured as true
			Is_First_Captured = 0;
		}
		else
			//(Is_First_Captured==0 && Is_Second_Captured==1)// if the second is captured
			//if (Is_Second_Captured==1)// if the second is captured
		{
			t_5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture third value
			Is_Second_Captured = 0;
		}

		if (t_5 > t_1)
		{
			Period = ((t_5 - t_1)*(0.00001));   // calculate the Period
		}
		Frequency = 1/(Period);

	}
}
 */
/******************** Lichtschranke version 3 *******************************************/
/*
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)		//Input Capture interrupt für lichtschranke
{
	if(htim==&htim4)											//wenn der timer 4 ist dann
	{
		/*************************Rising_edge_channel**************************************

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)		// if interrput source is channel 1
		{
			switch(zustand)								// Capture Zustande für Rising Edge
			{
			case 0:										//first Capture rising
				__HAL_TIM_SET_COUNTER(htim,0);			//stell counter auf 0 und zahle hoch
				zustand=1;								//übergang zum nachsten zustand
				break;
			case 1:
				t_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);	//second capture rising
				zustand =2;									//transisition auf zustand 2
				break;

			case 2:											// case third capture rising edge
				t_4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);		//read third capture
				__HAL_TIM_SET_COUNTER(htim,0);								//counter wieder auf 0 setzen und zyklus neubeginnne
				zustand=1;													//
				break;
			}
		}
		/*************************Falling_edge_channel**************************************

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)// if interrput source is channel 2
		{
			switch(zustand)								// Capture Zustande für falling Edge
			{
			case 1:										//first Capture falling
				t_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);		//read first capture fallng

				break;
			case 2:											//second Capture falling
				t_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	//read second capture
				break;
			}
		}
	}
}
*/
