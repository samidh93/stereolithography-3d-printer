/*
 * Temperatur_Regelung.c
 * Created on: 07.12.2019
 * Author: Sami
 */

#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "IIR_1_Order.h"
#include "Temperatur_Regelung.h"

double tempReading;
float tempF;
double tempK;
float tempC;
void Temperatur_Polymer_Regler(ADC_HandleTypeDef *hadc4, TIM_HandleTypeDef *htim15)
{

	HAL_ADC_PollForConversion(hadc4, 1); // poll for conversion
	tempReading = HAL_ADC_GetValue(hadc4); // get the adc value
	tempK = log(10000.0 * ((4096.0 / tempReading - 1))); //Specification of thermistor Modell: see source
	tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK ))* tempK );
	tempC = tempK - 273.15;
	tempF = (tempC * 9.0)/ 5.0 + 32.0;
	//printf("\nTemperature=%.2f\n",Filter_IIR_1_Order(tempC));	//Konsole Ausgabe filtriert
	//Temperature_print(); //LCD Ausgabe
	if (tempC>50 && tempC<60){
		HAL_TIM_PWM_Start(htim15, TIM_CHANNEL_1); //PWM für Heizung Start
		__HAL_TIM_SET_COMPARE(htim15, TIM_CHANNEL_1, 100); // 50% duty cycle, langsam heizen
	}
	if (tempC>20 && tempC<50){
		HAL_TIM_PWM_Start(htim15, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(htim15, TIM_CHANNEL_1, 200); // 100% duty cycle, schnell Heizen
	}

	if (tempC>=60)
	{
		__HAL_TIM_SET_COMPARE(htim15, TIM_CHANNEL_1, 0); // 0% duty cycle, 0 spannung
		HAL_TIM_PWM_Stop(htim15, TIM_CHANNEL_1);
	}

}

/**********implementation in main*************************/
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *ht)
{
	if (ht == &htim7) {
		Temperatur_Polymer_Regler(&hadc4, &htim15);
	}
}
 */
/************Source****************/
/*for this ntc modell is the conversion formula token from arduino startet set linked below on page 104/164
 * https://github.com/kali93/SLA_3d_printer/blob/master/Elegoo%20Super%20Starter%20Kit%20for%20UNO%20V1.0.18.12.24-Deutsch.pdf
 */
