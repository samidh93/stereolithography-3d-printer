/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @Project		   : SLA_3d_Printer
 * @Author		   : Sami Dhiab
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "stdlib.h"
#include "math.h"						//Include math bibliothek
#include "SchrittMotor_Y.h"				//Bibliothek der treiber Y_achse
#include "SchrittMotor_Z.h"				//Bibliothek der treiber Z_achse
#include "print_scan_usart2.h"			//Bibliothek zur ausgabe von werte durch printf via uart
#include "i2c-lcd.h"					//Dispaly Bibliothek
#include "IIR_1_Order.h"				//IIR filetr bibliothek
//#include "Delay_Mikro_Sekunde.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { Start, Init, Drucken, Ende} States_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc4;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/******************************************************************
 * *************************User_Variablen*************************
 * ****************************************************************/

/*******************************Variablen Sinus Tabelle*************************/
float pi = 3.1416f;						// Variable pi
float sinus [128];						// Array für Sinustabelle
float ZeitProSchwingung;				//zeit für 1 schwingung
float remainTime;						//verbleibende zeit
/**************************Zeit variablen Lichtschranke***************************/
volatile float t_1 = 0;						// Zeitpunkt erste fallende Flanke
volatile float t_2 = 0;						// Zeitpunkt zweite steigende Flanke
volatile float t_3 = 0;						// Zeitpunkt zweite fallende Flanke
volatile float t_4 = 0;						// Zeitpunkt dritte steigende Flanke
volatile float hell_1 = 0;					// bereich in dem wir belichten 1/3 der zeit(_1 bedeutet fur halbe schwingung nach rechts)
volatile float hell_2 = 0;					// bereich in dem wir belichten 1/3 der zeit(_2 bedeutet fur halbe schwingung nach links)
volatile float dunkel_1 = 0;					// bereich in dem wir nicht belichten(_1 bedeutet fur halbe schwingung nach rechts)
volatile float dunkel_2 = 0;					// bereich in dem wir nicht belichten(_2 bedeutet fur halbe schwingung nach links)
volatile float dunkel_1_90P = 0;				// Dunkel_1 bereich mit offset
volatile float dunkel_2_90P = 0;				// Dunkel_2 bereich mit offset

/*********************Variablen Amplitude Regler**************************/
volatile float frequenz_soll = 8.34f;				// Sollfrequenz  --> muss für bbeide LAser jeweils angepasst werden
volatile float frequenz_gemessen = 0;				// gemessene Frequenz
volatile float kdc = 0.25f;					// Vorfaktor für duty cycle in Sinustabelle
volatile float Toleranz_Regler = 0.5f;				// toleranz faktor pi regler

volatile float e = 0; 						// aktuelle Regelabweichung pro Iteration
volatile float esum = 0; 					// Aufsummierung Regelabweicuhung
volatile float kp = 0.8; 					// P-Anteil des Reglers
volatile float ki = 0.06f;					// I-Anteil des Reglers
volatile float kdc_max = 0.25f;					// Maximaler Vorfaktor von 0,25 (25%)
volatile float kdc_min = 0.1;					//Minimaler kdc-Wert des Programms
volatile float faktor_psc = 327272.72f;				// Faktor für psc zum Bestimmen der gemessenen Frequenz
volatile float verhaeltnis_soll= 0.6f;				// Fuhrungsgroße
volatile float verhaeltnis;  					// Verhaltnis Hell zu Dunkel

/******************************Zustände und Flags***********************************************/
volatile int zustand = 0;					// Zustands variable für case-Funktion der Lichtschranke
volatile int flag_schr_z =0;					// Für was sind diese flags?
volatile _Bool flag_schr_y = 0;					// flag für interrupt um die y achse zu bewegen
volatile _Bool ref_y = 0;					// Refernz der y-Achse
volatile _Bool ref_z = 0;					// Refernz der z-Achse
volatile _Bool ist_geschwungen = 0;				// Flag für wie viele schwingung sind gemacht
volatile int Anzahl_Schwingung = 0;				// Zähler fur anzahl der schwingung, parameter für regeleung
volatile int Anzahl_Ebenen = 0;
volatile _Bool Doorstate =0;					// Endschalter an Türe geschlossen?
volatile _Bool Start_Taster_gedruckt = 0;			// Damit Taster nicht zufällig Wert 1 enthält
//volatile _Bool Init = 0;
//volatile _Bool Start = 0;
//volatile _Bool Drucken = 0;
//volatile _Bool Ende = 0;
volatile _Bool Schalter_geschlossen = 0;


/***************************Zähler variablen****************************************************/
volatile int32_t zaehler_schr_y = 0;			//zähler für schritt der y achse
volatile int32_t zaehler = 0;				// Variable für das Abschreiben der Sinustabelle
volatile int32_t zaehler_X = 0;				//zahler für 250 abtasten der x achse  / Ich glaub das sind jetzt 280 oder so?
volatile int32_t zaehler_Y = 0;				//zahler für 250 abtasten der y achse
volatile int32_t Ebenerunter = 0;


/**************************Variablen kreis Berechnung****************************************/
volatile float xm = 124.5; 					// Mittelpunkt x
volatile float ym = 124.5;					// Mittelpunkt y
volatile float dx; 						// aktueller x-Achsen Abstand von aktuellem Punkt zu Mittelpunkt
volatile float dy;						// aktueller y-Achsen Abstand von aktuellem Punkt zu Mittelpunkt
volatile float dxy;						// Abstand yum Mittelpunkt (Ist dannn der spätere Betrag aus a² und b²)
float X_Achse[250];						// Array für x_Achse mit 250 Abfragen
//uint8_t ZweiD_Array[250][250] = {0};				// test array 50*50 (nicht verwendet)


/**********************Temperature Variablen*****************************************/
double tempReading;
float tempF;
double tempK;
float tempC;
_Bool Polymerready = 0;
uint32_t adc_buf[1];
double temp;
/************************Entprellen***************************************************/
_Bool ledState = SET;
_Bool buttonState;
_Bool lastButtonState = RESET;
int lastDebounceTime = 0;
int debounceDelay = 50;
int reading;
/***********************Extra**********************************************************/
float inkrement = (120/280);
float bogen_120 = 2.094395102;


/*
//IMPERIAL_MARCH
int	C8	=	17199	;
int	B7	=	18222	;
int	AB7	=	19306	;
int	A7	=	20454	;
int	GA7	=	21670	;
int	G7	=	22958	;
int	FG7	=	24324	;
int	F7	=	25770	;
int	E7	=	27303	;
int	DE7	=	28926	;
int	D7	=	30646	;
int	CD7	=	32469	;
int	C7	=	34399	;
int	B6	=	36445	;
int	AB6	=	38612	;
int	A6	=	40908	;
int	GA6	=	43341	;
int	G6	=	45918	;
int	FG6	=	48648	;
int	F6	=	51541	;
int	E6	=	54606	;
int	DE6	=	57853	;
int	D6	=	61293	;
int	CD6	=	64938	;
 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM17_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */
//static void state_machine(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/******************************************************************
 * *************************User_Funktionen_Definition*************************
 * ****************************************************************/


/*************************Delay in mikrosekunde***********************************/

void delay_us(uint16_t us)						//delay in µs
{
	__HAL_TIM_SET_COUNTER(&htim8,0);				//Timer8; Counter auf 0 setzen
	while (__HAL_TIM_GET_COUNTER(&htim8) < us);			//wenn das counter wert kleiner als gewunschte delay
}

/**********************Sinus Erzeugung*********************************************/
void generate_sinus()
{
	for(int i=0; i<128; i++)		//sinus tabelle auffulen für 128 werte
	{
		sinus [i] = sinf(pi*i/128);
	}
}

/******************2 dimensional Array für den Kreis***********************************/
/*void Kreis_berechnen()
{
	static uint8_t i;

	for(i=0; i<250; i++)					//for schleife der berechnung kreis punkte
	{
		dx = (i-xm);							//hier abs für postive werte
		dy = (zaehler_250_Y-ym);
		dxy = sqrtf((dx*dx)+(dy*dy));
		if ((dxy <= 120)&&(dxy >= 115))
		{
			X_Achse [i] = 1;

		}
		else
		{
			X_Achse [i] = 0;
		}
	}
}*/
/*************************Referenzfahrt y_achse********************************/

void home_y_achse()
{
	// Initialisierung Y_achse
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);			// Reset (inverted) PIN
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);			// Sleep (inverted) PIN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); 			// Enable (inverted) PIN

	// Referenzier Y-Achse
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);			// Direction Pin Reset (-sense rotation)
	do									// Mache Anweisung, während Pin Endschalter (PC5) gleich 1 ist (da Öffner)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);		// STEP PIN High
		delay_us(700);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);		// STEP PIN Low
		delay_us(700);
	}
	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == 1); 			// wenn gpio pin ist pin endlage y schalter betatigt



	// Dann fahre wieder von Endschalter weg zu Startposition für Druck

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,SET);				// Dazu wird Direction Pin umgedreht (auf set)

	for(int i=0; i<400; i++)						// Rotation Schleife Y_achse 500 Schritte = 10 mm
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);		// STEP PIN High
		delay_us(700);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);		// STEP PIN Low
		delay_us(700);
	}
	ref_y=1;								// Variable stellt dar, dass Referenzfahrt abgeschlossen wurde
}



/*************************Referenzfahrt_Z_achse********************************/


void home_z_achse()
{

	HAL_TIM_Base_Start(&htim8);					//Delay in mikrosekunde timer start

	// Initialisierung z-Achse

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);			// Reset (inverted) PIN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);			// Sleep (inverted) PIN
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); 			// Enable (inverted) PIN


	// Rotation Schleife: z-Achse ganz unten und dann bis end schalter

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);   		//  Direction Pin (+sense rotation) nach oben
	do									// do this while condition ist true
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);		// STEP PIN High
		delay_us(700);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);		// STEP PIN Low
		delay_us(700);
	}
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0); 			// wenn gpio pin ist pin endlage z schalter betätigt (Endschalter als Schließer ausgeführt, da Öffnerkontakt kaputt)

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);			//  Direction Pin set (-sense rotation) nach unten

	for(int i=0; i<1500; i++)						// Rotation Schleife Z_achse 20 Schritte sind 0,1 mm
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);		// STEP PIN High
		delay_us(700);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);		// STEP PIN Low
		delay_us(700);
	}
	ref_z=1;								// Referenzfaht
}


/*************************************Schrittmotor_Y_Achse_Bewegen_schritte******************************/

//Funktion zum Bewegen der y-Achse während Druckvorgang:


void Schrittmotor_y_achse_bewegen()
//Ablauf: Wenn Flag gesetzt und bereits über 20 Schwingungen erledigt, dann verfahren y-Achse um 20 Schritte
{
	if ((flag_schr_y == 1) && (Anzahl_Schwingung >= 20))			// wenn flag schrittmotor y-Achse = 1 und Schwinger ist breits 20 mal geschwungen
	{
		zaehler_schr_y++;						// Zählt Koordinate der y-Achse

		for(int i=0; i<20; i++)						// 20 Schritte fahren
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);	// STEP PIN High
			delay_us(1000);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);	// STEP PIN Low
			delay_us(1000);
		}

		//Funktion: Wenn y-Koordinate den Wert 250 erreicht hat, soll die Drehrichtung des Schrittmotors umgedreht werden
		int position_schr_y = zaehler_schr_y%250;			// Man teilt das aktuelle Feld immer durch 250 (zb. 50 durch 250; 100 durch 250;..). Man macht das so lange, bis der Rest 0 ist, also wenn 250 geteilt durch 250 gleich 1 ergibt und kein Rest übrig bleibt
		if (position_schr_y==0)						// Wenn y-Koordinate = 250, wird die Drehrichtung des Motors der y-Achse umgedreht (Falsch zuvor: DIR-Pin an H_Brücke wird getogglet)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
		}
		flag_schr_y = 0;						// Flag wird wieder auf 0 gesetzt, dadurch kann sich y-Motor erstmal nicht bewegen
	}
}
/*************************************Schrittmotor_Z_Achse_Ebene_Sinken******************************/
//Funktion zum Bewegen der Z-Achse während Druckvorgang:
int Z_Achse_ebene_sinken()
{
	if (flag_schr_z == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);		//  Direction Pin set (-sense rotation) nach unten

		for(int i=0; i<500; i++)					//Rotation Schleife Z_achse 20schritte 0,1 mm (Fahr 500 Schritte nach unten)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	// STEP PIN High
			delay_us(700);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	// STEP PIN Low
			delay_us(700);
		}
		//Druckbett fährt hier also 500 Schritte runter in das Polymer


		// Fahr Druckbett wieder hoch auf Druckposition
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_RESET);		// Drehrichtung wieder umkehren (nach oben)
		for(int i=0; i<480; i++)					// Fährt zuvor 500 Schritte runter und jetzt wieder 480 Schritte nach oben
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	// STEP PIN High
			delay_us(700);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	// STEP PIN Low
			delay_us(700);
		}
		//Warte 2 sekunden und lass Polymer abtropfen
		HAL_Delay(2000);								// Delay_z: zeit für jeder ebene + delay für die fahrt der schritte

		++Anzahl_Ebenen;						// Zähl die z-Koordinate um +1

		flag_schr_z = 0;
	}

	return Anzahl_Ebenen;
}




/***********************Lichtschranke_timer_initialise****************************************/
void Lichtschranke_timer_start()
{
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);	// input capture start mit interrupt fur die lichtschranke ch1 direct mode rising edge
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);	// input capture start mit interrupt fur die lichtschranke ch2 indirect mode falling edge
	// Input Capture:
}


/***********************Laser_change_state_Function**********************************************/
volatile void Laser_switch_state(_Bool laserstate)		//Funktion ändert Pin von Laser (je nach Feld)
{

	if (laserstate == 1)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);// Laser an

	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);// Laser aus
	}

}

/*******************************x-Achse_start_function********************************************/
// Beginne Schwingung der x-Achse aus Ruhelage

void schwinger_start()
{
	// Start H-Brücke
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); 	// Disable Pin H-brücke auf 0 setzen
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);		// Timer für PWM Erzeugung von 20 kHz (ne glaub jz anderen Wert)
	// 20.000 mal pro Sekunde (bei jeder PWM) passt die H-Brücke ihren DC an den vorgegebenen Wert der 128 Sinusse an)
	// Bei jedem eintreffenden PWM-Signal gibt die H-Brücke den entsprechenden Sinus-Wert weiter

	HAL_TIM_Base_Start_IT (&htim6);				//Timer interrupt um den duty cycle der pwm zu ändern
	generate_sinus();				// Funktion um die werte der sinus zu generieren
	__HAL_TIM_SET_AUTORELOAD(&htim6, (uint16_t) (72000000/(2560.f*frequenz_soll)-1)); //setzten des CCR-Werts für Sinus-Frequenz

}
/*******************************X_Achse_stop_function********************************************/


void schwinger_stop()
{
	// Stop H-Brücke
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); 	// Disable Pin H-brücke auf 1 setzen
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);		// Timer für PWM Erzeugung von 20 kHz stoppen
	HAL_TIM_Base_Stop (&htim6);				// Timer interrupt um die duty cycle des pwm zu ändern
	//generate_sinus();					// Funktion um die werte der sinus zu generieren
	//__HAL_TIM_SET_AUTORELOAD(&htim6, (uint16_t) (72000000/(2560.f*frequenz_soll)-1)); //setzten des CCR-Werts für Sinus-Frequenz
}


/********************************Amplitude_Regulation_Function**************************************/

void Amplitude_regler()
{
	 // Reglerstart wenn Frequenzband erreicht
	if ((frequenz_gemessen >= (frequenz_soll - Toleranz_Regler)) && ((frequenz_gemessen<=frequenz_soll + Toleranz_Regler)))
	{
		e = (verhaeltnis_soll - verhaeltnis);					// Regelabweichung zwischen Soll- und Ist-Wert
		esum = esum + e;										// Rechnen der Regelabweichung -> Summe der fehler
		kdc = kp*(e+ki*esum);									//berechne kdc anhand kp und ki

		if (kdc > kdc_max)										//wenn kdc großer als es max sein soll
		{
			kdc = kdc_max;										//kdc nimmt das maximum erlaubt
			esum = esum - e;									// der fehler ist dann negativ
		}
		if (kdc < kdc_min)										//wenn kdc ist kleiner als sein minimum wert
		{
			kdc = kdc_min;										//kdc nimmt die kleinste werte
			esum = esum + e;									//der fehler ist dann positiv
		}
	}
}


/***********************Adjust_resonance_frequency_at Begin**************************************/
/*Briefing: use *Potentiometer to adjust the resonance at the beginning
 * ADC Values could varies from 0 to 4095. As factor is the Log of the value given + 1.0f to avoid log infinity
 * 0,00008 is chosen based on estimation that from 0 to 4095 will be approximately distributed like [8..9] graduation
 * it means when the *Poti turned to max it reaches 9 and then decrease by turning in other side until it reaches 8hz.
 */
volatile float adjust_frequency(ADC_HandleTypeDef *hadc2, float *frequenz_soll)
{
	volatile float Frequenz_Akt = *frequenz_soll;   	// Aktuelle Frequenz soll Frequenz_Soll werden anfangs mit 8.0f

	HAL_ADC_PollForConversion(hadc2, 1); // poll for conversion: Hol ADC-Wert von Potti
	// get the adc value	// Erzeuge aus dem ADC-Wert einen Faktor
	volatile float Factor = logf(HAL_ADC_GetValue(hadc2)+1.0f)*0.00008f;

	if ((Frequenz_Akt>8.0) && (Frequenz_Akt<9.0))//Wenn aktuelle Frequenz im bereich 8 und 9,ikrement um factor umdrehung
	{
		Frequenz_Akt = Frequenz_Akt + Factor;// dann Frequenz_Akt = Frequenz_Akt + Factor;
	}

	else if (Frequenz_Akt>9.0)	//Wenn aktuelle Frequenz größer als 9.0 Hz, dann Frequenz_Akt = Frequenz_Akt - Factor;
	{
		Frequenz_Akt = Frequenz_Akt - Factor;
	}
	else if (Frequenz_Akt<8.0)	//Wenn aktuelle Frequenz kleiner als  als 8.0 Hz
	{
		Frequenz_Akt = *frequenz_soll;
	}

	*frequenz_soll = Frequenz_Akt;
	return *frequenz_soll;
}
//Abruf function: adjust_frequency(&hadc2, &frequenz_soll);

/***********************Calculate_Remaining_Time**************************************/
//function to calculate the remaining time and show on display
float CalculateRemainTime()
{
	//periode der schwingung in s mal 1000 ist in ms
	float ZeitProSchwingung = 1000/frequenz_soll;
	float Delay_z = 1372; //2000 + 980*1.4f:delay timefor z ahcse einstellung pro ebene:
	//1.4ms pro schritt mal 980 schritte + 2000ms delay fur polymer abtropfen
	float Delay_y = 40;	// delay time for y: 20schritte and 2ms pro schritt
	int soll_Anzahl_Ebenen = 50;	//50 ebene geplante
	int soll_Anzahl_zeilen = 250;	// 250 zeilen pro ebene
	int soll_Anzahl_Schwingung = 12520;
	//20 + 250*50: 20 Einschwingungen,250 schwingung pro ebene * anzahl geplante ebenen
	float soll_druckzeit = soll_Anzahl_Schwingung*ZeitProSchwingung+Delay_y*soll_Anzahl_zeilen+Delay_z*soll_Anzahl_Ebenen;
	//gesamte zeit zum drucken
	float ist_druckzeit =Anzahl_Schwingung*ZeitProSchwingung+Delay_y*zaehler_Y+Delay_z*Anzahl_Ebenen;
	//zeit vergangen beim druck
	remainTime = (soll_druckzeit - ist_druckzeit)/60000;
	//verbleibende zeit aktualisiert durch 1000 in s durch 60 in minuten
	float remainTime_prozent = (remainTime/soll_druckzeit)*100;
	return remainTime;
}
//this function can be replaced by using timer that counts at the beginnig and calculate the numbers of overflow

/*********************LED*************************************/
void LED_Anzeige_an()							//LED USer interface
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
}


_Bool Doorgetstate()				// Hol Wert von PB11 (Endschalter Türe) und schreibe den Wert 0 oder 1 in die Variable Doorstate
{
	Doorstate = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
	return Doorstate;			// Funktion gibt mir Variable Doorstate zurück
}

_Bool Start_Taster_getstate()			// gleiches wie bei Endschalter
{
	Start_Taster_gedruckt = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	return Start_Taster_gedruckt;
}



/***********************Init_z_achse******************************/
void init_Z_achse()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);			// Reset (inverted) PIN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);			// Sleep (inverted) PIN
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); 			// Enable (inverted) PIN
}


/****************************Init_function***********************************************/
void Init_Drucker()
{
	/******************************************************************
	 * *************************User_Funktionen_starten*************************
	 * ****************************************************************/

	/************* Heizung und Temperature Messung start****************************/
	//HAL_ADC_Start(&hadc4); 					// start ADC for temperature reading
	HAL_ADC_Start_DMA (&hadc4, &adc_buf[1], 1);	//start adc 4 in dma modus
	HAL_TIM_Base_Start_IT(&htim7);  			// start Timer for ADC mit interrupt jeder 10 sekunden
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); 		// PWM für Heizung Start

	/**************Display start***************************************************/
	lcd_init();								// initialise display
	lcd_clear();								// clear display



	/****************Lichtschranke start********************************************/
	Lichtschranke_timer_start();


	/****************LaserTimer start************************************************/
	// Ist auskommentiert
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);		//pwm start für den laser (moglicherweise auf one pulse)



	/***************Delay in mikrosekunde timer start********************************/
	HAL_TIM_Base_Start(&htim8);					//Delay in mikrosekunde timer start


	/**************Y_Z_Achse-referenzfahrt****************************************************/
	home_y_achse();
	init_Z_achse();

	/***************H-Brücke Sinus Schwingung start********************************************/
	// Start H-Brücke
	schwinger_start();
	//
}
/*****************************Music Function***************************************/
void playmusic()
{

	__HAL_TIM_SET_AUTORELOAD(&htim1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 3599*kdc_max);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 40908);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40908*kdc_max);
	HAL_Delay(800);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 40908);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40908*kdc_max);
	HAL_Delay(800);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 40908);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40908*kdc_max);
	HAL_Delay(800);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 51541);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 51541*kdc_max);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 34399);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 34399*kdc_max);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 40908);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40908*kdc_max);
	HAL_Delay(800);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 51541);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 51541*kdc_max);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 34399);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 34399*kdc_max);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 40908);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40908*kdc_max);
	HAL_Delay(1600);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);



	__HAL_TIM_SET_AUTORELOAD(&htim1, 27303);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 27303*kdc_max);
	HAL_Delay(800);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 27303);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 27303*kdc_max);
	HAL_Delay(800);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 27303);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 27303*kdc_max);
	HAL_Delay(800);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 25770);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 25770*kdc_max);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 34399);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 34399*kdc_max);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 43341);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 43341*kdc_max);
	HAL_Delay(800);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 51541);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 51541*kdc_max);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 34399);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 34399*kdc_max);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim1, 40908);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 40908*kdc_max);
	HAL_Delay(1600);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(10);


	__HAL_TIM_SET_AUTORELOAD(&htim1, 3599);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 3599*kdc_max);

}

/********************Z_Achse ebene runter um Polymer zu fluten************************/

void Zebene_runter()   // z-Achse soll eine Schicht nach unten
{
	if ((Ebenerunter==1)&&(ref_z==1))
	{

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,GPIO_PIN_SET);			// Direction Pin set (-sense rotation) unten

		for(int i=0; i<20; i++)							// Rotation Schleife Z_achse 20 Schritte 0,1 mm
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);		// STEP PIN High
			delay_us(700);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);		// STEP PIN Low
			delay_us(700);
		}

		Ebenerunter =0;
	}
}
/*********************************Debounce Buttoon*************************************/
/*Briefing: this function make a software button debounce
 * @Source : https://www.arduino.cc/en/tutorial/debounce
 */
void DebounceButton(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	reading = HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);			//read the input signal
	if (reading != lastButtonState) {					// wenn das zustand ungliech die letzte zustand
		lastDebounceTime = HAL_GetTick();				// letzte entprellenzeit nimmt Tick
	}
	if ((HAL_GetTick() - lastDebounceTime) > debounceDelay) {	//wenn tick - last >delay
		if (reading != buttonState) 		//fllas abweichend
		{
			buttonState = reading;		//state gleich read
		}
	}
	lastButtonState = reading;			//laststate = neustate
}
//Abruf function: DebounceButton(GPIOA,GPIO_PIN_5);
/************************************PLL Function************************************/
/*
//Vorgaben und Messwerte:
float phi_soll;		//Phasenverschiebung soll
float phi_ist;		//Phasenevrschiebung ist
float t_phi;  		//Phasenevrschiebung Zeit t


//Zeitvariablen
float t1;			//Zeit dunkel
float t0;			//Startzeit
float t21n;			//zeitpunkt T2 - T1
float t1n;			//Zeitpunkt dunkel
float t2n;			//Zeitpunkt hell
float t3n;			//Zeitpunkt dunkel
float t4n;			//Zeitpunkt hell
float t43n;			//Zeitpunkt T4-T3

//alte Zeiten
float t4n_1;		//Wert T4 aus vorheriger Periode
float t3n_1;		//Wert T3 aus vorheriger Periode
float Periodendauer;// Periodendauer

//Vergangenheitsform

//Berechnung t21 und t43
t21 = (((t2 - t1) /2) + t1);
t43 = (((t4 - t3) /2) + t3);

t_phi = (((t4n - t3n)/2) + t3n) - (((t4n-1 - t3n-1

 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
	MX_ADC4_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM15_Init();
	MX_TIM16_Init();
	MX_USART2_UART_Init();
	MX_TIM7_Init();
	MX_TIM6_Init();
	MX_TIM8_Init();
	MX_TIM17_Init();
	MX_DMA_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)  						// Hier beginnt Hauptprogramm:
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/******************************************************************
		 * *************************User_State_Machine**********************
		 * ****************************************************************/
		/**************************State Machine Function************************************************/

		void state_machine()
		{
			States_t State_Manager = Start;

			switch(State_Manager)
			{

			case Start:

				while (Doorgetstate() == 0)				// Erfüllt, wenn Türe noch offen ist
				{
					Laser_switch_state(0);				// Mach Laser aus

					if (ref_z == 0)						// und noch keine Referenzfahrt erledigt ist
					{
						home_z_achse();					// Dann home mal die z-Achse
					}
					Zebene_runter();					// Fahr eine Schicht runter pro druck auf taster
					lcd_print_state_cmd();				//Display function for start drucker
				}

				State_Manager = Init;
				break;

				// nachdem z-Achse ajustiert ist und Türe jz geschlossen wird und start taster gedruckt ist
				while ((Doorgetstate() == 1) && (Start_Taster_getstate() == 1))
				{
			case Init:
				lcd_clear();					//Clear display
				lcd_print_temp_cmd();			//function display to initialise and heat polymer
				Init_Drucker();					// ruf function init drucker
				lcd_clear();					//Clear display
				Temperature_print();			// show current temparture
				if (Polymerready == 1)			// check if polymer 60 grad erreicht hat
					State_Manager = Drucken;	//switch zum drucken
				break;
			case Drucken:
				lcd_clear();					//Clear display
				Schrittmotor_y_achse_bewegen();	// y verfahren
				Z_Achse_ebene_sinken();			// eben sinken
				Show_temp_Freq_Time_Layer();	//show all parameter drucker temperatur, frequnz, remain tme and ebene
				State_Manager = Ende;
				break;
			case Ende:
				if (Anzahl_Ebenen ==250)		// Wenn gedruckt und Anzahl Ebenen 250 erreicht hat, dann
				{
					schwinger_stop();				// Mach Schwinger aus
					lcd_clear();					//Clear display
					lcd_print_End_cmd();			//show Fertig display
				}
				State_Manager = Start;				//spring wieder in case start
				break;
				}
			}
		}

	}


	/******************************while/if Abfrage programm Ablauf(ohne state machine)******************************
		while (Doorgetstate() == 0)					// Erfüllt, wenn Türe noch offen ist (wichtig, da z-Achse ja noch justiert werden muss)
		{
			if (ref_z == 0)						// und noch keine Referenzfahrt erledigt ist
			{
				home_z_achse();					// Dann home mal die z-Achse
			}
			Zebene_runter();					// Fahr eine Schicht runter pro druck auf taster
		}


		while (Doorgetstate() == 1)					// Wenn z-Achse ajustiert ist und Türe jz geschlossen wird
		{
			if ((Start_Taster_getstate() == 1) && (Init ==0))	// Wenn dann auch Start-Taster gedrückt wird UND drucker aktoren nicht init
			{
				Laser_switch_state(0);				// Mach Laser aus

				Init_Drucker();					// ruf function init drucker
				Init = 1;					// Setze Init auf 1
				Start_Taster_gedruckt = 0;			// Setze Variable von Taster gedrückt wieder auf 0
			}
			if ((Init == 1) && (Doorgetstate() == 1))		// Wenn jetzt Türe zu ist und bereits z-Achse initialisert
			{

				Schrittmotor_y_achse_bewegen();			// y verfahren
				Z_Achse_ebene_sinken();				// eben sinken
				frequency_print();				// display Ausgabe
				Drucken = 1;
			}
			if ((Drucken ==1)&& (Anzahl_Ebenen ==50))		// Wenn gedruckt und Anzahl Ebenen 50 erreicht hat, dann
			{
				schwinger_stop();				// Mach Schwinger aus
				Init = 0;
				Ende = 1;					// Setz Variable Ende auf 1
			}
		}
	 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
			|RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15
			|RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM17
			|RCC_PERIPHCLK_TIM8|RCC_PERIPHCLK_ADC12
			|RCC_PERIPHCLK_ADC34|RCC_PERIPHCLK_TIM2
			|RCC_PERIPHCLK_TIM34;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
	PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
	PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
	PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
	PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
	PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc3) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief ADC4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC4_Init(void)
{

	/* USER CODE BEGIN ADC4_Init 0 */

	/* USER CODE END ADC4_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC4_Init 1 */

	/* USER CODE END ADC4_Init 1 */
	/** Common config
	 */
	hadc4.Instance = ADC4;
	hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc4.Init.Resolution = ADC_RESOLUTION_12B;
	hadc4.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc4.Init.ContinuousConvMode = ENABLE;
	hadc4.Init.DiscontinuousConvMode = DISABLE;
	hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc4.Init.NbrOfConversion = 1;
	hadc4.Init.DMAContinuousRequests = DISABLE;
	hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc4.Init.LowPowerAutoWait = DISABLE;
	hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc4) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC4_Init 2 */

	/* USER CODE END ADC4_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2000090E;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 3599;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 36-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 100-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 220-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xffff-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 220-1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0xffff-1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 10;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 9;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 1000;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void)
{

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 36000-1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 20000-1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 72-1;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 0xffff-1;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void)
{

	/* USER CODE BEGIN TIM15_Init 0 */

	/* USER CODE END TIM15_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 36000-1;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 200-1;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */

	/* USER CODE END TIM15_Init 2 */
	HAL_TIM_MspPostInit(&htim15);

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void)
{

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 0;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 0;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void)
{

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 0;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 0;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */

	/* USER CODE END TIM17_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, DIR_H_Pin|DIS_H_Pin|SO_H_Pin|LED_GR_N_Pin
			|STEP1_Z_Pin|DIR1_Z_Pin|ENA1_Z_Pin|RESET1_Z_Pin
			|Extra_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Laser_Pin|LD2_Pin|LED_ROT_Pin|LED_GELB_Pin
			|SLEEP1_Z_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SLEEP2_Y_GPIO_Port, SLEEP2_Y_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, STEP2_Y_Pin|DIR2_Y_Pin|ENA2_Y_Pin|RESET2_Y_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Blue_pushbutton_Pin */
	GPIO_InitStruct.Pin = Blue_pushbutton_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(Blue_pushbutton_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR_H_Pin DIS_H_Pin SO_H_Pin LED_GR_N_Pin
                           STEP1_Z_Pin DIR1_Z_Pin ENA1_Z_Pin RESET1_Z_Pin 
                           Extra_Pin */
	GPIO_InitStruct.Pin = DIR_H_Pin|DIS_H_Pin|SO_H_Pin|LED_GR_N_Pin
			|STEP1_Z_Pin|DIR1_Z_Pin|ENA1_Z_Pin|RESET1_Z_Pin
			|Extra_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Laser_Pin LD2_Pin LED_ROT_Pin LED_GELB_Pin
                           SLEEP1_Z_Pin */
	GPIO_InitStruct.Pin = Laser_Pin|LD2_Pin|LED_ROT_Pin|LED_GELB_Pin
			|SLEEP1_Z_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ENDLAGE_Y_Pin */
	GPIO_InitStruct.Pin = ENDLAGE_Y_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(ENDLAGE_Y_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DAC_MESSPIN_Pin */
	GPIO_InitStruct.Pin = DAC_MESSPIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DAC_MESSPIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ENDLAGE_Z_Pin */
	GPIO_InitStruct.Pin = ENDLAGE_Z_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(ENDLAGE_Z_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : TASTER3_PAUSE_Pin T_R_SCHALTER_Pin TASTER1_START_Pin */
	GPIO_InitStruct.Pin = TASTER3_PAUSE_Pin|T_R_SCHALTER_Pin|TASTER1_START_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : SLEEP2_Y_Pin */
	GPIO_InitStruct.Pin = SLEEP2_Y_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SLEEP2_Y_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : STEP2_Y_Pin DIR2_Y_Pin ENA2_Y_Pin RESET2_Y_Pin */
	GPIO_InitStruct.Pin = STEP2_Y_Pin|DIR2_Y_Pin|ENA2_Y_Pin|RESET2_Y_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/******************************************************************
 * *************************User_Interrupts**********************
 * ****************************************************************/
/******************************Sinus_erzeugung_interrupt**************************************/

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *ht) 		// Funktion zum Abschreiben der Sinustabelle per Timerinterrupt
{
	if (ht == &htim6)						// wenn timer is timer 6 dann
	{
		zaehler++;						// Bei jedem Interrupt geht der Sinus-Wert-Zähler +1 hoch..
		int position = zaehler%128;				// Teile aktuelle Position durch 128 und gibt Rest aus
		if (position==0)					// Wenn bei Division durch 128 kein rest bleibt (eben wenn Zähler selbst 128), dann wieder postion gleich null, sprich sinus ist 0
		{							// Wenn 128 erreicht wird hier dann Drehrichtung der H-Brücke geändert
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);		// DIR-Pin an H_Brücke wird getogglet

		}


		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, sinus[position]*3599*kdc);		//setze neu duty cycle wert in CCR mit ARR=3599
	}

	/******************************Drucken_Agorithmus**************************************/

	if ((ht == &htim3) &&(flag_schr_z == 0)) //&&		//wenn Timer is timer 3 auslöst (und z nicht in bewegvorgang ist) dann anfang belichten
	{
		if (Anzahl_Schwingung >= 20)			//Sind bereits mehr als 20 Schwingungen durchgeführt?
		{

			zaehler_X++;				// Setze zaehler_X um eins nach oben: zaehler_x ist aktuelle x-Koordinate
			zaehler_X = zaehler_X%266;		// Teile aktuelle x-Koordinate durch 266: 250+16 alls offset
			__HAL_TIM_SET_COUNTER(&htim3,0);	// Setze Counter 3 zurück
			if (zaehler_X==0)			// Wenn "zaehler_X = zaehler_X%266;" gleich 0 ist
				// (also Schwinger in Schwingung alle x-Koordinaten durchlaufen hat), dann {...}				/
			{
				HAL_TIM_Base_Stop_IT (&htim3);	// Dann stoppe TIM_3 per Interrupt

				zaehler_Y++;			// Wenn die Schwingung alle x-Koordinaten in einer Schwingung durch hat,
				// dann setze y-Koordinate um +1 höher
				zaehler_Y = zaehler_Y%250;	// Kontrolliere ob in y-Koordinate schon alle 250 Koordinaten durchlaufen sind
				// (Ablauf: Teile aktuellen Wert durch 250 und verwende den Rest %)
				if (zaehler_Y==0)		// Wenn aktuelle y-koordinate durch /250 keinen Rest ergibt (also wenn wir auf y = 250 stehen)
				{
					flag_schr_z = 1; 	// hier: Ebene absenken oder für kreis. "flag_schr_z" bedeutet, dass als nächstes der z-Motor verfahren wird
					// und so lange keine Belichtung stattfinden kann.
					// dann z-Achse_ebene_sinken();
				}
			}

			/********************************Belichten************************************/
			if (zaehler_X >= 15)			// Wenn 15 Felder auf x-Koordinate durchlaufen sind (also 15 Interrupts ausgelöst haben), dann:
			{
				dx = (zaehler_X-15-xm);		// Berechne den Abstand (dx) der aktuellen x-Position (zaehler_x) zum Mittelpunkt (xm)
				// und ziehe dann 15 davon ab. Sinn: Druckfeld eingrenzen, sodass die ersten 15 Interrupts keinen Laser auslösen
				dy = (zaehler_Y-ym);				// abstand dy ergibt sich aus aktuellen zahler - y mittelwert
				dxy = sqrtf((dx*dx)+(dy*dy));		//Satz pythagoras a^{2}+b^{2}=c^{2} ==>c={\sqrt {a^{2}+b^{2}}}.
			}
			if ((dxy <= 125)&&(dxy >= 75)) //Wandstarke der Ring: bereich wozum belichtem soll 125 -75 = 50
			{
				Laser_switch_state(1);			//laser on
			}
			else							//wenn außerhalb des bereich ist
			{
				Laser_switch_state(0);
			}
		}
	}


	/************************************Heizung Regelung**************************************/
	if (ht == &htim7)										// wenn timer is timer  dann
	{
		void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
		{
			if (hadc == &hadc4)
			{
				//HAL_ADC_PollForConversion(&hadc4, 1); // poll for conversion
				//tempReading = HAL_ADC_GetValue(&hadc4); // get the adc value
				tempReading = adc_buf[1];				//lese die werte aus dma buffer
				tempK = log(10000.0 * ((4096.0 / tempReading - 1))); //Specification of thermistor Modell
				tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK ))* tempK );
				tempC = tempK - 273.15;
				tempF = (tempC * 9.0)/ 5.0 + 32.0;
				printf("\nTemperature=%.2f\n",Filter_IIR_1_Order(tempC));	//Konsole Ausgabe
				//Temperature_print(); //LCD Ausgabe
				if ((tempC>50) && (tempC<60))
				{
					HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); //PWM für Heizung Start
					__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 100); // 50% duty cycle, langsam heizen
				}
				if (tempC>20 && tempC<50){
					HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
					__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 200); // 100% duty cycle, schnell Heizen
				}

				if (tempC>=60)
				{
					__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0); // 0% duty cycle, 0 spannung
					HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
					Polymerready = 1;
				}
			}
		}
	}
}
/******************************Input_capture_Lichtschranke_interrupt**************************************/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)		//Input Capture interrupt für lichtschranke
{
	if(htim==&htim4)											//wenn der timer 4 ist dann
	{
		/*************************Rising_edge_channel**************************************/

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

				dunkel_2 =(t_2-t_1);						// berechne dunkel zeit 2/250 aber unten offset
				dunkel_2_90P = (t_2-t_1)/280;				// berechne dunkel zeit 2 mit offset 250+15*2
				__HAL_TIM_SET_AUTORELOAD(&htim3, (uint16_t) dunkel_1_90P);	//setzten des ARRwert timer 3 als zeit für abtasten

				flag_schr_y = 1;							//flag für die bewegung der schrittmtor der y achse
				zustand =2;									//transisition auf zustand 2
				break;
			case 2:											// case third capture rising edge
				t_4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);		//read third capture
				frequenz_gemessen = faktor_psc/t_4;			// berechne die frequenz aus der dritte capture
				verhaeltnis = ((t_1+(t_3-t_2))/((t_2-t_1)+(t_4-t_3)));		//berechne aus der zeiten der verhaltnis

				dunkel_1 = (t_4-t_3);			//berechne die erste dunkle seite
				dunkel_1_90P = (t_4-t_3)/280;
				__HAL_TIM_SET_AUTORELOAD(&htim3, (uint16_t) dunkel_2_90P);//stell die wert der dunkel im timer 3 zur belichten


				flag_schr_y = 1;						// flag für bewegung y_achse
				ist_geschwungen = 1;						//setze ist geschwungen auf 1
				++Anzahl_Schwingung;
				/******************************Amplitude Regler***************************/
				Amplitude_regler();
				__HAL_TIM_SET_COUNTER(htim,0);								//counter wieder auf 0 setzen und zyklus neubeginnne
				zustand=1;
				break;
			}
		}
		/*************************Falling_edge_channel**************************************/

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)// if interrput source is channel 2
		{
			switch(zustand)								// Capture Zustande für falling Edge
			{
			case 1:										//first Capture falling
				t_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);		//read first capture fallng
				hell_1 = t_1/1000;						//berechne daraus die hell zeit 1
				HAL_TIM_Base_Start_IT (&htim3);					//timer 3 zum belichten starten

				break;
			case 2:											//second Capture falling
				t_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	//read second capture
				hell_2 = (t_3-t_2)/1000;						//berechne die zweite hell seite

				//flag_schr_y = 1;							//stell das flag für bewegung schrittmotor
				HAL_TIM_Base_Start_IT (&htim3);				//start timer 3 zur belichten mit interrupt


				break;
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)	// exti callback: kann nutzlich für frequenz , kdc, ebene einstellen usw.
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		Ebenerunter = 1;
	}

}

/*TODO List:
 *Optimierung
 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char *file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
