/*
 * Filter_3_Order.c
 * Created on: 08.12.2019
 * Author: Sami
 */
#include "Filter_3_Order.h"
// Filter coefficient is 1(no Weight)
float FIR[3];
int FIR_zaehler = 0;
float Sum_Fir = 0;
float Div_Fir = 0;
float Filter_FIR_3_Order (float FIR_Value) // FIR Filter 3 order function
{
	for (FIR_zaehler = 0; FIR_zaehler <2; FIR_zaehler++) // for schleife für 3 Werte
	{
		FIR[FIR_zaehler]= FIR_Value;		     // aüsfüllen des FIR Array
		Sum_Fir = (Sum_Fir+FIR[FIR_zaehler]);   // summieren alle werte des Array dann dividieren durch 3
		Div_Fir = Sum_Fir/3;					// Resultat
	}
	FIR_zaehler = 0;
	Sum_Fir = 0;

	return(Div_Fir); // Rückgabe der Wert des filters
}
