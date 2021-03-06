/*
 * FIR_5Order_RingBuffer.c
 * Created on: 10.12.2019
 * Author: Sami
 */
/*
 * *This function is called every cycle. It substracts the oldest value from
 * the sum, adds the new value to the sum, overwrites the oldest value with the
 * new value and increments the ring buffer pointer (with rewind on overflow).
 */
// This Fir 5 order Filter use the Ring Buffer Method to minimize data storage when moving average
// Coef =1; no Weight

#include "FIR_5Order_RingBuffer.h"

float filter_buffer[5];			//Buffer Filter 5 werte
float filter_sum = 0;			// summe der werte buffer
float *filter_position;			//zeiger auf die position in buffer

float FIR_5_RingBuffer(float new_value){

filter_sum = 0;					//initilaise summe
filter_position = filter_buffer;//initialise zeiger anfangs array
for(int i=0; i<5; i++)
	{
	filter_buffer[i] = 0;		//alle speicher auf 0 setzen
	}
	filter_sum -= *filter_position;  // alt wert von der summe substrahieren
	*filter_position = new_value;    // old value mit new ?berschreiben
	filter_sum += new_value;         //summe mit neuer wert inkrementieren
	//check ob ring buffer voll ist
	if(++filter_position >= filter_buffer + 5)//
	{
	filter_position = filter_buffer; // setzt zeiger wieder auf erste wert speicher

	}

	return filter_sum *0.2; // return das resultat der Summe/5 oder Summe*0.2
}
/* Quelle : [atwillys.de] Digitale Filter in C f?r Embedded-Anwendungen
 * http://atwillys.de/content/cc/digital-filters-in-c/
 */
