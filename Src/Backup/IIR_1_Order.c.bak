/*
 * IIR_1_Order.c
 *
 *  Created on: 09.12.2019
 *      Author: Home
 */
/********** IIR Filter Program************/
#include "IIR_1_Order.h"

float old_value=0.0f;

float coeffs[2] = {   // Koeffizienten des Filters (werte aus der Vorlesung entnommen)

		0.1f,  // new value weight:Je höher, desto schneller reagiert die Ausgabe,
		//aber es kommt auch mehr Rauschen durch.

		0.9f   // old value weight:Je höher, desto stärker werden Störungen entfernt,
		//desto langsamer ändert sich aber auch die Ausgabe des Filters
};

float Filter_IIR_1_Order (float new_value){ // IIR Filter 1 order function

	// Berechnung y = a0 * x(k-0) + a1 * x(k-1)
	return old_value = coeffs[0] * new_value + coeffs[1] * old_value;
}


/*Quelle: [atwillys.de] Digitale Filter in C für Embedded-Anwendungen
 * http://atwillys.de/content/cc/digital-filters-in-c/
 */
