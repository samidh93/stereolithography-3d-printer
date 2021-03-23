/*
 * print_scan_usart2.h
 *
 *  Created on: 30.11.2019
 *      Author: Home
 */

#include "stm32f3xx_hal.h"
extern UART_HandleTypeDef huart2;

#ifndef PRINT_SCAN_USART2_H_
#define PRINT_SCAN_USART2_H_

int _write(int32_t file, uint8_t *ptr, int32_t len)
{
	HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
	return len;
}
int _read(int32_t file, uint8_t *ptr, int32_t len)
{
	HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
	return len;
}


#endif /* PRINT_SCAN_USART2_H_ */
