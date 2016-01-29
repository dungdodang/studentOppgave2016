/*
 * print_utils.h
 *
 * Created: 29.01.2012 18:39:12
 *  Author: Dung Do Dang
 */ 


#ifndef PRINT_UTILS_H_
#define PRINT_UTILS_H_
#include "stm32f7xx_hal.h"

void printUart_init(UART_HandleTypeDef *huart_ptr);
void printUart(const char *format, ...);

#endif /* PRINT_UTILS_H_ */
