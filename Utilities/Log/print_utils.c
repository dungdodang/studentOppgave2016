/*
 * print_utils.c
 *
 * Created: 29.01.2012 18:38:25
 *  Author: Dung Do Dang
 */ 

#ifndef PRINT_UTILS_H_
#define PRINT_UTILS_H_

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "print_utils.h"
#include "stm32f7xx_hal.h"

/* private pointer to save the uart handler */
static UART_HandleTypeDef *huart;

/**
  * @brief  Assign the uart handler pointer and enable the uart
  * @param  ptr: uart handler
  * @retval None
  */
void printUart_init(UART_HandleTypeDef *ptr)
{
	huart = ptr;
	
	__HAL_UART_ENABLE(huart);
}

/**
  * @brief  This funtion takes arguments like the standard printf() function call and print text to uart.
	* @param 	Same arguments as printf(): printf("A number: %d \r\n", aNumber);
  * @retval None
  */
void printUart(const char *format, ...)
{
	va_list arg;
  char text[256];
  unsigned char i=0;

  va_start(arg, format);
  i = vsprintf(text, format, arg);
  while (i < sizeof(text) - 1) text[i++] = '\0';
  va_end(arg);
	HAL_UART_Transmit(huart,(uint8_t *)&text,strlen(text),1000);
}

#endif /* PRINT_UTILS_H_ */
