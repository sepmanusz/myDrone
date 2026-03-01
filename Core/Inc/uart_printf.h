#ifndef UART_PRINTF_H
#define UART_PRINTF_H

#include "main.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;

/* UART printf - adatok küldése */
void uart_printf(const char *format, ...);

#endif /* UART_PRINTF_H */
