#include "uart_printf.h"
#include <stdarg.h>
#include <string.h>

#define BUFFER_SIZE 512

/**
 * @brief UART printf implementáció - float támogatás engedélyezésével
 */
void uart_printf(const char *format, ...)
{
    static char buffer[BUFFER_SIZE];
    va_list args;
    
    va_start(args, format);
    /* Printf float támogatás: -u _printf_float flag szükséges a linkerben */
    vsnprintf(buffer, BUFFER_SIZE, format, args);
    va_end(args);
    
    HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
