#include <stdio.h>
#include <stdarg.h>
#include "util.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

void uart_printf(const char *fmt, ...)
{
    char buffer[256];
    int len;
    va_list args;
    va_start(args, fmt);
    len = vsnprintf(buffer, 256, fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (unsigned char *) buffer, len, -1);
}
