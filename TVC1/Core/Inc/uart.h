#ifndef UART_H
#define UART_H

#include <stdint.h>

void uartInit(uint32_t baud);
void uartTransmit(char data);
void uartPrint(const char* str);
void uartPrintln(const char* str);
void uartPrint_int(int value);

#endif 