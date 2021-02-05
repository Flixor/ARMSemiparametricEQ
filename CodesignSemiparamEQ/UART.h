/*
 * UART.h
 *
 * Created: 19-Mar-18 14:24:43
 *  Author: Willem van der Kooij
 */ 


#ifndef UART_H_
#define UART_H_

void Init_UART(void);
void UART_Putchar(char c);
void UART_Puts(const char *str);
char UART_Getchar(void);
void UART_Printf(char* format, ...);

#endif /* UART_H_ */