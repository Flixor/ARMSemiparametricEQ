/*
 * UART.c
 *
 * Created: 18-Mar-18 22:16:50
 *  Author: Willem van der Kooij
 */ 

#include "sam.h"
#include <component/pio.h>
#include <component/pmc.h>
#include <component/usart.h>

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "UART.h"

static void USART_PioSetup(void);
static char *convert(unsigned int num, int base);

 /* for printf */
extern volatile void *volatile stdio_base;
extern int (*ptr_put)(void volatile*, char);


static inline int Ctrl_putchar(void volatile* usart, const char c);




/*
 * Setup USART1 to use with debugging
 *
 * Settings UART 1 (yes, asynchronous)
 *	Baud	=	115200
 *	Length	=	8 bits
 *	Parity	=	None
 *	Stop	=	1 bit
 */


void Init_UART(void)
{	
	USART_PioSetup();
	
	FLEXCOM1 -> FLEXCOM_MR = FLEXCOM_MR_OPMODE_USART;
	
	//write protection
	USART1 -> US_WPMR &= ~(US_WPMR_WPEN);
	USART1 -> US_WPMR = US_WPMR_WPKEY_PASSWD;
	
	//setup baud 115K2 (130 for 120MHz) (9 for 8 MHz)
	USART1 -> US_BRGR	=	US_BRGR_CD(130);	//CLKDIV
	
	
	//setup USART1
	USART1 -> US_MR	=	US_MR_USART_MODE_NORMAL
					|	US_MR_USCLKS_MCK
					|	US_MR_CHRL_8_BIT		//8-bit
					|	US_MR_PAR_NO			//no parity
					|	US_MR_NBSTOP_1_BIT		//1 stop bit
					|	US_MR_CHMODE_NORMAL
					|	US_MR_OVER;				//oversampling
	
	USART1 -> US_CR	=	US_CR_RXEN	//enable receiver
					|	US_CR_TXEN;	//enable transmitter
		
							
	/* Setup stream for printf (link syscalls) */
	stdio_base = (void *)USART1;
	ptr_put = (int (*)(void volatile*,char))&Ctrl_putchar;
	
	setbuf(stdout, NULL);	//dont buffer stdout
	setbuf(stdin, NULL);	//dont buffer stdin
						
						
	printf("UART Initialized\r\n");	
}

static void USART_PioSetup(void)
{
	//key should be written to PIO already
	//enable peripheral control of IO pins
	PIOB -> PIO_PDR	|=	PIO_PDR_P2	//Rx
					|	PIO_PDR_P3;	//Tx
	
	//PB2 & PB3 to peripheral A
	PIOB -> PIO_ABCDSR[0] &= ~(0x03 << 2);
	PIOB -> PIO_ABCDSR[1] &= ~(0x03 << 2);
}


static inline int Ctrl_putchar(void volatile* usart, const char c)
{
	USART1 -> US_THR = (c & 0xFF);
	while (!(((USART1 -> US_CSR) & US_CSR_TXRDY_Msk) == US_CSR_TXRDY_Msk));	//wait for character to shift out
	return 0;
}



void UART_Putchar(char c)
{
	USART1 -> US_THR = (c & 0xFF);
	while (!(((USART1 -> US_CSR) & US_CSR_TXRDY_Msk) == US_CSR_TXRDY_Msk));	//wait for character to shift out
}

void UART_Puts(const char *str)
{
	for (uint8_t i = 0; i < strlen(str); i++) {
		UART_Putchar(str[i]);
	}
}

void UART_Printf(char* format, ...)
{
	char *traverse;
	unsigned int i;
	char *s;
	
	va_list arg;
	va_start(arg, format);
	
	for (traverse = format; *traverse != '\0'; traverse++) {
		while (*traverse != '%' && *traverse != '\0') {
			UART_Putchar(*traverse);
			traverse++;
		}

		if (*traverse != '\0') {
			traverse++;
		} else break;

		switch (*traverse) {
			case 'c':	
				i = va_arg(arg,int);
				UART_Putchar(i);
				break;
			case 'd':
				i = va_arg(arg,int);
				if (i < 0) {
					i = -i;
					UART_Putchar('-');
				}
				UART_Puts(convert(i, 10));
				break;
			case 'o':
				i = va_arg(arg, unsigned int);
				UART_Puts(convert(i, 8));
				break;
			case 's': s = va_arg(arg, char*);
				UART_Puts(s);
				break;
			case 'x': 
				i = va_arg(arg, unsigned int);
				UART_Puts(convert(i, 16));
				break;
			case 'X':
				i = va_arg(arg, unsigned int);
				char *CapitalHex = convert(i, 16);
				UART_Puts(CapitalHex + 0x41);
				break;
			case 'f':
				//floating point to string
				//puts(sprintf(va_arg(arg, char*)));
				break;
		}
	}
	va_end(arg);
}

char* convert(unsigned int num, int base)
{
	static char Representation[17]= "0123456789ABCDEF";
	static char buffer[50];
	char *ptr;

	ptr = &buffer[49];
	*ptr = '\0';

	do {
		*--ptr = Representation[num%base];
		num /= base;
	}
	while(num != 0);
	
	return (ptr);
}

	

char UART_Getchar(void)
{
	//while (!(((USART1 -> US_CSR) & US_CSR_RXRDY_Msk) == US_CSR_RXRDY_Msk));
	return USART1 -> US_RHR;
}


