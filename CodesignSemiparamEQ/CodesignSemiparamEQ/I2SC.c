/*
 * I2S.c
 *
 * Created: 18-Mar-18 22:16:10
 *  Author: Willem van der Kooij
 *	Subauthor lol: Flixor, 3sep19
 */ 

#include "sam.h"
#include <component/pio.h>
#include <component/pmc.h>
#include <component/i2sc.h>
#include <component/pdc.h>
#include <component/matrix.h>
#include <stdio.h>

#include "I2SC.h"
#include "UART.h"

#define BITS_PER_CHANNEL	32
#define SAMPLE_RATE			48000 // as determined by AK4558EN settings
#define BIT_RATE			(BITS_PER_CHANNEL * SAMPLE_RATE)

//#define I2SC_MASTER_MODE
#define I2SC_SLAVE_MODE

// I2SC_MR_FORMAT - bits 6-7 - I2S mode of LJ mode (reserved according to i2sc.h, FORMAT according to datasheet SAMG55)
#define I2SC_MR_FORMAT_Pos 6
#define I2SC_MR_FORMAT_Msk (_U_(0x3) << I2SC_MR_FORMAT_Pos) 
#define I2SC_MR_FORMAT_I2S (I2SC_MR_FORMAT_Msk & _U_(0x0) << I2SC_MR_FORMAT_Pos)
#define I2SC_MR_FORMAT_LJ (I2SC_MR_FORMAT_Msk & _U_(0x1) << I2SC_MR_FORMAT_Pos)
// 0x2 and 0x3 are still reserved


static void I2SC_PioSetup(void);


	/*
	In Master mode, if the Peripheral clock frequency is higher than 96 MHz, the PCKx clock from
	PMC must be selected as I2SC input clock by writing a ’1’ in the CLKSELx bit of the
	CCFG_I2CLKSEL register located in Matrix (See Figure 33-3 ”I2SC Clock Generation”).
	*/


void Init_I2SC(int32_t *DMAsrcbuffL, int32_t *DMAsrcbuffR, int32_t *DMAdstbuffL, int32_t *DMAdstbuffR, uint32_t DMAbuffLen)
{	
	I2SC_PioSetup();	//enable peripheral to use IO pins

	I2SC0 -> I2SC_CR	=	I2SC_CR_SWRST;
	
	#ifdef I2SC_MASTER_MODE
	I2SC0 -> I2SC_MR	=	I2SC_MR_MODE_MASTER
						|	I2SC_MR_DATALENGTH_32_BITS
						//|	I2SC_MR_RXLOOP			//loop data back internally
						|	I2SC_MR_TXSAME
						|	I2SC_MR_TXDMA
						|	I2SC_MR_IMCKDIV(5)			//combined with IMCKFS(31) / 1024_Val gives 128fs master clock
						|	I2SC_MR_IMCKFS(I2SC_MR_IMCKFS_M2SF256_Val)	//combined with IMCKDIV(7) gives 128fs master clock
						|	I2SC_MR_IMCKMODE;						
	#endif
	
	#ifdef I2SC_SLAVE_MODE
	I2SC0 -> I2SC_MR	=	I2SC_MR_MODE_SLAVE
						|	I2SC_MR_TXDMA
						|	I2SC_MR_RXDMA
						|	I2SC_MR_DATALENGTH_32_BITS
						//|	I2SC_MR_RXLOOP
						|	I2SC_MR_TXSAME;
	#endif
			
	I2SC_DMAenable(DMAsrcbuffL, DMAsrcbuffR, DMAdstbuffL, DMAdstbuffR, DMAbuffLen);
	
	I2SC0 -> I2SC_IER =	I2SC_IER_ENDTX; //ENDTX? or TXRDY
	
	NVIC_SetPriority(I2SC0_IRQn, 0);
	NVIC_EnableIRQ(I2SC0_IRQn);
		
	//fucking peripherals...., if written directly I2SC0 fails, write 1 by one
	I2SC0 -> I2SC_CR	=	0;
	I2SC0 -> I2SC_CR	|=	I2SC_CR_RXEN;	//enable receiver
	I2SC0 -> I2SC_CR	|=	I2SC_CR_TXEN;	//enable transmitter
	I2SC0 -> I2SC_CR	|=	I2SC_CR_CKEN;	//generate clock
		
				
	//UART_Puts("I2SC Initialized\r\n");	
	
}

void I2SC_DMAenable(int32_t *srcbuffL, int32_t *srcbuffR, int32_t *dstbuffL, int32_t *dstbuffR, uint32_t len)
{
	Pdc *I2SC0_2 = (Pdc *)((uint32_t)I2SC0 + 0x200U);                  /**< \brief (I2SC0 PDC ) Base Address R channel */
	
	//Left channel Tx
	I2SC0 -> I2SC_TPR = (int32_t)srcbuffL;
	I2SC0 -> I2SC_TCR = len;
	I2SC0 -> I2SC_TNPR = (int32_t)srcbuffL;
	I2SC0 -> I2SC_TNCR = len;
	
	I2SC0 -> I2SC_RPR = (int32_t)dstbuffL;
	I2SC0 -> I2SC_RCR = len;
	I2SC0 -> I2SC_RNPR = (int32_t)dstbuffL;
	I2SC0 -> I2SC_RNCR = len;
	
	//Right channel Tx 
	I2SC0_2 -> PERIPH_TPR = (int32_t)srcbuffR;
	I2SC0_2 -> PERIPH_TCR = len;
	I2SC0_2 -> PERIPH_TNPR = (int32_t)srcbuffR;
	I2SC0_2 -> PERIPH_TNCR = len;
	
 	I2SC0_2 -> PERIPH_RPR = (int32_t)dstbuffR;
 	I2SC0_2 -> PERIPH_RCR = len;
 	I2SC0_2 -> PERIPH_RNPR = (int32_t)dstbuffR;
 	I2SC0_2 -> PERIPH_RNCR = len;
	
	
	I2SC0 -> I2SC_PTCR		= I2SC_PTCR_TXTEN | I2SC_PTCR_RXTEN;
	I2SC0_2 -> PERIPH_PTCR	= I2SC_PTCR_TXTEN | I2SC_PTCR_RXTEN;
}

uint32_t I2SC_ReadData(void)
{
	//while (! (I2SC0 -> I2SC_SR & I2SC_SR_RXBUFF));
	return TwosComplementConvert(I2SC0 -> I2SC_RHR);
	//return I2SC0 -> I2SC_RHR;
}

void I2SC_WriteData(uint32_t txdata)
{
	I2SC0 -> I2SC_THR = TwosComplementConvert(txdata);
	//while (! (I2SC0 -> I2SC_SR & I2SC_SR_TXRDY));
}


static void I2SC_PioSetup(void)
{
	//enable peripheral control of IO pins
	PIOA -> PIO_PDR	|=	PIO_PDR_P0	//Clock
					|	PIO_PDR_P1	//Word Select
					|	PIO_PDR_P2	//Data In
					|	PIO_PDR_P3	//Data Out
					|	PIO_PDR_P4	//Master Clock
					|	PIO_PDR_P17 //Data Out 2
					|	PIO_PDR_P18;//Master Clock 2
	
	
	/* ABCDSR[] register setup
	 *	[1]	[2]		peripheral	
	 *	 0	 0		A
	 *	 1	 0		B
	 *	 0	 1		C
	 *	 1	 1		D
	 * since not used pins are not used in PCB hand them to peripheral as well
	 */
	
	//PA0 & PA1 to peripheral A
	PIOA -> PIO_ABCDSR[0] &= ~( PIO_ABCDSR_P0 | PIO_ABCDSR_P1 | PIO_ABCDSR_P17 | PIO_ABCDSR_P18 );
	PIOA -> PIO_ABCDSR[1] &= ~( PIO_ABCDSR_P0 | PIO_ABCDSR_P1 | PIO_ABCDSR_P17 | PIO_ABCDSR_P18 );
	
	//PA2 & PA3 & PA4 to peripheral B
	PIOA -> PIO_ABCDSR[0] |=  ( PIO_ABCDSR_P2 | PIO_ABCDSR_P3 | PIO_ABCDSR_P4 );
	PIOA -> PIO_ABCDSR[1] &= ~( PIO_ABCDSR_P2 | PIO_ABCDSR_P3 | PIO_ABCDSR_P4 );
}


int32_t TwosComplementConvert(int32_t data)
{
	data = ~(data);		//invert digits
	data = data + 1;	//add 1
	
	return data;
}
