/*
 * TWI.c
 *
 * Created: 19-Mar-18 13:54:43
 *  Author: Willem van der Kooij
 */ 

#include "sam.h"
#include <component/pio.h>
#include <component/pmc.h>
#include <component/twi.h>
#include <component/matrix.h>

#include "UART.h"
#include "TWI.h"
#include "AK4588EN.h"

static void TWI_PioSetup(void);


/*
 * TWI0
 *
 * Clock Calculations
 * Clock_In = (120.000.000 / 150 = 800.000)
 * 400.000 = 800.000 / 2
 * CLKDIV * 2^(CKDIV) = 8
 */
void Init_TWI(void)
{
	TWI_PioSetup();	
	
	FLEXCOM0 -> FLEXCOM_MR = FLEXCOM_MR_OPMODE_TWI;

		
	//write protection
	TWI0 -> TWI_WPMR &= ~(TWI_WPMR_WPEN);
	TWI0 -> TWI_WPMR = TWI_WPMR_WPKEY_PASSWD;
	
	TWI0 -> TWI_CWGR	=	TWI_CWGR_CLDIV(0x01)
						|	TWI_CWGR_CHDIV(0x01)
						|	TWI_CWGR_CKDIV(0x03)
						|	TWI_CWGR_BRSRCCLK;		//clock source PMC PCK6

	
	TWI0 -> TWI_CR		=	TWI_CR_MSEN		//master mode
						|	TWI_CR_SVDIS	//disable slave mode
						//|	TWI_CR_HSEN		//high speed mode
						|	TWI_CR_THRCLR;	//clear transmit holding register
	
	TWI0 -> TWI_FILTR	=	TWI_FILTR_PADFEN; //pad filter enable, must be enabled in high-speed mode		
					
	TWI0 -> TWI_MMR		=	TWI_MMR_IADRSZ_NONE	//no internal device address
						|	TWI_MMR_DADR(AK4588EN_I2C_ADDR);
							
	UART_Puts("TWI Initialized\r\n");	
	
}


void TWI_WriteByte(uint8_t slaveAddr, uint8_t txdata)
{
	TWI0 -> TWI_MMR = TWI_MMR_DADR(slaveAddr); //load master mode reg with slave addr and write mode
	//TWI0 -> TWI_CR	= TWI_CR_START; //done automagicaly with writes
	TWI0 -> TWI_THR = txdata;
	TWI0 -> TWI_CR	= TWI_CR_STOP;
	
	while (!(TWI0 -> TWI_SR & TWI_SR_TXRDY));
	while (!(TWI0 -> TWI_SR & TWI_SR_TXCOMP));

}


uint8_t TWI_Read(uint8_t slaveAddr, uint8_t RegAddr)
{
	TWI0 -> TWI_MMR |= (TWI_MMR_MREAD);			//Master Read Direction
	
	return ((TWI0 -> TWI_RHR) & 0xFF);
}


static void TWI_PioSetup(void)
{
	/* enable internal pull up on TWI pins PA9 (SCL) & PA10 (SDA) */
	PIOA -> PIO_PPDDR	|= PIO_PPDDR_P9 | PIO_PPDDR_P10;	//first disable internal pull down
	PIOA -> PIO_PUER	|= PIO_PUSR_P9 | PIO_PUSR_P10;		//enable internal pull up
	
	//enable peripheral control of IO pins
	PIOA -> PIO_PDR	|=	PIO_PDR_P9		//SCL
					|	PIO_PDR_P10;	//SDA
	
	//PA9 & PA10 to peripheral A (both bits should be zero)
	PIOA -> PIO_ABCDSR[0] &= ~(0x03 << 9);
	PIOA -> PIO_ABCDSR[1] &= ~(0x03 << 9);
}