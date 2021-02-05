/*
 * PMC.c
 *
 * Created: 22-Apr-18 17:14:07
 *  Author: Willem van der Kooij
 */ 

#include "sam.h"
#include <component/pmc.h>

#include "PMC.h"

/*
 * Clock Frequencies
 * 
 * Master Clock = PLLA Clock
 * Main Clock = 8MHz
 * Slow Clock = XOSC
 * PLLA Clock = 120MHz
 */

void Setup_PMC(void)
{	
	/* Write Key */
	PMC -> PMC_WPMR  &=~PMC_WPMR_WPEN;
	PMC -> PMC_WPMR  |= PMC_WPMR_WPKEY_PASSWD;
			
	/* Init IO ports */
	PMC -> PMC_PCER0	=	PMC_PCER0_PID8		//enable FLEXCOM_0 for TWI0
						|	PMC_PCER0_PID9		//enable FLEXCOM_1 for UART1
						|	PMC_PCER0_PID11		//enable PIOA peripheral clock
						|	PMC_PCER0_PID12		//enable PIOB peripheral clock
						|	PMC_PCER0_PID16;	//enable I2SC0 peripheral clock
	
	/* Setup PCK1 for output on led, output freq of 800.000 Hz */
	PMC -> PMC_SCDR		= PMC_SCDR_PCK1;					//disable system clock 6 (FLEXCOM 0/1/2/3)
	PMC -> PMC_PCK[1]	= PMC_PCK_CSS(PMC_PCK_CSS_MCK_Val) | PMC_PCK_PRES(0);	//use master clock with prescaler(149 + 1)
	while(!((PMC -> PMC_SR & PMC_SR_PCKRDY1)));				//wait for programmable clock to be ready
	PMC -> PMC_SCER		|= PMC_SCER_PCK1;					//enable PCK1
		
	/* Setup PCK4 for I2SC */
	PMC -> PMC_SCDR		|= PMC_SCDR_PCK4;	//disable clock before setup
	PMC -> PMC_PCK[4]	 = PMC_PCK_CSS(PMC_PCK_CSS_MCK_Val)| PMC_PCK_PRES(4); //120.000.000 / 4 = 30.000.000 clock
	while (!(PMC -> PMC_SR & (PMC_SR_PCKRDY4)));
	PMC -> PMC_SCER		|= PMC_SCER_PCK4; //enable PCK4
	
	/* Setup PCK6 for TWI, output freq of 800.000 Hz */
	PMC -> PMC_SCDR		= PMC_SCDR_PCK6;					//disable system clock 6 (FLEXCOM 0/1/2/3)
	PMC -> PMC_PCK[6]	= PMC_PCK_CSS(PMC_PCK_CSS_MCK_Val) | PMC_PCK_PRES(149);	//use master clock with prescaler(149 + 1)
	while(!((PMC -> PMC_SR & PMC_SR_PCKRDY6)));				//wait for programmable clock to be ready
	PMC -> PMC_SCER		|= PMC_SCER_PCK6;					//enable PCK6
}