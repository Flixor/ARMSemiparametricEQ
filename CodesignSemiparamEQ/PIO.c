/*
 * PIO.c
 *
 * Created: 22-Apr-18 17:16:12
 *  Author: Willem van der Kooij
 */ 


#include "sam.h"
#include <component/pio.h>
#include <stddef.h>

#include "PIO.h"


void Init_PIO(void)
{
	PIOA -> PIO_WPMR	&=~ PIO_WPMR_WPEN;
	PIOA -> PIO_WPMR	|=  PIO_WPMR_WPKEY_PASSWD;
	
	PIOB -> PIO_WPMR	&=~ PIO_WPMR_WPEN;
	PIOB -> PIO_WPMR	|=  PIO_WPMR_WPKEY_PASSWD;
	
	/*
	 * "Sometimes you should say fuck it and leave the pins to the peripherals fuck current consumption"
	 */
	
	PIOA -> PIO_PDR	=	0xFFFFFFFF;
	PIOB -> PIO_PDR	=	0xFFFFFFFF;
	
	PIOA -> PIO_PER		|=	PIO_PER_P27 | PIO_PER_P28 | PIO_PER_P29 | PIO_PER_P30;	//only pins I want to control through the PIO controller
	PIOA -> PIO_OER		|=	PIO_OER_P27 | PIO_OER_P28 | PIO_OER_P29 | PIO_PER_P30;
	PIOA -> PIO_CODR	|=	PIO_CODR_P27| PIO_CODR_P28| PIO_CODR_P29| PIO_PER_P30;
	
	Led3State(0);
	AK4558EN_PDN_State(1);
}

void Led3State(uint8_t state)
{
	if (state != 0) {
		/* light up LED */
		PIOA -> PIO_SODR	|= PIO_SODR_P28;		//Set output data register
	} else {
		PIOA -> PIO_CODR	|= PIO_CODR_P28;		//clear output data register
	}
}

void AK4558EN_PDN_State(uint8_t state)
{
	if (state != 0) {
		/* set high */
		PIOA -> PIO_SODR	|= PIO_SODR_P27;		//Set output data register
		} else {
		PIOA -> PIO_CODR	|= PIO_CODR_P27;		//clear output data register
	}
}