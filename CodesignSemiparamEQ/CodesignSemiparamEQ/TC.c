/*
 * TC.c
 *
 * Created: 9/10/2019 3:10:35 PM
 *  Author: Flixor
 */ 


void Init_TC0(void){
	
	TC0 -> TC
	
	// nvic for interrupts, PERI ID 23 for tc0
	NVIC_SetPriority(TC0_IRQn, 1);
	NVIC_EnableIRQ(TC0_IRQn);
	
	
	
}