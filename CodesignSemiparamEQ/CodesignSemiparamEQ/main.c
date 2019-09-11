/*
 * ArmConvolution1.c
 *
 * Created: 8/21/2019 7:35:54 PM
 * Author : Flixor
 */ 


#define PMC_TIMEOUT 2048

#include "sam.h"
#include <component/pio.h>
#include <component/pmc.h>
#include <component/wdt.h>
#include <component/usart.h>
#include <component/pdc.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

#include "PMC.h"
#include "PIO.h"
#include "I2SC.h"
#include "UART.h"
#include "TWI.h"
#include "Matrix.h"
#include "AK4588EN.h"
#include "FPU.h"
#include "arm_math.h"
//#include "arm_const_structs.h"



#define I2SC_TXRDY_LED
#define TWI0_TXRDY_LED

#define PING	0
#define PONG	1
static volatile uint8_t newdata = 0;
static volatile uint8_t PingPong = PING;

#define CHANNEL_L	0
#define CHANNEL_R	1
uint8_t channelCount = CHANNEL_L;


/* FFT still necessary ?

#define FFT_FORWARD 0
#define FFT_INVERSE 1
#define FFT_BITREVERSE 1

const arm_cfft_instance_f32* S = &arm_cfft_sR_f32_len512;
 */


/* I2S */
#define I2SC_BUFFSZ 256

int32_t ReceiveBufL1[I2SC_BUFFSZ];
int32_t ReceiveBufL2[I2SC_BUFFSZ];
int32_t ReceiveBufR1[I2SC_BUFFSZ];
int32_t ReceiveBufR2[I2SC_BUFFSZ];

int32_t TransmitBufL1[I2SC_BUFFSZ];
int32_t TransmitBufL2[I2SC_BUFFSZ];
int32_t TransmitBufR1[I2SC_BUFFSZ];
int32_t TransmitBufR2[I2SC_BUFFSZ];

float transferBufInputL[I2SC_BUFFSZ];
float transferBufInputR[I2SC_BUFFSZ];
float transferBufOutputL[I2SC_BUFFSZ];
float transferBufOutputR[I2SC_BUFFSZ];


/* SVF */
#define FS 48000.0
#define SVF_Q 0.5 //  eigenlijk Q = 2; q = 1/Q
#define FC_INIT 1000.0
#define FC_LIM_UPPER 4000.0
#define FC_LIM_LOWER 250.0
// rxrdy interrupt
static volatile float Fc = FC_INIT;
static volatile float saved_Fc;
static volatile float amplfactor = 1.0f;



/* Debug */
char floatPrintStr[40] = "%d:\t g:%g \t f:%f \t li:%li \r\n"; // args: {str, ctr, value, value, value}

#define FILTER_ON 1



/* Functions Calls */
static void Init_Board(void);
static uint8_t Init_Clock(void);



float svf_bandpass(float input) {
	static float hp, bp, lp, prev_bp, prev_lp;
	
	float f = 2.0f * sin(PI * Fc / FS);
	
	prev_bp = bp;
	prev_lp = lp;
	
	lp = prev_bp*f + prev_lp;	
	hp = input - prev_bp*SVF_Q - lp;
	bp = hp*f + prev_bp;
	
	return bp * amplfactor;
}


int main(void)
{
	/* Initialize the SAM system, run of default 8MHz clock ATM */
	SystemInit();
	
	/* Setup Board and peripherals */
	Init_Board();

	// print initial fc value
	printf("Fc: \t%g\r\n", Fc);
			
			
	while (1) {
	
		if (newdata) {

			if (PingPong == PING) {
				
				for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
					if (FILTER_ON)	{ TransmitBufR2[i] = svf_bandpass(ReceiveBufR2[i]); }
									  //transferBufOutputR[i] *= factor;
									  //TransmitBufR2[i] = transferBufOutputR[i]; }
					else			{ TransmitBufR2[i] = ReceiveBufR2[i]; }
				}			
				
			} else { // == PONG
				
				for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
					if (FILTER_ON)	{ TransmitBufR1[i] = svf_bandpass(ReceiveBufR1[i]); }
					else			{ TransmitBufR1[i] = ReceiveBufR1[i]; }
				}
				
			}

			newdata = 0;
									
		}	
		
		asm("nop");
		
	}
}


static void Init_Board(void)
{
	#if (__FPU_PRESENT == 1)
	fpu_enable();
	#endif
	Init_Clock();	//setup clock to run @ 120MHz instead of the default 8MHz
	/* Disable Watchdog */
	WDT -> WDT_CR = WDT_CR_KEY_PASSWD;
	WDT -> WDT_MR = WDT_MR_WDDIS;	//disable WDT, register can be written only once
	
	Setup_PMC();
	Init_PIO();
	
	
	
	#ifdef PCK1_OUTPUT_ON_PINA30
	/* Configure CLKOUT pin */
	PIOA -> PIO_PUER	|= PIO_PUER_P30;	//pullup pin A17
	PIOA -> PIO_PDR		|= PIO_PDR_P30;		//PIO disable pin A17
	/* PA30 to peripheral A (make sure) */
	PIOA -> PIO_ABCDSR[0] &=~ (PIO_ABCDSR_P30);
	PIOA -> PIO_ABCDSR[1] &=~ (PIO_ABCDSR_P30);
	#endif
	
	Init_UART();
	Setup_Matrix();
	Init_TWI();
	AK4588EN_Init();
	
	#if (__FPU_USED == 1)
	UART_Printf("fpu, %d\r\n", ((REG_CPACR & 0xF00000) >> 20));
	#endif

	
	Init_I2SC(TransmitBufL1, TransmitBufR1, ReceiveBufL1, ReceiveBufR1, I2SC_BUFFSZ);

	UART_Puts("\r\nBoard_Init successful\r\nNow starting program\r\n");
}

//chapter 17.3
static uint8_t Init_Clock(void)	//run clock @ 120MHz
{
	EFC->EEFC_FMR = EEFC_FMR_FWS(5) | EEFC_FMR_CLOE; //set flash wait state to max. for 120MHz clock
	
	//enable crystal osc source
	SUPC -> SUPC_CR = SUPC_CR_KEY_PASSWD | SUPC_CR_XTALSEL;		//switch slow clock on EXTOSC
	while (!  ((SUPC->SUPC_SR & SUPC_SR_OSCSEL) && (PMC->PMC_SR & PMC_SR_OSCSELS))); //wait for OSC to be ready
	
	//PLL enable and lock
	/* 120.000.000 = 32.768 * (3661 + 1) */
	PMC -> CKGR_PLLAR = CKGR_PLLAR_MULA(0);		//Always stop PLL first
	PMC -> CKGR_PLLAR =  CKGR_PLLAR_MULA(3661) | CKGR_PLLAR_PLLAEN(1) | CKGR_PLLAR_PLLACOUNT(0x3FU);
	while (! (PMC -> PMC_SR & PMC_SR_LOCKA) );	//wait for PLL lock
	
	//PMC switch MCK to PLL
	PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk));
	for (uint32_t ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);	--ul_timeout) {
		if (ul_timeout == 0) {
			return 1;
		}
	}
	
	PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |	PMC_MCKR_CSS_PLLA_CLK;

	for (uint32_t ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);	--ul_timeout) {
		if (ul_timeout == 0) {
			return 1;
		}
	}

	//trim clock to 120MHz    trim value 12
	uint32_t ul_pwmr = SUPC -> SUPC_PWMR & (~(0xFu << 9));
	SUPC -> SUPC_PWMR = SUPC_PWMR_KEY_PASSWD | ul_pwmr | SUPC_PWMR_ECPWRS | ((12 & 0xFu) << 9);
	
	return 0;
	
}

Pdc *I2SC0_2 = (Pdc *)((uint32_t)I2SC0 + 0x200U);                  /**< \brief (I2SC0 PDC ) Base Address R channel */


void I2SC0_Handler(void) {
	
	if (channelCount == CHANNEL_L && (I2SC0 -> I2SC_SR & I2SC_SR_ENDTX) ) {

		if (PingPong == PING) {
			I2SC0 -> I2SC_TNPR = (uint32_t)ReceiveBufL1; //transmit buffer 1
			I2SC0 -> I2SC_TNCR = I2SC_BUFFSZ;
			
			I2SC0 -> I2SC_RNPR = (uint32_t)ReceiveBufL1; //receive to buffer 1
			I2SC0 -> I2SC_RNCR = I2SC_BUFFSZ;

			} else { // PONG
			I2SC0 -> I2SC_TNPR = (uint32_t)ReceiveBufL2;	//transmit buffer 2
			I2SC0 -> I2SC_TNCR = I2SC_BUFFSZ;

			I2SC0 -> I2SC_RNPR = (uint32_t)ReceiveBufL2;		//read to buffer 2
			I2SC0 -> I2SC_RNCR = I2SC_BUFFSZ;
		}

		channelCount = CHANNEL_R;
		
	} else if (channelCount == CHANNEL_R && (I2SC0 -> I2SC_SR & I2SC_SR_ENDTX) ) {	
		
		if (PingPong == PING) {
			I2SC0_2 -> PERIPH_TNPR = (uint32_t)TransmitBufR1;
			I2SC0_2 -> PERIPH_TNCR = I2SC_BUFFSZ;

			I2SC0_2 -> PERIPH_RNPR = (uint32_t)ReceiveBufR1;
			I2SC0_2 -> PERIPH_RNCR = I2SC_BUFFSZ;

			PingPong = PONG; 
			} else { // PONG
			I2SC0_2 -> PERIPH_TNPR = (uint32_t)TransmitBufR2;
			I2SC0_2 -> PERIPH_TNCR = I2SC_BUFFSZ;
			
			I2SC0_2 -> PERIPH_RNPR = (uint32_t)ReceiveBufR2;
			I2SC0_2 -> PERIPH_RNCR = I2SC_BUFFSZ;
			
			PingPong = PING;
		}
		
		channelCount = CHANNEL_L;
		newdata = 1;
	}
}


// USART1 RXRDY interrupt
void FLEXCOM1_Handler(void){
	
	char c = USART1 -> US_RHR;
	
	if ((c == 'q' || c == 'Q') && Fc < FC_LIM_UPPER){
		Fc += FC_LIM_LOWER;
		} else if ((c == 'a' || c == 'A') && Fc > FC_LIM_LOWER){
		Fc -= FC_LIM_LOWER;
	}
	
	//amplfactor = 1.0f / (float) pow(2.0, (Fc/(FC_LIM_UPPER)));
	amplfactor = 1.0f;
	
	if (Fc != saved_Fc) {
		printf("Fc: \t%g\t amplfactor: \t%g\r\n", Fc, amplfactor);
		saved_Fc = Fc;
	}
	
	USART1 -> US_CR |= US_CR_RSTSTA_Msk; // clears overrun error OVRE bit in receiver, which then clears RXRDY
}

