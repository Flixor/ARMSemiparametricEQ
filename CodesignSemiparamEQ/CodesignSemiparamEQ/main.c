/*
 * CodesignSemiparamEQ.c
 *
 * Created: 9/09/2019 7:35:54 PM
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


/* Functions Calls */
static void Init_Board(void);
static uint8_t Init_Clock(void);


/* I2S */
#define I2SC_TXRDY_LED
#define TWI0_TXRDY_LED

#define PING	0
#define PONG	1
static volatile uint8_t newdata = 0;
static volatile uint8_t PingPong = PING;

#define CHANNEL_L	0
#define CHANNEL_R	1
uint8_t channelCount = CHANNEL_L;

#define I2SC_BUFFSZ 256

int32_t ReceiveBufL1[I2SC_BUFFSZ];
int32_t ReceiveBufL2[I2SC_BUFFSZ];
int32_t ReceiveBufR1[I2SC_BUFFSZ];
int32_t ReceiveBufR2[I2SC_BUFFSZ];

int32_t TransmitBufL1[I2SC_BUFFSZ];
int32_t TransmitBufL2[I2SC_BUFFSZ];
int32_t TransmitBufR1[I2SC_BUFFSZ];
int32_t TransmitBufR2[I2SC_BUFFSZ];


/* params */
#define SR 48000.0f

#define MIN_V_DEV 18.0f

/* variables set in uart rxrdy interrupt */
static volatile float Vfreq = 1000.0f;
static volatile uint8_t new_freq;

static volatile float Vampl = 2000.0f;
static volatile uint8_t new_ampl;

static volatile float Q = 2.0f;
static volatile uint8_t new_q;

static volatile uint8_t boost = 0;
static volatile uint8_t new_direction;

/* Debug */
#define FILTER_ON 1


/* calculate and set coefficients */
static void calc_set_coeffs(float *coeffs, float V_freq, float V_ampl){
		
	/* gain of BP, for summing with orig signal */
	float gain_db_coeff_boost = 20 * log(V_ampl / 2000.0f) / log(10);
	/* yes, db needs to be divided by 40 in exponent (an extra sqrt), to get correct coefficient value */
	float gain_lin_coeff_boost = pow(10.0, gain_db_coeff_boost/40.0);

	/* for cut */
	float gain_db_coeff_cut = 20 * log(2000.0f / V_ampl + 1.0f) / log(10) * -1;
	float gain_lin_coeff_cut = pow(10.0, gain_db_coeff_cut/40.0);	
	
	/* total gain (after summing with orig signal) */
	float gain_lin_total = V_ampl / 2000.0f + 1.0f;
	float gain_db_total = 20 * log(gain_lin_total) / log(10);
	
	/* Vfreq [mV] == center frequency of BP */
	float wc = 2 * M_PI * V_freq / SR;
	
	/* should actually be alpha = sin(wc) / (Q * 2) here, 
	 * but we do a magic Q adjustment...
	 * I don't make the rules around here
	 * Measurements say jump, I say how high */
	float Q_drift = Q * (1 - wc / (M_PI * 4));
	float alpha_boost = sin(wc) * gain_lin_coeff_boost / (Q_drift * 2);
	float alpha_cut = sin(wc) * gain_lin_coeff_cut * gain_lin_total / (Q_drift * 2);
	
	
	float a0, a1, a2, b0, b1, b2;
	/* boost */
	if (boost) {
		b0 = alpha_boost * gain_lin_coeff_boost;
		b1 = 0;
		b2 = -1 * alpha_boost * gain_lin_coeff_boost;
		a0 = 1 + (alpha_boost / gain_lin_coeff_boost);
		a1 = -2 * cos(wc);
		a2 = 1 - (alpha_boost/ gain_lin_coeff_boost);
	}
	/* cut */
	else {
		b0 = -1 * alpha_cut * gain_lin_coeff_cut;
		b1 = 0;
		b2 = alpha_cut * gain_lin_coeff_cut;
		a0 = 1 + (alpha_cut / gain_lin_coeff_cut);
		a1 = -2 * cos(wc);
		a2 = 1 - (alpha_cut / gain_lin_coeff_cut);
	}

	
	coeffs[0] = b0 / a0;
	coeffs[1] = b1 / a0;
	coeffs[2] = b2 / a0;
	coeffs[3] = -1 * a1 / a0;
	coeffs[4] = -1 * a2 / a0;	
}


int main(void)
{
	/* Initialize the SAM system, run of default 8MHz clock ATM */
	SystemInit();
	
	/* Setup Board and peripherals */
	Init_Board();
	

	/* init filter */
	float coeffs_f32[5];
	calc_set_coeffs(coeffs_f32, Vfreq, Vampl);
	
	float bq_f32_state[4];

	arm_biquad_casd_df1_inst_f32 bq_f32;
	arm_biquad_cascade_df1_init_f32 (
									&bq_f32,		// biquad struct
									1,				// num stages
									coeffs_f32,		// ptr to coeffs (b0 b1 b2 a1 a2)
									bq_f32_state	// 4*numstages element biquad state buffer
									);
										
	float buf_f32[I2SC_BUFFSZ];
	float buf_add_f32[I2SC_BUFFSZ];
	
	int32_t *rx_buf, *tx_buf;
	
	float default_gain = pow(10.0, 0.88/20.0);
	
	new_ampl = 1;
	new_freq = 1;
	new_q = 1;
	new_direction = 1;
	
	while (1) {
	
		/* run dsp when there's new audio data */
		if (newdata) {
			
			if (PingPong == PING) {
				rx_buf = ReceiveBufR2;
				tx_buf = TransmitBufR2;
			}
			else { /* == PONG */
				rx_buf = ReceiveBufR1;
				tx_buf = TransmitBufR1;
			}

			//printf("rx_buf[0] %li \r\n", rx_buf[0]);
			
			if (FILTER_ON){	
				/* convert to float */
				arm_q31_to_float(rx_buf, buf_f32, I2SC_BUFFSZ);
				/* copy to addition buffer and apply default gain */
				for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
					buf_f32[i] = buf_add_f32[i] = buf_f32[i] * default_gain;
				}
				/* apply bandpass */
				arm_biquad_cascade_df1_f32(&bq_f32, buf_f32, buf_f32, I2SC_BUFFSZ);
				/* sum with original gained signal */
				for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
					buf_f32[i] += buf_add_f32[i];
				}
				/* convert back to q31 */
				arm_float_to_q31(buf_f32, tx_buf, I2SC_BUFFSZ);

			}
			else {	/* no dsp */		
				for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
					tx_buf[i] = rx_buf[i];
				}
			}
			
			newdata = 0;									
		}	
		
		/* calculate new coefficients when there's a new Vfreq */
		if (new_freq) {
			calc_set_coeffs(coeffs_f32, Vfreq, Vampl);
			new_freq = 0;
			printf("freq %g\r\n", Vfreq);
		}
		
		/* calculate new coefficients when there's a new Vampl */
		if (new_ampl) {
			calc_set_coeffs(coeffs_f32, Vfreq, Vampl);
			new_ampl = 0;
			printf("gain db %g\r\n", 20 * log((Vampl / 2000.0f) + 1.0f) / log(10));
		}
		
		if (new_q) {
			calc_set_coeffs(coeffs_f32, Vfreq, Vampl);
			new_q = 0;			
			printf("Q %g\r\n", Q);
		}
		
		if (new_direction){
			calc_set_coeffs(coeffs_f32, Vfreq, Vampl);
			new_direction = 0;
			printf("%s\r\n", boost ? "BOOST" : "CUT");
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
			I2SC0 -> I2SC_TNPR = (uint32_t)ReceiveBufL2; //transmit buffer 2
			I2SC0 -> I2SC_TNCR = I2SC_BUFFSZ;

			I2SC0 -> I2SC_RNPR = (uint32_t)ReceiveBufL2; //receive to buffer 2
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


static volatile char ubuf[6];
static volatile uint8_t i;

// USART1 RXRDY interrupt
void FLEXCOM1_Handler(void){
	
	char c = USART1 -> US_RHR;
	
	/* if id: put id at end of str (after \0) */
	if (c == 'a' || c == 'f' || c == 'q'){
		ubuf[5] = c;
		i = 0;
	}
	/* if a digit 0-9 */
	else if (c > 47 && c < 58 && i < 4){
		ubuf[i] = c;
		i++;
	}
	/* when terminating character received*/
	else if (c == 'k' && i > 0){
		ubuf[i] = '\0';
		
		if (ubuf[5] == 'a'){ 
			float Vampl_new = atoi(ubuf);
			if (abs(Vampl_new - Vampl) > MIN_V_DEV) { /* more than 20 mV diff in adc reading*/
				Vampl = Vampl_new;
				new_ampl = 1;
			}
					
		}
		else if (ubuf[5] == 'f'){
			float Vfreq_new = atoi(ubuf);
			if (abs(Vfreq_new - Vfreq) > MIN_V_DEV) { /* more than 20 mV diff in adc reading*/
				Vfreq = Vfreq_new;
				new_freq = 1;
			}
			
		}
		else if (ubuf[5] == 'q'){
			Q = atoi(ubuf);
			Q /= 1000.0f;
			new_q = 1;			
		}
				
		ubuf[5] = 0;
	}
	/* to flip boost and cut */
	else if (c == 'c'){
		boost = 0;
		new_direction = 1;
	}
	else if (c == 'b'){
		boost = 1;
		new_direction = 1;
	}
		
	USART1 -> US_CR |= US_CR_RSTSTA_Msk; // clears overrun error OVRE bit in receiver, which then clears RXRDY
}

