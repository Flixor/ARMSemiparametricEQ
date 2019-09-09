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
#include "arm_const_structs.h"
#include "IRdata.h"



#define I2SC_TXRDY_LED
#define TWI0_TXRDY_LED

#define PING	0
#define PONG	1
static volatile uint8_t newdata = 0;
static volatile uint8_t PingPong = PING;

#define CHANNEL_L	0
#define CHANNEL_R	1
uint8_t channelCount = CHANNEL_L;


/* Convolution*/
#define CONVOL_BLOCKSZ 2*IR_PARTSZ

#define FFT_FORWARD 0
#define FFT_INVERSE 1
#define FFT_BITREVERSE 1

const arm_cfft_instance_f32* S = &arm_cfft_sR_f32_len512;

float audioFftBufL0[CONVOL_BLOCKSZ] = { 0 };
float audioFftBufL1[CONVOL_BLOCKSZ] = { 0 };
float audioFftBufL2[CONVOL_BLOCKSZ] = { 0 };
float audioFftBufL3[CONVOL_BLOCKSZ] = { 0 };
float audioFftBufL4[CONVOL_BLOCKSZ] = { 0 };
float audioFftBufR0[CONVOL_BLOCKSZ] = { 0 };
float audioFftBufR1[CONVOL_BLOCKSZ] = { 0 };
float audioFftBufR2[CONVOL_BLOCKSZ] = { 0 };
float audioFftBufR3[CONVOL_BLOCKSZ] = { 0 };
float audioFftBufR4[CONVOL_BLOCKSZ] = { 0 };

float* audioFftBufferArrayL[NR_OF_BUFFS] = {audioFftBufL0, audioFftBufL1, audioFftBufL2, audioFftBufL3, audioFftBufL4};
float* audioFftBufferArrayR[NR_OF_BUFFS] = {audioFftBufR0, audioFftBufR1, audioFftBufR2, audioFftBufR3, audioFftBufR4};

#define CONV_TRIGGER 'g'
#define CONV_OFF 0
#define CONV_ON 1



/* I2S */
#define I2SC_BUFFSZ 256

int32_t ReceiveBufL1[I2SC_BUFFSZ] = { 0 };
int32_t ReceiveBufL2[I2SC_BUFFSZ] = { 0 };
int32_t ReceiveBufR1[I2SC_BUFFSZ] = { 0 };
int32_t ReceiveBufR2[I2SC_BUFFSZ] = { 0 };

int32_t TransmitBufL1[I2SC_BUFFSZ] = { 0 };
int32_t TransmitBufL2[I2SC_BUFFSZ] = { 0 };
int32_t TransmitBufR1[I2SC_BUFFSZ] = { 0 };
int32_t TransmitBufR2[I2SC_BUFFSZ] = { 0 };

float32_t transferBufInputL[I2SC_BUFFSZ] = {0};
float32_t transferBufInputR[I2SC_BUFFSZ] = {0};
float32_t transferBufOutputL[I2SC_BUFFSZ] = {0};
float32_t transferBufOutputR[I2SC_BUFFSZ] = {0};



/* Debug */
char floatPrintStr[40] = "%d:\t g:%g \t f:%f \t li:%li \r\n"; // args: {str, ctr, value, value, value}

#define CONVOLVE_L 0


/* Functions Calls */
static void Init_Board(void);
static uint8_t Init_Clock(void);


void timeDomR2C(float32_t* pRealSrc, float32_t* pComplDst, uint16_t lenReal) {
	for (int i = 0; i < lenReal; i++){
		pComplDst[i*2] = pRealSrc[i];
		pComplDst[(i*2)+1] = 0.0f;
	}
}

void timeDomC2R(float32_t* pComplSrc, float32_t* pRealDst, uint16_t lenReal) {
	// phase determination not necessary because post-IFFT should only be real values anyway...
	for (int i = 0; i < lenReal; i++){
		pRealDst[i] = pComplSrc[i*2];
	}
}

/* 
from:	{re(0), re(N/2), re(1), im(1), ..., re((N/2)-1), im ((N/2)-1)}
to:		{re(0), im(0), re(1), im(1), ..., re(N), im(N))}, so bin > N/2 are negative freqs with complex conjugate values of the positive freq bins, mirrored in the middle bin
		complex conjugate: a + bj --> a - bj
*/
void freqDomR2C(const float32_t* pRealSrc, float32_t* pComplDst, uint16_t lenReal) {
	
	pComplDst[0] = pRealSrc[0];
	pComplDst[1] = 0.0; // im(0) is always 0 (because DC)
	for (uint16_t bin = 2; bin < lenReal; bin += 2){
		pComplDst[bin] = pRealSrc[bin];
		pComplDst[bin+1] = pRealSrc[bin+1];
	}
	
	pComplDst[lenReal] = pRealSrc[1]; // re(N/2) was packed away here (cos im(0) is always 0.0)
	pComplDst[lenReal+1] = 0.0; // im(N/2) is also always 0.0
	// create conjugates
	for (uint16_t bin = 2; bin < lenReal; bin += 2){
		pComplDst[lenReal + bin] = pComplDst[lenReal - bin];
		pComplDst[lenReal + bin+1] = -(pComplDst[(lenReal - bin)+1]);
	}
}

/* (a + bj) * (c + dj) */
void complMul(float32_t a, float32_t b, float32_t c, float32_t d, float32_t* resultRe, float32_t* resultIm){
	*resultRe = a*c - b*d;
	*resultIm = b*c + a*d;
}

const double frac_int2float = 32768.0*65536.0 - 1.0;
float32_t int32_to_float(int32_t in){
	double dval = (double) in; // int32 -> double
	dval /= frac_int2float;
	return (float) dval; // double -> float
}
int32_t float_to_int32(float32_t fl){
	double dval = (double) fl; // float -> double
	dval *= frac_int2float;
	return (int32_t) dval; // double -> int32
}

void printConvState(uint8_t state){
	static uint8_t savedstate;
	if (savedstate < state){ // off -> on
		printf("Cassette player convolution ON\r\n");
	} else if (savedstate > state){ // on -> off
		printf("Cassette player convolution OFF\r\n");
	}
	savedstate = state;
}

void convolve(float32_t* inputAudioBuf, float32_t* outputAudioBuf, uint8_t channel){
		
	/* PROCESS INPUT */
	
	// static indexes, 1 per channel, will initialize as 0
	static uint8_t bufArrayIndex[2]; 
	
	// select audio fft buf array
	float** audioFftBufferArray;
	if (channel == CHANNEL_L){
		audioFftBufferArray = audioFftBufferArrayL;
	} else {
		audioFftBufferArray = audioFftBufferArrayR;		
	}
	
	// clear audio fft buf
	for (uint16_t i = 0; i < CONVOL_BLOCKSZ; i++){
		audioFftBufferArray[bufArrayIndex[channel]][i] = 0.0f;
	}
	// put complex timedom version of input audio in bufarray
	timeDomR2C(inputAudioBuf, audioFftBufferArray[bufArrayIndex[channel]], I2SC_BUFFSZ); 
		
	// complex fft inplace
	arm_cfft_f32(S, audioFftBufferArray[bufArrayIndex[channel]], FFT_FORWARD, FFT_BITREVERSE);

	// incr bufarray index
	bufArrayIndex[channel] = (bufArrayIndex[channel] + 1) % NR_OF_BUFFS;


	
	/* CONVOLUTION */
	
	float32_t convResultBuf[CONVOL_BLOCKSZ] = {0}; // 
	static float32_t overlapBufL[IR_PARTSZ]; // for saving of convolution result tail
	static float32_t overlapBufR[IR_PARTSZ]; // for saving of convolution result tail
	float32_t* overlapBufArray[2] = {overlapBufL, overlapBufR}; // one per channel

	
	int8_t IRbufIndex = NR_OF_BUFFS - 1;
	float32_t multResultBuf[CONVOL_BLOCKSZ];
	float32_t IRComplFftBuf[CONVOL_BLOCKSZ];
	
	while (IRbufIndex >= 0){

		freqDomR2C(IRFftBufferArray[IRbufIndex], IRComplFftBuf, IR_PARTSZ);

		for (uint16_t bin = 0; bin < CONVOL_BLOCKSZ; bin += 2){
			/* (a + bj) * (c + dj) = resultRe + resultIm*j */ 
			complMul(audioFftBufferArray[bufArrayIndex[channel]][bin],
					 audioFftBufferArray[bufArrayIndex[channel]][bin+1],
					 IRComplFftBuf[bin],
					 IRComplFftBuf[bin+1],
					 multResultBuf + bin,
					 multResultBuf + bin+1
					 );
					 
			convResultBuf[bin] += multResultBuf[bin];
			convResultBuf[bin+1] += multResultBuf[bin+1];
		}

		bufArrayIndex[channel] = (bufArrayIndex[channel] + 1) % NR_OF_BUFFS;
		IRbufIndex--;
	}
	
	
	// IFFT
	arm_cfft_f32(S, convResultBuf, FFT_INVERSE, FFT_BITREVERSE);
	
	// save convolution tail
	for (uint16_t sample = 0; sample < IR_PARTSZ; sample++){
		convResultBuf[sample] += overlapBufArray[channel][sample];
		overlapBufArray[channel][sample] = convResultBuf[IR_PARTSZ + sample];
	}
	
	// convert complex timedom back to real timedom
	// the tail that has been saved in overlapBuf will be cut off
	timeDomC2R(convResultBuf, outputAudioBuf, I2SC_BUFFSZ);

}



int main(void)
{
	/* Initialize the SAM system, run of default 8MHz clock ATM */
	SystemInit();
	
	/* Setup Board and peripherals */
	Init_Board();
	
	printf("Cassette recorder convolution OFF\r\n");
				
	while (1) {
	
		if (newdata) {
			if (PingPong == PING) {
				if (UART_Getchar() == CONV_TRIGGER){ // conv on
					printConvState(CONV_ON);

					for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
						if (CONVOLVE_L) { transferBufInputL[i] = int32_to_float(ReceiveBufL1[i]); }
						else			{ TransmitBufL2[i] = ReceiveBufL1[i]; }
						transferBufInputR[i] = int32_to_float(ReceiveBufR1[i]);
	
					}
					
					if (CONVOLVE_L) { convolve(transferBufInputL, transferBufOutputL, CHANNEL_L); }
					convolve(transferBufInputR, transferBufOutputR, CHANNEL_R);
					
					for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
						if (CONVOLVE_L) { TransmitBufL2[i] = float_to_int32(transferBufOutputL[i]); }
						TransmitBufR2[i] = float_to_int32(transferBufOutputR[i]);
					}
					
				} else { // conv off
					printConvState(CONV_OFF);
					
					for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
						TransmitBufL2[i] = ReceiveBufL1[i];
						TransmitBufR2[i] = ReceiveBufR2[i];
					}
				}
				
			} else { // == PONG
				if (UART_Getchar() == CONV_TRIGGER){  // conv on

					for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
						if (CONVOLVE_L) { transferBufInputL[i] = int32_to_float(ReceiveBufL2[i]); }
						else			{ TransmitBufL1[i] = ReceiveBufL2[i]; }
						transferBufInputR[i] = int32_to_float(ReceiveBufR2[i]);								
					}
					
					if (CONVOLVE_L) { convolve(transferBufInputL, transferBufOutputL, CHANNEL_L); }
					convolve(transferBufInputR, transferBufOutputR, CHANNEL_R);
					
					for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
						if (CONVOLVE_L) { TransmitBufL1[i] = float_to_int32(transferBufOutputL[i]); }
						TransmitBufR1[i] = float_to_int32(transferBufOutputR[i]);

					}
					
				} else { // conv off
					
					for (uint16_t i = 0; i < I2SC_BUFFSZ; i++){
						TransmitBufL1[i] = ReceiveBufL2[i];
						TransmitBufR1[i] = ReceiveBufR1[i];
					}
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


void I2SC0_Handler(void)
{
		
	if (channelCount == CHANNEL_L && (I2SC0 -> I2SC_SR & I2SC_SR_ENDTX) ) {

		if (PingPong == PING) {
			I2SC0 -> I2SC_TNPR = (uint32_t)TransmitBufL1; //transmit buffer 1
			I2SC0 -> I2SC_TNCR = I2SC_BUFFSZ;
			
			I2SC0 -> I2SC_RNPR = (uint32_t)ReceiveBufL1; //receive to buffer 1
			I2SC0 -> I2SC_RNCR = I2SC_BUFFSZ;

			} else { // PONG
			I2SC0 -> I2SC_TNPR = (uint32_t)TransmitBufL2;	//transmit buffer 2
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



